#include "physicssolver.h"
#include "obb.h"
#include "collision.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <functional>
#include <unordered_set>

// ============================================================================
// Construction
// ============================================================================

PhysicsSolver::PhysicsSolver() {
    floorBody.scale = glm::vec3(FLOOR_HALF_EXTENT * 2.0f, FLOOR_THICKNESS, FLOOR_HALF_EXTENT * 2.0f);
    floorBody.position = glm::vec3(0.0f, FLOOR_Y - FLOOR_THICKNESS * 0.5f, 0.0f);
    floorBody.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    floorBody.velocity = glm::vec3(0.0f);
    floorBody.angularVelocity = glm::vec3(0.0f);

    floorBody.inverseMass = 0.0f;
    floorBody.updateInertiaTensor();

    floorBody.restitution = 0.3f;
    floorBody.friction = 0.6f;

    // Static body used as the B-side for all plane contacts.
    planeBody.inverseMass = 0.0f;
    planeBody.inverseInertiaLocal = glm::mat3(0.0f);
    planeBody.inverseInertiaWorld = glm::mat3(0.0f);
    planeBody.velocity = glm::vec3(0.0f);
    planeBody.angularVelocity = glm::vec3(0.0f);
    planeBody.position = glm::vec3(0.0f);
    planeBody.restitution = 0.3f; // overridden per-plane at contact time
    planeBody.friction = 0.6f;    // overridden per-plane at contact time
}

PhysicsSolver::~PhysicsSolver() {}

// ============================================================================
// Integration (discrete, single body — retained for reference)
// ============================================================================

void PhysicsSolver::integrate(RigidBody& body, float deltaTime) {
    if (body.inverseMass == 0.0f) return;

    const glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    body.acceleration = gravity;
    body.velocity += body.acceleration * deltaTime;
    body.position += body.velocity * deltaTime;
    body.acceleration = glm::vec3(0.0f);

    if (body.inverseInertiaLocal != glm::mat3(0.0f)) {
        const glm::mat3 R = glm::mat3_cast(body.orientation);
        body.inverseInertiaWorld = R * body.inverseInertiaLocal * glm::transpose(R);

        body.angularVelocity += (body.inverseInertiaWorld * body.torque) * deltaTime;

        const glm::quat angVelQuat(0.0f, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z);
        body.orientation += (angVelQuat * body.orientation) * (0.5f * deltaTime);
        body.orientation = glm::normalize(body.orientation);

        body.torque = glm::vec3(0.0f);
    }
}

// ============================================================================
// Split integration used by the CCD sub-stepper (skips sleeping bodies)
// ============================================================================

void PhysicsSolver::applyGravity(RigidBody& body, float dt) const {
    if (body.inverseMass == 0.0f || body.asleep) return;
    const glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    body.velocity += gravity * dt;
}

void PhysicsSolver::integratePositions(std::vector<RigidBody>& bodies, float dt) {
    if (dt <= 0.0f) return;
    for (auto& body : bodies) {
        if (body.inverseMass == 0.0f || body.asleep) continue;

        body.position += body.velocity * dt;

        if (body.inverseInertiaLocal != glm::mat3(0.0f)) {
            // Angular-momentum-conserving integration:
            // 1. Compute L = I_world · ω (angular momentum, conserved quantity).
            // 2. Integrate orientation using current ω.
            // 3. Recompute I_world from new orientation.
            // 4. Recover ω = I_world_new⁻¹ · L.
            // This exactly conserves L for torque-free motion, including
            // asymmetric free precession (the Dzhanibekov effect).
            const glm::mat3 Rpre = glm::mat3_cast(body.orientation);
            const glm::mat3 IworldPre = Rpre * body.inertiaLocal * glm::transpose(Rpre);
            const glm::vec3 L = IworldPre * body.angularVelocity;

            const glm::quat angVelQuat(0.0f, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z);
            body.orientation += (angVelQuat * body.orientation) * (0.5f * dt);
            body.orientation = glm::normalize(body.orientation);

            const glm::mat3 R = glm::mat3_cast(body.orientation);
            body.inverseInertiaWorld = R * body.inverseInertiaLocal * glm::transpose(R);

            // Recover ω from conserved L using the updated inertia.
            body.angularVelocity = body.inverseInertiaWorld * L;
        }
    }
}

// ============================================================================
// Floor Contact Generation (rotation-aware, multi-point)
// ============================================================================

std::vector<CollisionInfo> PhysicsSolver::generateFloorContacts(const RigidBody& body) const {
    std::vector<CollisionInfo> contacts;

    if (body.inverseMass == 0.0f) return contacts;

    const float floorTopY = floorBody.position.y + floorBody.scale.y * 0.5f;

    // --- Sphere-floor: single contact point at bottom of sphere ---
    if (body.shape == ShapeType::Sphere) {
        const float bottomY = body.position.y - body.radius;
        if (bottomY >= floorTopY + SPECULATIVE_MARGIN) return contacts;

        CollisionInfo info;
        info.collided = true;
        info.point = glm::vec3(body.position.x, bottomY, body.position.z);
        info.penetration = floorTopY - bottomY; // >0 overlap, <0 gap within margin
        info.normal = glm::vec3(0.0f, -1.0f, 0.0f);
        info.featureId = 0x00FF00FFu; // unique stable ID for sphere-floor contact
        contacts.push_back(info);
        return contacts;
    }

    // --- Box-floor: rotation-aware multi-point (existing logic) ---
    const glm::vec3 halfSize = body.scale * 0.5f;
    const glm::vec3 localCorners[8] {
        {-halfSize.x, -halfSize.y, -halfSize.z}, { halfSize.x, -halfSize.y, -halfSize.z},
        { halfSize.x, -halfSize.y,  halfSize.z}, {-halfSize.x, -halfSize.y,  halfSize.z},
        {-halfSize.x,  halfSize.y, -halfSize.z}, { halfSize.x,  halfSize.y, -halfSize.z},
        { halfSize.x,  halfSize.y,  halfSize.z}, {-halfSize.x,  halfSize.y,  halfSize.z}
    };

    glm::vec3 worldCorners[8];
    float lowestY = 0.0f;

    for (int i = 0; i < 8; ++i) {
        worldCorners[i] = body.position + body.orientation * localCorners[i];
        if (i == 0 || worldCorners[i].y < lowestY) {
            lowestY = worldCorners[i].y;
        }
    }

    if (lowestY >= floorTopY + SPECULATIVE_MARGIN) return contacts; // beyond margin: no contact

    for (int i = 0; i < 8 && contacts.size() < 4; ++i) {
        const float y = worldCorners[i].y;
        if (y <= lowestY + FACE_CONTACT_EPSILON) {
            CollisionInfo info;
            info.collided = true;
            info.point = worldCorners[i];
            info.penetration = floorTopY - y;   // signed: >0 below floor, <0 just above
            info.normal = glm::vec3(0.0f, -1.0f, 0.0f);
            info.featureId = static_cast<uint32_t>(i);
            contacts.push_back(info);
        }
    }

    return contacts;
}

// ============================================================================
// Plane Contact Generation (arbitrary normal — slopes, walls, ramps)
// ============================================================================

std::vector<CollisionInfo> PhysicsSolver::generatePlaneContacts(const RigidBody& body, const StaticPlane& plane) const {
    std::vector<CollisionInfo> contacts;
    if (body.inverseMass == 0.0f) return contacts;

    const glm::vec3& n = plane.normal;

    // --- Finite plane bounds check ---
    // If halfExtent is non-zero, reject bodies whose projection onto the plane
    // falls outside the bounded rectangle.
    if (plane.halfExtent.x > 0.0f || plane.halfExtent.y > 0.0f) {
        // Compute tangent axes if not provided
        glm::vec3 t1 = plane.tangent1;
        glm::vec3 t2 = plane.tangent2;
        if (glm::dot(t1, t1) < 0.5f) {
            // Auto-generate tangent basis from normal
            glm::vec3 ref = (std::abs(n.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
            t1 = glm::normalize(glm::cross(n, ref));
            t2 = glm::cross(n, t1);
        }
        // Project body centre onto the plane's local 2D coordinates
        const glm::vec3 rel = body.position - plane.point;
        const float u = glm::dot(rel, t1);
        const float v = glm::dot(rel, t2);
        // Allow a margin of the body's radius/half-extent so edge contacts still work
        const float margin = (body.shape == ShapeType::Sphere) ? body.radius : glm::length(body.scale) * 0.5f;
        if (std::abs(u) > plane.halfExtent.x + margin || std::abs(v) > plane.halfExtent.y + margin) {
            return contacts; // body is outside the finite plane
        }
    }

    if (body.shape == ShapeType::Sphere) {
        // Sphere-plane: single contact at the point on the sphere closest to the plane.
        const float dist = glm::dot(body.position - plane.point, n);
        const float pen = body.radius - dist; // >0 overlap, <0 gap
        if (pen < -SPECULATIVE_MARGIN) return contacts;

        CollisionInfo info;
        info.collided = true;
        info.point = body.position - n * dist; // closest point on plane
        info.penetration = pen;
        info.normal = -n; // convention: points from body A toward body B (plane)
        info.featureId = 0xCC000000u;
        contacts.push_back(info);
    } else {
        // Box-plane: test all 8 corners against the plane.
        const glm::vec3 halfSize = body.scale * 0.5f;
        const glm::vec3 localCorners[8] {
            {-halfSize.x, -halfSize.y, -halfSize.z}, { halfSize.x, -halfSize.y, -halfSize.z},
            { halfSize.x, -halfSize.y,  halfSize.z}, {-halfSize.x, -halfSize.y,  halfSize.z},
            {-halfSize.x,  halfSize.y, -halfSize.z}, { halfSize.x,  halfSize.y, -halfSize.z},
            { halfSize.x,  halfSize.y,  halfSize.z}, {-halfSize.x,  halfSize.y,  halfSize.z}
        };

        // Find the minimum signed distance (deepest corner)
        float minDist = std::numeric_limits<float>::max();
        glm::vec3 worldCorners[8];
        for (int i = 0; i < 8; ++i) {
            worldCorners[i] = body.position + body.orientation * localCorners[i];
            const float d = glm::dot(worldCorners[i] - plane.point, n);
            if (d < minDist) minDist = d;
        }

        if (minDist >= SPECULATIVE_MARGIN) return contacts; // all corners far from plane

        // Retain corners within FACE_CONTACT_EPSILON of the deepest (up to 4)
        for (int i = 0; i < 8 && contacts.size() < 4; ++i) {
            const float d = glm::dot(worldCorners[i] - plane.point, n);
            if (d <= minDist + FACE_CONTACT_EPSILON) {
                CollisionInfo info;
                info.collided = true;
                info.point = worldCorners[i];
                info.penetration = -d; // >0 overlap (corner behind plane)
                info.normal = -n;      // points from body toward plane
                info.featureId = 0xCC000000u | static_cast<uint32_t>(i);
                contacts.push_back(info);
            }
        }
    }

    return contacts;
}

// ============================================================================
// Spatial-Hash Broadphase
// ============================================================================

namespace {
    inline uint64_t packCell(int x, int y, int z) {
        const uint64_t ux = static_cast<uint64_t>(x + 1048576) & 0x1FFFFF;
        const uint64_t uy = static_cast<uint64_t>(y + 1048576) & 0x1FFFFF;
        const uint64_t uz = static_cast<uint64_t>(z + 1048576) & 0x1FFFFF;
        return ux | (uy << 21) | (uz << 42);
    }
}

void PhysicsSolver::buildBroadphasePairs(const std::vector<RigidBody>& bodies,
                                         const std::vector<glm::vec3>& aabbMin,
                                         const std::vector<glm::vec3>& aabbMax,
                                         std::vector<std::pair<int, int>>& outPairs) const {
    const float inv = 1.0f / SPATIAL_CELL_SIZE;

    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(bodies.size() * 2);

    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        const int minx = static_cast<int>(std::floor(aabbMin[i].x * inv));
        const int miny = static_cast<int>(std::floor(aabbMin[i].y * inv));
        const int minz = static_cast<int>(std::floor(aabbMin[i].z * inv));
        const int maxx = static_cast<int>(std::floor(aabbMax[i].x * inv));
        const int maxy = static_cast<int>(std::floor(aabbMax[i].y * inv));
        const int maxz = static_cast<int>(std::floor(aabbMax[i].z * inv));

        for (int cz = minz; cz <= maxz; ++cz)
            for (int cy = miny; cy <= maxy; ++cy)
                for (int cx = minx; cx <= maxx; ++cx)
                    grid[packCell(cx, cy, cz)].push_back(i);
    }

    std::unordered_set<uint64_t> seen;
    seen.reserve(bodies.size() * 4);

    for (const auto& cell : grid) {
        const std::vector<int>& ids = cell.second;
        for (std::size_t a = 0; a < ids.size(); ++a) {
            for (std::size_t b = a + 1; b < ids.size(); ++b) {
                int i = ids[a];
                int j = ids[b];
                if (i > j) std::swap(i, j);
                const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(i)) << 32)
                                   | static_cast<uint32_t>(j);
                if (seen.insert(key).second) {
                    outPairs.emplace_back(i, j);
                }
            }
        }
    }
}

// ============================================================================
// Contact Precomputation
// ============================================================================

void PhysicsSolver::precomputeContact(Contact& c) {
    c.rA = c.info.point - c.a->position;
    c.rB = c.info.point - c.b->position;

    const glm::vec3& n = c.info.normal;

    glm::vec3 ref = (std::abs(n.x) < 0.9f) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    c.tangent1 = glm::normalize(glm::cross(n, ref));
    c.tangent2 = glm::cross(n, c.tangent1);

    auto computeEffectiveMass = [&](const glm::vec3& dir) -> float {
        const glm::vec3 rAxDir = glm::cross(c.rA, dir);
        const glm::vec3 rBxDir = glm::cross(c.rB, dir);
        const float angularA = glm::dot(dir, glm::cross(c.a->inverseInertiaWorld * rAxDir, c.rA));
        const float angularB = glm::dot(dir, glm::cross(c.b->inverseInertiaWorld * rBxDir, c.rB));
        const float denom = c.a->inverseMass + c.b->inverseMass + angularA + angularB;
        return (denom > 1e-10f) ? (1.0f / denom) : 0.0f;
    };

    c.effectiveMassNormal = computeEffectiveMass(n);
    c.effectiveMassTangent1 = computeEffectiveMass(c.tangent1);
    c.effectiveMassTangent2 = computeEffectiveMass(c.tangent2);

    const glm::vec3 velAtA = c.a->velocity + glm::cross(c.a->angularVelocity, c.rA);
    const glm::vec3 velAtB = c.b->velocity + glm::cross(c.b->angularVelocity, c.rB);
    c.initialRelVelN = glm::dot(velAtB - velAtA, n);
}

// ============================================================================
// Contact Cache: Matching & Warm Starting
// ============================================================================

void PhysicsSolver::matchAndLoadCache(std::vector<Contact>& contacts) {
    for (auto& c : contacts) {
        c.accumulatedNormalImpulse = 0.0f;
        c.accumulatedTangentImpulse1 = 0.0f;
        c.accumulatedTangentImpulse2 = 0.0f;

        const ContactKey key{ c.a, c.b, c.info.featureId };
        auto it = contactCache.find(key);
        if (it != contactCache.end()) {
            c.accumulatedNormalImpulse   = it->second.normal   * WARM_START_SCALE;
            c.accumulatedTangentImpulse1 = it->second.tangent1 * WARM_START_SCALE;
            c.accumulatedTangentImpulse2 = it->second.tangent2 * WARM_START_SCALE;
        }
    }
}

void PhysicsSolver::warmStart(std::vector<Contact>& contacts) {
    for (const auto& c : contacts) {
        const glm::vec3 impulse = c.accumulatedNormalImpulse * c.info.normal
                                + c.accumulatedTangentImpulse1 * c.tangent1
                                + c.accumulatedTangentImpulse2 * c.tangent2;

        c.a->velocity -= impulse * c.a->inverseMass;
        c.b->velocity += impulse * c.b->inverseMass;

        c.a->angularVelocity -= c.a->inverseInertiaWorld * glm::cross(c.rA, impulse);
        c.b->angularVelocity += c.b->inverseInertiaWorld * glm::cross(c.rB, impulse);
    }
}

void PhysicsSolver::storeCache(const std::vector<Contact>& contacts) {
    contactCache.clear();
    contactCache.reserve(contacts.size() * 2);

    for (const auto& c : contacts) {
        const ContactKey key{ c.a, c.b, c.info.featureId };
        contactCache[key] = CachedImpulse{
            c.accumulatedNormalImpulse,
            c.accumulatedTangentImpulse1,
            c.accumulatedTangentImpulse2
        };
    }
}

// ============================================================================
// Velocity Solver (restitution + friction; penetration via split-impulse)
// ============================================================================

void PhysicsSolver::solveVelocities(std::vector<Contact>& contacts, bool reverse) {
    const int count = static_cast<int>(contacts.size());
    for (int idx = 0; idx < count; ++idx) {
        Contact& c = contacts[reverse ? (count - 1 - idx) : idx];
        const glm::vec3 velAtA = c.a->velocity + glm::cross(c.a->angularVelocity, c.rA);
        const glm::vec3 velAtB = c.b->velocity + glm::cross(c.b->angularVelocity, c.rB);
        const glm::vec3 dv = velAtB - velAtA;

        const float vn = glm::dot(dv, c.info.normal);

        // Speculative non-penetration + restitution.
        //   gap (penetration < 0):  permit approach only up to just touching
        //                           this step (targetVn = penetration/dt, negative).
        //   touching/overlap (>=0): for RESTING contacts, permit no approach (0).
        //                           for IMPACT contacts, demand the bounce velocity.
        // Existing overlap is removed by the energy-free position solve below.
        float targetVn = std::min(c.info.penetration, 0.0f) / FIXED_DT;

        // Restitution: only for genuine impacts (high closing speed).
        // The REST_THRESHOLD gate (0.5 m/s) already prevents bounce at resting
        // contacts, so no additional penetration gate is needed.
        const float e = std::min(c.a->restitution, c.b->restitution);
        if (c.initialRelVelN < -REST_THRESHOLD) {
            const float bounceTarget = -e * c.initialRelVelN;
            targetVn = std::max(targetVn, bounceTarget);
        }

        float lambda = c.effectiveMassNormal * (targetVn - vn);

        const float oldAccumN = c.accumulatedNormalImpulse;
        c.accumulatedNormalImpulse = std::max(0.0f, oldAccumN + lambda);
        lambda = c.accumulatedNormalImpulse - oldAccumN;

        const glm::vec3 normalImpulse = lambda * c.info.normal;
        c.a->velocity -= normalImpulse * c.a->inverseMass;
        c.b->velocity += normalImpulse * c.b->inverseMass;
        c.a->angularVelocity -= c.a->inverseInertiaWorld * glm::cross(c.rA, normalImpulse);
        c.b->angularVelocity += c.b->inverseInertiaWorld * glm::cross(c.rB, normalImpulse);


        // Friction — Tangent 1
        {
            const glm::vec3 vA = c.a->velocity + glm::cross(c.a->angularVelocity, c.rA);
            const glm::vec3 vB = c.b->velocity + glm::cross(c.b->angularVelocity, c.rB);
            const float vt1 = glm::dot(vB - vA, c.tangent1);
            float lambdaT1 = c.effectiveMassTangent1 * (-vt1);

            const float mu = std::sqrt(c.a->friction * c.b->friction);
            const float maxFriction = mu * c.accumulatedNormalImpulse;

            const float oldT1 = c.accumulatedTangentImpulse1;
            c.accumulatedTangentImpulse1 = std::clamp(oldT1 + lambdaT1, -maxFriction, maxFriction);
            lambdaT1 = c.accumulatedTangentImpulse1 - oldT1;

            const glm::vec3 fi = lambdaT1 * c.tangent1;
            c.a->velocity -= fi * c.a->inverseMass;
            c.b->velocity += fi * c.b->inverseMass;
            c.a->angularVelocity -= c.a->inverseInertiaWorld * glm::cross(c.rA, fi);
            c.b->angularVelocity += c.b->inverseInertiaWorld * glm::cross(c.rB, fi);
        }

        // Friction — Tangent 2
        {
            const glm::vec3 vA = c.a->velocity + glm::cross(c.a->angularVelocity, c.rA);
            const glm::vec3 vB = c.b->velocity + glm::cross(c.b->angularVelocity, c.rB);
            const float vt2 = glm::dot(vB - vA, c.tangent2);
            float lambdaT2 = c.effectiveMassTangent2 * (-vt2);

            const float mu = std::sqrt(c.a->friction * c.b->friction);
            const float maxFriction = mu * c.accumulatedNormalImpulse;

            const float oldT2 = c.accumulatedTangentImpulse2;
            c.accumulatedTangentImpulse2 = std::clamp(oldT2 + lambdaT2, -maxFriction, maxFriction);
            lambdaT2 = c.accumulatedTangentImpulse2 - oldT2;

            const glm::vec3 fi = lambdaT2 * c.tangent2;
            c.a->velocity -= fi * c.a->inverseMass;
            c.b->velocity += fi * c.b->inverseMass;
            c.a->angularVelocity -= c.a->inverseInertiaWorld * glm::cross(c.rA, fi);
            c.b->angularVelocity += c.b->inverseInertiaWorld * glm::cross(c.rB, fi);
        }
    }
}

// ============================================================================
// Split-Impulse Position Solver (energy-neutral penetration correction)
// ============================================================================

void PhysicsSolver::solvePositions(std::vector<Contact>& contacts, bool reverse) {
    const int count = static_cast<int>(contacts.size());
    for (int idx = 0; idx < count; ++idx) {
        Contact& c = contacts[reverse ? (count - 1 - idx) : idx];
        const float excess = c.info.penetration - PENETRATION_SLOP;
        if (excess <= 0.0f) continue;

        float bias = (POSITION_BETA / FIXED_DT) * excess;
        if (bias > MAX_CORRECTION_SPEED) bias = MAX_CORRECTION_SPEED; // gentle, no violent shove

        const glm::vec3 vpA = c.a->pseudoLinearVel + glm::cross(c.a->pseudoAngularVel, c.rA);
        const glm::vec3 vpB = c.b->pseudoLinearVel + glm::cross(c.b->pseudoAngularVel, c.rB);
        const float vpn = glm::dot(vpB - vpA, c.info.normal);

        float lambda = c.effectiveMassNormal * (bias - vpn);

        const float oldP = c.accumulatedPositionImpulse;
        c.accumulatedPositionImpulse = std::max(0.0f, oldP + lambda);
        lambda = c.accumulatedPositionImpulse - oldP;

        const glm::vec3 P = lambda * c.info.normal;
        c.a->pseudoLinearVel  -= P * c.a->inverseMass;
        c.b->pseudoLinearVel  += P * c.b->inverseMass;
        c.a->pseudoAngularVel -= c.a->inverseInertiaWorld * glm::cross(c.rA, P);
        c.b->pseudoAngularVel += c.b->inverseInertiaWorld * glm::cross(c.rB, P);
    }
}

void PhysicsSolver::integratePseudoVelocities(std::vector<RigidBody>& bodies) {
    for (auto& body : bodies) {
        if (body.inverseMass == 0.0f || body.asleep) continue;

        // Cap total pseudo-velocity so the combined position correction from
        // multiple simultaneous contacts cannot inject unbounded PE. This
        // prevents the transient energy rise during manifold transitions
        // (e.g. edge→face where 4 new contacts each push upward independently).
        const float pvLen = glm::length(body.pseudoLinearVel);
        if (pvLen > MAX_CORRECTION_SPEED) {
            body.pseudoLinearVel *= (MAX_CORRECTION_SPEED / pvLen);
        }

        body.position += body.pseudoLinearVel * FIXED_DT;

        const glm::vec3& pa = body.pseudoAngularVel;
        if (glm::dot(pa, pa) > 0.0f && body.inverseInertiaLocal != glm::mat3(0.0f)) {
            const glm::quat q(0.0f, pa.x, pa.y, pa.z);
            body.orientation += (q * body.orientation) * (0.5f * FIXED_DT);
            body.orientation = glm::normalize(body.orientation);

            const glm::mat3 R = glm::mat3_cast(body.orientation);
            body.inverseInertiaWorld = R * body.inverseInertiaLocal * glm::transpose(R);
        }
    }
}

// ============================================================================
// Discrete detection + resolution (single narrowphase pass, sleep-aware)
// ============================================================================

void PhysicsSolver::detectAndResolve(std::vector<RigidBody>& bodies) {
    for (auto& body : bodies) {
        body.isColliding = false;
        body.pseudoLinearVel = glm::vec3(0.0f);
        body.pseudoAngularVel = glm::vec3(0.0f);
    }
    floorBody.pseudoLinearVel = glm::vec3(0.0f);
    floorBody.pseudoAngularVel = glm::vec3(0.0f);

    const std::size_t n = bodies.size();
    islandEdges.clear();
    std::vector<Contact> contacts;

    std::vector<OBB> obbs(n);
    std::vector<glm::vec3> aabbMin(n), aabbMax(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (bodies[i].shape == ShapeType::Sphere) {
            // Sphere AABB: center ± radius
            const glm::vec3 r(bodies[i].radius);
            aabbMin[i] = bodies[i].position - r;
            aabbMax[i] = bodies[i].position + r;
            // OBB not used for spheres but fill it to avoid uninitialized reads
            obbs[i].center = bodies[i].position;
            obbs[i].halfExtents = r;
            obbs[i].axes = glm::mat3(1.0f);
        } else {
            obbs[i] = OBB::fromRigidBody(bodies[i]);
            const glm::vec3 ext = glm::abs(obbs[i].axes[0]) * obbs[i].halfExtents.x
                                + glm::abs(obbs[i].axes[1]) * obbs[i].halfExtents.y
                                + glm::abs(obbs[i].axes[2]) * obbs[i].halfExtents.z;
            aabbMin[i] = obbs[i].center - ext;
            aabbMax[i] = obbs[i].center + ext;
        }
    }

    std::vector<std::pair<int, int>> candidatePairs;
    buildBroadphasePairs(bodies, aabbMin, aabbMax, candidatePairs);
    std::sort(candidatePairs.begin(), candidatePairs.end()); // deterministic order for stable stacks

    // Single narrowphase pass — dispatch based on shape types. Build island
    // edges + wake from collision results, then gather contacts for non-sleeping
    // pairs.
    struct PendingContact { int i; int j; CollisionInfo info; };
    std::vector<PendingContact> pendingContacts;
    pendingContacts.reserve(candidatePairs.size() * 2);
    std::unordered_set<int> islandsToWake;

    for (const auto& pr : candidatePairs) {
        const int i = pr.first;
        const int j = pr.second;

        if (aabbMin[i].x > aabbMax[j].x || aabbMax[i].x < aabbMin[j].x) continue;
        if (aabbMin[i].y > aabbMax[j].y || aabbMax[i].y < aabbMin[j].y) continue;
        if (aabbMin[i].z > aabbMax[j].z || aabbMax[i].z < aabbMin[j].z) continue;

        // Determine shape pair and run appropriate narrowphase
        const ShapeType shapeI = bodies[i].shape;
        const ShapeType shapeJ = bodies[j].shape;
        bool colliding = false;
        std::vector<CollisionInfo> pairContacts;

        if (shapeI == ShapeType::Box && shapeJ == ShapeType::Box) {
            // Box-Box: OBB-SAT
            const SATResult sat = Collision::testOBB(obbs[i], obbs[j]);
            if (sat.colliding) {
                colliding = true;
                pairContacts = Collision::generateManifold(obbs[i], obbs[j], sat);
            }
        } else if (shapeI == ShapeType::Sphere && shapeJ == ShapeType::Sphere) {
            // Sphere-Sphere
            CollisionInfo ci = Collision::testSphereSphere(
                bodies[i].position, bodies[i].radius,
                bodies[j].position, bodies[j].radius);
            if (ci.collided) { colliding = true; pairContacts.push_back(ci); }
        } else {
            // Sphere-Box (ensure sphere is "A", box is "B" for consistent normal)
            const int si = (shapeI == ShapeType::Sphere) ? i : j;
            const int bi = (shapeI == ShapeType::Sphere) ? j : i;
            CollisionInfo ci = Collision::testSphereOBB(
                bodies[si].position, bodies[si].radius, obbs[bi]);
            if (ci.collided) {
                colliding = true;
                // If we swapped order, flip the normal (it points A->B in our convention)
                if (si != i) ci.normal = -ci.normal;
                pairContacts.push_back(ci);
            }
        }

        if (!colliding) continue;

        const bool dynI = bodies[i].inverseMass > 0.0f;
        const bool dynJ = bodies[j].inverseMass > 0.0f;
        if (dynI && dynJ) {
            islandEdges.emplace_back(i, j);
            const bool asleepI = bodies[i].asleep;
            const bool asleepJ = bodies[j].asleep;
            if (asleepI && !asleepJ) islandsToWake.insert(bodies[i].islandId);
            else if (asleepJ && !asleepI) islandsToWake.insert(bodies[j].islandId);
        }

        for (const auto& ci : pairContacts) {
            pendingContacts.push_back({ i, j, ci });
        }
    }

    for (int id : islandsToWake) wakeIsland(bodies, id);

    for (const auto& pc : pendingContacts) {
        if (bodies[pc.i].asleep && bodies[pc.j].asleep) continue; // both sleeping: no solve
        contacts.push_back({&bodies[pc.i], &bodies[pc.j], pc.info,
                            glm::vec3(0.0f), glm::vec3(0.0f),
                            0.0f, 0.0f, 0.0f,
                            glm::vec3(0.0f), glm::vec3(0.0f),
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    }

    // Body-floor contacts (awake bodies only)
    for (auto& body : bodies) {
        if (body.asleep) continue;
        for (const CollisionInfo& floorContact : generateFloorContacts(body)) {
            contacts.push_back({&body, &floorBody, floorContact,
                                glm::vec3(0.0f), glm::vec3(0.0f),
                                0.0f, 0.0f, 0.0f,
                                glm::vec3(0.0f), glm::vec3(0.0f),
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
        }
    }

    // Body-plane contacts (arbitrary slopes/walls)
    for (const auto& plane : planes) {
        // Set planeBody material to match this plane's properties
        planeBody.restitution = plane.restitution;
        planeBody.friction = plane.friction;
        planeBody.position = plane.point;

        for (auto& body : bodies) {
            if (body.asleep) continue;
            for (const CollisionInfo& pc : generatePlaneContacts(body, plane)) {
                contacts.push_back({&body, &planeBody, pc,
                                    glm::vec3(0.0f), glm::vec3(0.0f),
                                    0.0f, 0.0f, 0.0f,
                                    glm::vec3(0.0f), glm::vec3(0.0f),
                                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
            }
        }
    }

    lastContactCount = static_cast<int>(contacts.size());

    for (const auto& c : contacts) {
        if (c.b == &floorBody || c.b == &planeBody) continue;

        const glm::vec3 rA = c.info.point - c.a->position;
        const glm::vec3 rB = c.info.point - c.b->position;
        const glm::vec3 velAtA = c.a->velocity + glm::cross(c.a->angularVelocity, rA);
        const glm::vec3 velAtB = c.b->velocity + glm::cross(c.b->angularVelocity, rB);
        const float vRelN = glm::dot(velAtB - velAtA, c.info.normal);

        if (vRelN < -REST_THRESHOLD) {
            c.a->isColliding = true;
            c.b->isColliding = true;
        }
    }

    if (contacts.empty()) {
        contactCache.clear();
        return;
    }

    for (auto& c : contacts) {
        precomputeContact(c);
    }

    // Shock propagation ordering: solve lowest contacts first. In a stack the
    // support load must travel from the ground up; a Gauss-Seidel sweep that
    // visits the base contact before the ones above it propagates that support
    // in a single pass, so a tall stack converges instead of slowly sinking and
    // leaning (which then injects energy and topples). Ordering only changes
    // convergence rate, not the converged solution -- momentum is preserved.
    std::sort(contacts.begin(), contacts.end(),
              [](const Contact& x, const Contact& y) {
                  return x.info.point.y < y.info.point.y;
              });

    matchAndLoadCache(contacts);
    warmStart(contacts);

    // Symmetric Gauss-Seidel: alternate the sweep direction every iteration.
    // A fixed sweep order gives the earliest-solved contacts a persistent
    // advantage, which injects a consistent directional bias -- in a symmetric
    // stack that bias seeds a lean that gravity then amplifies into a topple.
    // Alternating the order cancels the bias while applying identical impulses
    // (momentum-preserving; no damping/snapping).
    for (int iter = 0; iter < solverIterations; ++iter) {
        solveVelocities(contacts, (iter & 1) != 0);
    }

    // Energy-free penetration removal (split impulse). Operates on pseudo-
    // velocities that are integrated into position only, so correcting overlap
    // never changes a body's real momentum -> no bounce, no rocking at rest.
    for (int iter = 0; iter < positionIterations; ++iter) {
        solvePositions(contacts, (iter & 1) != 0);
    }
    integratePseudoVelocities(bodies);

    if (captureDiagnostics) {
        lastSolvedContacts.clear();
        lastSolvedContacts.reserve(contacts.size());
        for (const auto& c : contacts) {
            ContactDebug d;
            d.a = c.a;
            d.b = c.b;
            d.point = c.info.point;
            d.normal = c.info.normal;
            d.penetration = c.info.penetration;
            d.normalImpulse = c.accumulatedNormalImpulse;
            d.frictionImpulse = std::sqrt(c.accumulatedTangentImpulse1 * c.accumulatedTangentImpulse1
                                        + c.accumulatedTangentImpulse2 * c.accumulatedTangentImpulse2);
            d.floorContact = (c.b == &floorBody || c.b == &planeBody);
            lastSolvedContacts.push_back(d);
        }
    }

    storeCache(contacts);
}

// ============================================================================
// Continuous Collision Detection (Conservative Advancement)
// ============================================================================

OBB PhysicsSolver::predictOBB(const RigidBody& b, float t) {
    OBB o;
    o.center = b.position + b.velocity * t;

    glm::quat q = b.orientation;
    const glm::quat wq(0.0f, b.angularVelocity.x, b.angularVelocity.y, b.angularVelocity.z);
    q = q + (wq * q) * (0.5f * t);
    q = glm::normalize(q);

    o.halfExtents = b.scale * 0.5f;
    o.axes = glm::mat3_cast(q);
    return o;
}

bool PhysicsSolver::isCCDCandidate(const RigidBody& b, float dt) const {
    if (b.inverseMass == 0.0f || b.asleep) return false;
    const float minExtent = (b.shape == ShapeType::Sphere)
        ? b.radius * 2.0f
        : std::min(std::min(b.scale.x, b.scale.y), b.scale.z);
    const float disp = glm::length(b.velocity) * dt;
    return disp > CCD_MOTION_FACTOR * minExtent;
}

PhysicsSolver::TOIResult PhysicsSolver::computePairTOI(const RigidBody& A, const RigidBody& B, float dt) const {
    TOIResult res;

    const float rmaxA = glm::length(A.scale * 0.5f);
    const float rmaxB = glm::length(B.scale * 0.5f);
    const float angBound = glm::length(A.angularVelocity) * rmaxA + glm::length(B.angularVelocity) * rmaxB;

    float t = 0.0f;
    for (int iter = 0; iter < CCD_MAX_ITERATIONS; ++iter) {
        const OBB oa = predictOBB(A, t);
        const OBB ob = predictOBB(B, t);
        const DistanceResult dr = Collision::distanceOBB(oa, ob);

        if (dr.distance < CCD_TOLERANCE) {
            if (iter == 0) return res;
            res.hit = true;
            res.toi = t;
            res.closingSpeed = std::max(1e-4f, glm::dot(A.velocity - B.velocity, dr.normal) + angBound);
            return res;
        }

        const float closing = glm::dot(A.velocity - B.velocity, dr.normal) + angBound;
        if (closing <= 1e-6f) return res;

        t += dr.distance / closing;
        if (t >= dt) return res;
    }

    const OBB oa = predictOBB(A, t);
    const OBB ob = predictOBB(B, t);
    const DistanceResult dr = Collision::distanceOBB(oa, ob);
    res.hit = true;
    res.toi = t;
    res.closingSpeed = std::max(1e-4f, glm::dot(A.velocity - B.velocity, dr.normal) + angBound);
    return res;
}

PhysicsSolver::TOIResult PhysicsSolver::computeFloorTOI(const RigidBody& B, float dt) const {
    TOIResult res;

    const float floorTopY = floorBody.position.y + floorBody.scale.y * 0.5f;
    const float rmax = glm::length(B.scale * 0.5f);
    const float angBound = glm::length(B.angularVelocity) * rmax;

    float t = 0.0f;
    for (int iter = 0; iter < CCD_MAX_ITERATIONS; ++iter) {
        const OBB o = predictOBB(B, t);
        glm::vec3 corners[8];
        o.getCorners(corners);
        float lowestY = corners[0].y;
        for (int k = 1; k < 8; ++k) lowestY = std::min(lowestY, corners[k].y);
        const float d = lowestY - floorTopY;

        if (d < CCD_TOLERANCE) {
            if (iter == 0) return res;
            res.hit = true;
            res.toi = t;
            res.closingSpeed = std::max(1e-4f, -B.velocity.y + angBound);
            return res;
        }

        const float closing = -B.velocity.y + angBound;
        if (closing <= 1e-6f) return res;

        t += d / closing;
        if (t >= dt) return res;
    }

    res.hit = true;
    res.toi = t;
    res.closingSpeed = std::max(1e-4f, -B.velocity.y + angBound);
    return res;
}

PhysicsSolver::TOIResult PhysicsSolver::findEarliestTOI(const std::vector<RigidBody>& bodies, float dt) const {
    TOIResult best;
    best.hit = false;
    best.toi = dt;

    const std::size_t n = bodies.size();
    if (n == 0) return best;

    std::vector<char> candidate(n);
    std::vector<glm::vec3> sweptMin(n), sweptMax(n);
    bool anyCandidate = false;

    for (std::size_t i = 0; i < n; ++i) {
        candidate[i] = isCCDCandidate(bodies[i], dt) ? 1 : 0;
        if (candidate[i]) anyCandidate = true;

        const OBB o = OBB::fromRigidBody(bodies[i]);
        const glm::vec3 ext = glm::abs(o.axes[0]) * o.halfExtents.x
                            + glm::abs(o.axes[1]) * o.halfExtents.y
                            + glm::abs(o.axes[2]) * o.halfExtents.z;

        const glm::vec3 disp = bodies[i].velocity * dt;
        const float rmax = glm::length(bodies[i].scale * 0.5f);
        const float skin = glm::length(bodies[i].angularVelocity) * rmax * dt
                         + CCD_TOLERANCE + ISLAND_CONTACT_MARGIN;

        glm::vec3 lo = o.center - ext + glm::min(disp, glm::vec3(0.0f));
        glm::vec3 hi = o.center + ext + glm::max(disp, glm::vec3(0.0f));
        lo -= glm::vec3(skin);
        hi += glm::vec3(skin);
        sweptMin[i] = lo;
        sweptMax[i] = hi;
    }

    if (!anyCandidate) return best;

    for (std::size_t i = 0; i < n; ++i) {
        if (!candidate[i]) continue;
        const TOIResult tf = computeFloorTOI(bodies[i], dt);
        if (tf.hit && tf.toi < best.toi) best = tf;
    }

    std::vector<std::pair<int, int>> pairs;
    buildBroadphasePairs(bodies, sweptMin, sweptMax, pairs);

    for (const auto& pr : pairs) {
        const int i = pr.first;
        const int j = pr.second;
        if (!candidate[i] && !candidate[j]) continue;

        const TOIResult tp = computePairTOI(bodies[i], bodies[j], dt);
        if (tp.hit && tp.toi < best.toi) best = tp;
    }

    return best;
}

// ============================================================================
// Sleeping / Islands
// ============================================================================

void PhysicsSolver::wakeIsland(std::vector<RigidBody>& bodies, int islandId) {
    if (islandId < 0) return;
    for (auto& b : bodies) {
        if (b.islandId == islandId && b.asleep) {
            b.asleep = false;
            b.sleepTimer = 0.0f;
        }
    }
}

void PhysicsSolver::updateSleeping(std::vector<RigidBody>& bodies, float dt) {
    const int n = static_cast<int>(bodies.size());
    if (n == 0) return;

    std::vector<int> parent(n);
    for (int i = 0; i < n; ++i) parent[i] = i;

    std::function<int(int)> find = [&](int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite = [&](int a, int b) { parent[find(a)] = find(b); };

    for (const auto& e : islandEdges) unite(e.first, e.second);

    const float linSq = SLEEP_LINEAR_THRESHOLD * SLEEP_LINEAR_THRESHOLD;
    const float angSq = SLEEP_ANGULAR_THRESHOLD * SLEEP_ANGULAR_THRESHOLD;

    for (int i = 0; i < n; ++i) {
        RigidBody& b = bodies[i];
        if (b.inverseMass == 0.0f) { b.islandId = -1; continue; }

        const bool slow = glm::dot(b.velocity, b.velocity) < linSq
               && glm::dot(b.angularVelocity, b.angularVelocity) < angSq;
        if (slow) b.sleepTimer += dt;
        else      b.sleepTimer = std::max(0.0f, b.sleepTimer - SLEEP_DECAY_RATE * dt);

    }

    std::unordered_map<int, float> islandMinTimer;
    for (int i = 0; i < n; ++i) {
        if (bodies[i].inverseMass == 0.0f) continue;
        const int r = find(i);
        const float t = bodies[i].sleepTimer;
        auto it = islandMinTimer.find(r);
        if (it == islandMinTimer.end()) islandMinTimer[r] = t;
        else it->second = std::min(it->second, t);
    }

    for (int i = 0; i < n; ++i) {
        RigidBody& b = bodies[i];
        if (b.inverseMass == 0.0f) continue;

        const int r = find(i);
        b.islandId = r;

        if (islandMinTimer[r] >= SLEEP_TIME) {
            if (!b.asleep) {
                b.asleep = true;
                b.velocity = glm::vec3(0.0f);
                b.angularVelocity = glm::vec3(0.0f);
            }
        } else {
            b.asleep = false;
        }
    }
}

// ============================================================================
// Full CCD-aware + sleeping step
// ============================================================================

void PhysicsSolver::step(std::vector<RigidBody>& bodies, float dt) {
    using clock = std::chrono::high_resolution_clock;

    double ccdMs = 0.0;    // time inside findEarliestTOI (the CCD search)
    double solveMs = 0.0;  // time inside detectAndResolve (broadphase + narrowphase + solve)
    int substeps = 0;

    if (gravityEnabled) {
        for (auto& b : bodies) applyGravity(b, dt);
    }

    // (1) Resolve contact constraints at the CURRENT configuration BEFORE moving.
    // Symplectic order: we integrate the *constrained* velocity, not the raw
    // gravity-loaded one. On a resting contact the normal + friction constraints
    // drive the relative velocity to zero here, so the subsequent integration
    // produces (almost) no motion. Solving AFTER integrating instead let every
    // body slide tangentially by ~g*sin(theta)*dt^2 each frame before friction
    // could act -- harmless on flat contacts, but a permanent creep on the
    // spiral's leaning (slanted-normal) pile. This is the standard Box2D/Bullet
    // ordering and is what makes resting piles actually stop.
    {
        const auto tSolve0 = clock::now();
        detectAndResolve(bodies);
        const auto tSolve1 = clock::now();
        solveMs += std::chrono::duration<double, std::milli>(tSolve1 - tSolve0).count();
    }

    // (2) Advance positions with continuous collision detection. Any new impact
    // uncovered part-way through the step is resolved before continuing so fast
    // bodies still cannot tunnel.
    float remaining = dt;
    int guard = 0;

    while (remaining > CCD_TIME_EPS && guard < CCD_MAX_SUBSTEPS) {
        ++guard;
        ++substeps;

        const auto tCCD0 = clock::now();
        const TOIResult toi = findEarliestTOI(bodies, remaining);
        const auto tCCD1 = clock::now();
        ccdMs += std::chrono::duration<double, std::milli>(tCCD1 - tCCD0).count();

        float advance;
        if (toi.hit) {
            const float seat = (CCD_TOLERANCE + PENETRATION_SLOP) / toi.closingSpeed;
            advance = std::min(remaining, toi.toi + seat);
        } else {
            advance = remaining;
        }
        if (advance < 0.0f) advance = 0.0f;

        integratePositions(bodies, advance);
        remaining -= advance;

        // Only resolve again if there is still time left in the step (i.e. we
        // stopped early at a TOI). The final slice's contacts are resolved at
        // the start of the next step.
        if (remaining > CCD_TIME_EPS) {
            const auto tSolve0 = clock::now();
            detectAndResolve(bodies);
            const auto tSolve1 = clock::now();
            solveMs += std::chrono::duration<double, std::milli>(tSolve1 - tSolve0).count();
        }
    }

    if (sleepingEnabled) updateSleeping(bodies, dt);

    // Diagnostic: print only on expensive frames so normal running stays quiet.
    const double totalMs = ccdMs + solveMs;
    if (totalMs > 3.0) {
        int awake = 0;
        for (const auto& b : bodies) {
            if (b.inverseMass > 0.0f && !b.asleep) ++awake;
        }
        std::cout << "[PhysProfile] total=" << totalMs << "ms"
                  << "  ccd=" << ccdMs << "ms"
                  << "  solve=" << solveMs << "ms"
                  << "  substeps=" << substeps
                  << "  awake=" << awake << "/" << bodies.size()
                  << "  contacts=" << lastContactCount << "\n";
    }
}
