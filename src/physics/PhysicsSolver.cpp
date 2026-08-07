#include "physicssolver.h"
#include "obb.h"
#include "collision.h"
#include <algorithm>
#include <cmath>
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
}

PhysicsSolver::~PhysicsSolver() {}

// ============================================================================
// Integration
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
// Floor Contact Generation (rotation-aware, multi-point)
// ============================================================================

std::vector<CollisionInfo> PhysicsSolver::generateFloorContacts(const RigidBody& body) const {
    std::vector<CollisionInfo> contacts;

    if (body.inverseMass == 0.0f) return contacts;

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

    const float floorTopY = floorBody.position.y + floorBody.scale.y * 0.5f;

    if (lowestY >= floorTopY) return contacts;

    for (int i = 0; i < 8 && contacts.size() < 4; ++i) {
        const float y = worldCorners[i].y;
        if (y <= lowestY + FACE_CONTACT_EPSILON) {
            CollisionInfo info;
            info.collided = true;
            info.point = worldCorners[i];
            info.penetration = floorTopY - y;
            info.normal = glm::vec3(0.0f, -1.0f, 0.0f); // A(body) -> B(floor)
            info.featureId = static_cast<uint32_t>(i);
            contacts.push_back(info);
        }
    }

    return contacts;
}

// ============================================================================
// Spatial-Hash Broadphase
// ============================================================================

namespace {
    // Packs a signed 3D cell coordinate into a 64-bit key (21 bits per axis).
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

    // Bucket every body into all grid cells its AABB overlaps
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

    // Emit unique candidate pairs from bodies that share a cell
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
// Contact Cache: Matching & Warm Starting (O(1) hash lookup)
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
// Velocity Solver (Accumulated Impulse, Catto-style)
// ============================================================================

void PhysicsSolver::solveVelocities(std::vector<Contact>& contacts) {
    for (auto& c : contacts) {
        const glm::vec3 velAtA = c.a->velocity + glm::cross(c.a->angularVelocity, c.rA);
        const glm::vec3 velAtB = c.b->velocity + glm::cross(c.b->angularVelocity, c.rB);
        const glm::vec3 dv = velAtB - velAtA;

        // Normal impulse
        const float vn = glm::dot(dv, c.info.normal);

        float restitutionBias = 0.0f;
        const float e = std::min(c.a->restitution, c.b->restitution);
        if (c.initialRelVelN < -REST_THRESHOLD) {
            restitutionBias = -e * c.initialRelVelN;
        }

        float positionBias = 0.0f;
        const float excess = c.info.penetration - PENETRATION_SLOP;
        if (excess > 0.0f) {
            positionBias = (BAUMGARTE_FACTOR / FIXED_DT) * excess;
        }

        float lambda = c.effectiveMassNormal * (-(vn - restitutionBias - positionBias));

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
// Main Solver Entry Point
// ============================================================================

void PhysicsSolver::detectAndResolve(std::vector<RigidBody>& bodies) {
    for (auto& body : bodies) {
        body.isColliding = false;
    }

    const std::size_t n = bodies.size();
    std::vector<Contact> contacts;

    // Precompute OBBs and their world-space AABBs once per body
    std::vector<OBB> obbs(n);
    std::vector<glm::vec3> aabbMin(n), aabbMax(n);
    for (std::size_t i = 0; i < n; ++i) {
        obbs[i] = OBB::fromRigidBody(bodies[i]);
        const glm::vec3 ext = glm::abs(obbs[i].axes[0]) * obbs[i].halfExtents.x
                            + glm::abs(obbs[i].axes[1]) * obbs[i].halfExtents.y
                            + glm::abs(obbs[i].axes[2]) * obbs[i].halfExtents.z;
        aabbMin[i] = obbs[i].center - ext;
        aabbMax[i] = obbs[i].center + ext;
    }

    // Broadphase: spatial hash -> candidate pairs
    std::vector<std::pair<int, int>> candidatePairs;
    buildBroadphasePairs(bodies, aabbMin, aabbMax, candidatePairs);

    // Narrowphase on candidate pairs (AABB refine, then OBB-SAT + manifold)
    for (const auto& pr : candidatePairs) {
        const int i = pr.first;
        const int j = pr.second;

        if (aabbMin[i].x > aabbMax[j].x || aabbMax[i].x < aabbMin[j].x) continue;
        if (aabbMin[i].y > aabbMax[j].y || aabbMax[i].y < aabbMin[j].y) continue;
        if (aabbMin[i].z > aabbMax[j].z || aabbMax[i].z < aabbMin[j].z) continue;

        const SATResult sat = Collision::testOBB(obbs[i], obbs[j]);
        if (!sat.colliding) continue;

        for (const CollisionInfo& info : Collision::generateManifold(obbs[i], obbs[j], sat)) {
            contacts.push_back({&bodies[i], &bodies[j], info,
                                glm::vec3(0.0f), glm::vec3(0.0f),
                                0.0f, 0.0f, 0.0f,
                                glm::vec3(0.0f), glm::vec3(0.0f),
                                0.0f, 0.0f, 0.0f, 0.0f});
        }
    }

    // Body-floor contacts
    for (auto& body : bodies) {
        for (const CollisionInfo& floorContact : generateFloorContacts(body)) {
            contacts.push_back({&body, &floorBody, floorContact,
                                glm::vec3(0.0f), glm::vec3(0.0f),
                                0.0f, 0.0f, 0.0f,
                                glm::vec3(0.0f), glm::vec3(0.0f),
                                0.0f, 0.0f, 0.0f, 0.0f});
        }
    }

    lastContactCount = static_cast<int>(contacts.size());

    // Flag bodies red only when actively impacting (not resting)
    for (const auto& c : contacts) {
        if (c.b == &floorBody) continue;

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

    matchAndLoadCache(contacts);
    warmStart(contacts);

    for (int iter = 0; iter < SOLVER_ITERATIONS; ++iter) {
        solveVelocities(contacts);
    }

    storeCache(contacts);
}
