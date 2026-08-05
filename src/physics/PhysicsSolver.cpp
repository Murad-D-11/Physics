#include "physicssolver.h"
#include "obb.h"
#include "collision.h"
#include <algorithm>
#include <cmath>

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

    floorBody.restitution = 0.3f; // concrete-like, was incorrectly 1.0
    floorBody.friction = 0.6f;
}

PhysicsSolver::~PhysicsSolver() {}

// ============================================================================
// Integration
// ============================================================================

void PhysicsSolver::integrate(RigidBody& body, float deltaTime) {
    if (body.inverseMass == 0.0f) return;

    // Linear: gravity + symplectic Euler
    const glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    body.acceleration = gravity;
    body.velocity += body.acceleration * deltaTime;
    body.position += body.velocity * deltaTime;
    body.acceleration = glm::vec3(0.0f);

    // Angular: integrate angular velocity -> orientation
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

    // Build the 8 world-space corners of the rotated box
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

    // Any corner within epsilon of the deepest becomes a contact point
    for (int i = 0; i < 8 && contacts.size() < 4; ++i) {
        const float y = worldCorners[i].y;
        if (y <= lowestY + FACE_CONTACT_EPSILON) {
            CollisionInfo info;
            info.collided = true;
            info.point = worldCorners[i];
            info.penetration = floorTopY - y;
            info.normal = glm::vec3(0.0f, -1.0f, 0.0f); // from dynamic body toward floor (A->B convention)
            info.featureId = static_cast<uint32_t>(i);    // corner index for warm starting
            contacts.push_back(info);
        }
    }

    return contacts;
}

// ============================================================================
// Contact Precomputation
// ============================================================================

void PhysicsSolver::precomputeContact(Contact& c) {
    c.rA = c.info.point - c.a->position;
    c.rB = c.info.point - c.b->position;

    const glm::vec3& n = c.info.normal;

    // --- Build orthonormal tangent basis ---
    // Pick a vector not parallel to n, cross it with n to get tangent1
    glm::vec3 ref = (std::abs(n.x) < 0.9f) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    c.tangent1 = glm::normalize(glm::cross(n, ref));
    c.tangent2 = glm::cross(n, c.tangent1);

    // --- Effective mass along normal ---
    // Formula: 1 / (mA_inv + mB_inv + dot(n, cross(IA_inv * cross(rA, n), rA))
    //                                 + dot(n, cross(IB_inv * cross(rB, n), rB)))
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

    // --- Initial relative velocity along normal (for restitution) ---
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

        // Search cache for matching contact (same body pair + same feature ID)
        for (const auto& cached : contactCache) {
            if (cached.a == c.a && cached.b == c.b && cached.featureId == c.info.featureId) {
                c.accumulatedNormalImpulse = cached.accumulatedNormalImpulse * WARM_START_SCALE;
                c.accumulatedTangentImpulse1 = cached.accumulatedTangentImpulse1 * WARM_START_SCALE;
                c.accumulatedTangentImpulse2 = cached.accumulatedTangentImpulse2 * WARM_START_SCALE;
                break;
            }
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
    contactCache.reserve(contacts.size());

    for (const auto& c : contacts) {
        CachedContact cached;
        cached.a = c.a;
        cached.b = c.b;
        cached.featureId = c.info.featureId;
        cached.accumulatedNormalImpulse = c.accumulatedNormalImpulse;
        cached.accumulatedTangentImpulse1 = c.accumulatedTangentImpulse1;
        cached.accumulatedTangentImpulse2 = c.accumulatedTangentImpulse2;
        contactCache.push_back(cached);
    }
}

// ============================================================================
// Velocity Solver (Accumulated Impulse, Catto-style)
// ============================================================================

void PhysicsSolver::solveVelocities(std::vector<Contact>& contacts) {
    for (auto& c : contacts) {
        // Current relative velocity at the contact point
        const glm::vec3 velAtA = c.a->velocity + glm::cross(c.a->angularVelocity, c.rA);
        const glm::vec3 velAtB = c.b->velocity + glm::cross(c.b->angularVelocity, c.rB);
        const glm::vec3 dv = velAtB - velAtA;

        // ================================================================
        // Normal impulse
        // ================================================================
        const float vn = glm::dot(dv, c.info.normal);

        // Restitution bias: only when initial approach speed exceeded rest threshold
        float restitutionBias = 0.0f;
        const float e = std::min(c.a->restitution, c.b->restitution);
        if (c.initialRelVelN < -REST_THRESHOLD) {
            restitutionBias = -e * c.initialRelVelN;
        }

        // Baumgarte position bias: gently resolves penetration through velocity
        // bias = (beta / dt) * max(0, penetration - slop)
        float positionBias = 0.0f;
        const float excess = c.info.penetration - PENETRATION_SLOP;
        if (excess > 0.0f) {
            positionBias = (BAUMGARTE_FACTOR / FIXED_DT) * excess;
        }

        // Desired impulse delta for this iteration
        float lambda = c.effectiveMassNormal * (-(vn - restitutionBias - positionBias));

        // CLAMP accumulated impulse: normal force can only push, never pull
        const float oldAccumN = c.accumulatedNormalImpulse;
        c.accumulatedNormalImpulse = std::max(0.0f, oldAccumN + lambda);
        lambda = c.accumulatedNormalImpulse - oldAccumN; // actual delta

        // Apply normal impulse
        const glm::vec3 normalImpulse = lambda * c.info.normal;
        c.a->velocity -= normalImpulse * c.a->inverseMass;
        c.b->velocity += normalImpulse * c.b->inverseMass;
        c.a->angularVelocity -= c.a->inverseInertiaWorld * glm::cross(c.rA, normalImpulse);
        c.b->angularVelocity += c.b->inverseInertiaWorld * glm::cross(c.rB, normalImpulse);

        // ================================================================
        // Friction impulse — Tangent 1
        // ================================================================
        {
            const glm::vec3 velAtA2 = c.a->velocity + glm::cross(c.a->angularVelocity, c.rA);
            const glm::vec3 velAtB2 = c.b->velocity + glm::cross(c.b->angularVelocity, c.rB);
            const glm::vec3 dv2 = velAtB2 - velAtA2;

            const float vt1 = glm::dot(dv2, c.tangent1);
            float lambdaT1 = c.effectiveMassTangent1 * (-vt1);

            // Coulomb friction cone: |P_t| <= mu * P_n (using accumulated normal impulse)
            const float mu = std::sqrt(c.a->friction * c.b->friction);
            const float maxFriction = mu * c.accumulatedNormalImpulse;

            const float oldAccumT1 = c.accumulatedTangentImpulse1;
            c.accumulatedTangentImpulse1 = std::clamp(oldAccumT1 + lambdaT1, -maxFriction, maxFriction);
            lambdaT1 = c.accumulatedTangentImpulse1 - oldAccumT1;

            const glm::vec3 frictionImpulse1 = lambdaT1 * c.tangent1;
            c.a->velocity -= frictionImpulse1 * c.a->inverseMass;
            c.b->velocity += frictionImpulse1 * c.b->inverseMass;
            c.a->angularVelocity -= c.a->inverseInertiaWorld * glm::cross(c.rA, frictionImpulse1);
            c.b->angularVelocity += c.b->inverseInertiaWorld * glm::cross(c.rB, frictionImpulse1);
        }

        // ================================================================
        // Friction impulse — Tangent 2
        // ================================================================
        {
            const glm::vec3 velAtA3 = c.a->velocity + glm::cross(c.a->angularVelocity, c.rA);
            const glm::vec3 velAtB3 = c.b->velocity + glm::cross(c.b->angularVelocity, c.rB);
            const glm::vec3 dv3 = velAtB3 - velAtA3;

            const float vt2 = glm::dot(dv3, c.tangent2);
            float lambdaT2 = c.effectiveMassTangent2 * (-vt2);

            const float mu = std::sqrt(c.a->friction * c.b->friction);
            const float maxFriction = mu * c.accumulatedNormalImpulse;

            const float oldAccumT2 = c.accumulatedTangentImpulse2;
            c.accumulatedTangentImpulse2 = std::clamp(oldAccumT2 + lambdaT2, -maxFriction, maxFriction);
            lambdaT2 = c.accumulatedTangentImpulse2 - oldAccumT2;

            const glm::vec3 frictionImpulse2 = lambdaT2 * c.tangent2;
            c.a->velocity -= frictionImpulse2 * c.a->inverseMass;
            c.b->velocity += frictionImpulse2 * c.b->inverseMass;
            c.a->angularVelocity -= c.a->inverseInertiaWorld * glm::cross(c.rA, frictionImpulse2);
            c.b->angularVelocity += c.b->inverseInertiaWorld * glm::cross(c.rB, frictionImpulse2);
        }
    }
}

// ============================================================================
// Main Solver Entry Point
// ============================================================================

void PhysicsSolver::detectAndResolve(std::vector<RigidBody>& bodies) {
    // Reset collision flags
    for (auto& body : bodies) {
        body.isColliding = false;
    }

    std::vector<Contact> contacts;

    // ---- Body-body detection: OBB-SAT + Sutherland-Hodgman manifold ----
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) {
            const OBB obbA = OBB::fromRigidBody(bodies[i]);
            const OBB obbB = OBB::fromRigidBody(bodies[j]);

            const SATResult sat = Collision::testOBB(obbA, obbB);
            if (!sat.colliding) continue;

            for (const CollisionInfo& info : Collision::generateManifold(obbA, obbB, sat)) {
                contacts.push_back({&bodies[i], &bodies[j], info,
                                    glm::vec3(0.0f), glm::vec3(0.0f),
                                    0.0f, 0.0f, 0.0f,
                                    glm::vec3(0.0f), glm::vec3(0.0f),
                                    0.0f, 0.0f, 0.0f, 0.0f});
            }
        }
    }

    // ---- Body-floor detection ----
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

    // Mark colliding bodies — only when actively impacting (not resting)
    for (const auto& c : contacts) {
        if (c.b == &floorBody) continue;

        // Check relative approach speed at this contact
        const glm::vec3 rA = c.info.point - c.a->position;
        const glm::vec3 rB = c.info.point - c.b->position;
        const glm::vec3 velAtA = c.a->velocity + glm::cross(c.a->angularVelocity, rA);
        const glm::vec3 velAtB = c.b->velocity + glm::cross(c.b->angularVelocity, rB);
        const float vRelN = glm::dot(velAtB - velAtA, c.info.normal);

        // Only flag as "colliding" (red) if approaching faster than rest threshold
        if (vRelN < -REST_THRESHOLD) {
            c.a->isColliding = true;
            c.b->isColliding = true;
        }
    }

    if (contacts.empty()) {
        contactCache.clear();
        return;
    }

    // ---- Precompute per-contact solver data ----
    for (auto& c : contacts) {
        precomputeContact(c);
    }

    // ---- Warm start from cached impulses ----
    matchAndLoadCache(contacts);
    warmStart(contacts);

    // ---- Iterative velocity solver ----
    for (int iter = 0; iter < SOLVER_ITERATIONS; ++iter) {
        solveVelocities(contacts);
    }

    // ---- Store to cache for next frame ----
    storeCache(contacts);
}
