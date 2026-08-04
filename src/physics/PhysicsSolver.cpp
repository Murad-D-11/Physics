#include "physicssolver.h"
#include "aabb.h"
#include "collision.h"
#include <algorithm>
#include <cmath>

// namespace {
//     /**
//      * Local contact point produced by the box-plane contact generation in floorCollision().
//      * Not exposed in the header since it's only needed internally, unlike CollisionInfo
//      * which is shared with the box-box path.
//      */
//     struct FloorContactPoint {
//         glm::vec3 point;
//         float penetration; // FLOOR - point.y: positive means this corner is below the floor
//     };
// }

// Builds the floor body
PhysicsSolver::PhysicsSolver() {
    floorBody.scale = glm::vec3(FLOOR_HALF_EXTENT * 2.0f, FLOOR_THICKNESS, FLOOR_HALF_EXTENT * 2.0f);
    floorBody.position = glm::vec3(0.0f, FLOOR_Y - FLOOR_THICKNESS * 0.5f, 0.0f); // top face lands exactly on FLOOR_Y, same plane the old solver used
    floorBody.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    floorBody.velocity = glm::vec3(0.0f);
    floorBody.angularVelocity = glm::vec3(0.0f);

    floorBody.inverseMass = 0.0f; // infinite mass: no impulse can move it
    floorBody.updateInertiaTensor(); // can't be spun either

    floorBody.restitution = 1.0f;
    floorBody.friction = 1.0f;
}

PhysicsSolver::~PhysicsSolver() {} // same ye olde reasoning for the destructor

void PhysicsSolver::integrate(RigidBody& body, float deltaTime) {
    if (body.inverseMass == 0.0f) {
        return; // inverseMass of zero means that the object has infinite mass, so gravity and integration have no effect on them
    }
    
    // vvvvv Gravity + Euler integration vvvvv

    const glm::vec3 gravity(0.0f, -9.81f, 0.0f); // this is the gravitation vector, it only affects the y-component (9.81 m/s^2 pulling downwards)
    
    body.acceleration = gravity; // sets the acceleration vector as the gravity vector when falling
    body.velocity += body.acceleration * deltaTime; // simple velocity from acceleration conversion
    body.position += body.velocity * deltaTime; // simple position from velocity conversion

    // ^^^^^ Gravity + Euler integration ^^^^^

    body.acceleration = glm::vec3(0.0f); // resets acceleration to zero after integrating, gravity would stack if otherwise

    // vvvvv Angular integration vvvvv //

    if (body.inverseInertiaLocal != glm::mat3(0.0f)) {
        /**
         * World-space inverse inertia tensor, recomputed every step from the body's
         * current orientation. This has to be redone every step because a box resists
         * spinning differently about each of its own axes, so its resistance to
         * spinning about world axes changes continuously as it tumbles, even though
         * inverseInertiaLocal itself never changes.
         */
        const glm::mat3 R = glm::mat3_cast(body.orientation);
        body.inverseInertiaWorld = R * body.inverseInertiaLocal * glm::transpose(R);

        body.angularVelocity += (body.torque * body.inverseInertiaWorld) * deltaTime; // angular equivalent of velocity += acceleration * dt;
        const glm::quat angularVelocityQuat(0.0f, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z);
        
        body.orientation += (angularVelocityQuat * body.orientation) * (0.5f * deltaTime); // quaternion multiplication composes teh spin with the current orientation; scaling it by 0.5 * dt integrates it forward one step (the 0.5 comes directly from the quaternion derivative formula)
        body.orientation = glm::normalize(body.orientation); // each integration step above is a first-order approximation, so repeated additions slowly drift the quaternion away from unit length; renormalizing keeps it a valid rotation
        
        body.torque = glm::vec3(0.0f); // cleared after integrationm just like linear acceleration (otherwise would stack)
    }

    // ^^^^^ Angular integration ^^^^^ //
}

// Replaced the floorCollision() function
std::vector<CollisionInfo> PhysicsSolver::generateFloorContacts(const RigidBody& body) const {
    std::vector<CollisionInfo> contacts;

    if (body.inverseMass == 0.0f) {
        return contacts; // static body --> infinite mass, nothing can push it. Dividing by inverseMass below would be a division by zero
    }

    // vvvvv Contact Point Generation vvvvv //
    /**
     * A box can touch a plane at 1, 2, or 4 corners depending on orientation
     * 1 corner --> balanced on a tip
     * 2 corners --> balanced on an edge
     * 4 corners --> resting flat on a face
     */

    // Build the eight world-space corners once. Every geometric quantity below
    // (detection, penetration, contact point) is derived from this same rotated
    // geometry, so there's no longer a mix of axis-aligned and rotated math.
    const glm::vec3 halfSize = body.scale * 0.5f;
    const glm::vec3 localCorners[8] {
        {-halfSize.x, -halfSize.y, -halfSize.z}, {halfSize.x, -halfSize.y, -halfSize.z},
        {halfSize.x, -halfSize.y, halfSize.z}, {-halfSize.x, -halfSize.y, halfSize.z},
        {-halfSize.x, halfSize.y, -halfSize.z}, {halfSize.x, halfSize.y, -halfSize.z},
        {halfSize.x, halfSize.y, halfSize.z}, {-halfSize.x, halfSize.y, halfSize.z}
    };

    glm::vec3 worldCorners[8];
    float lowestY = 0.0f;

    for (int i = 0; i < 8; ++i) {
        worldCorners[i] = body.position + body.orientation * localCorners[i];

        if (i == 0 || worldCorners[i].y < lowestY) {
            lowestY = worldCorners[i].y;
        }
    }

    const float floorTopY = floorBody.position.y + floorBody.scale.y * 0.5f; // derived from floorBody instead of the FLOOR_Y constant directly

    if (lowestY >= floorTopY) {
        return contacts; // no penetration, nothing to resolve
    }

    /**
     * Any corner within FACE_CONTACT_EPSILON of the deepest corner is treated as resting
     * on the same face as it, even if the particular corner isn't technically below the floor
     * itself. This is what turns a single deepest-corner contact into a proper 4-point face
     * contact once a cube has settled flat, instead of only ever tracking whichever one corner
     * happens to be lowest at that instant.
     */
    for (int i = 0; i < 8 && contacts.size() < 4; ++i) {
        const float y = worldCorners[i].y;

        if (y <= lowestY + FACE_CONTACT_EPSILON) {
            CollisionInfo info;
            info.collided = true;
            info.point = worldCorners[i];
            info.penetration = floorTopY - y;
            info.normal = glm::vec3(0.0f, -1.0f, 0.0f); // points from the dynamic body down toward the floor
            contacts.push_back(info);
        }
    }

    return contacts;
}

// void PhysicsSolver::applyFloorImpulse(RigidBody& body, const glm::vec3& contactPoint) {
//     /**
//      * Single box-plane contact resolution: the same two-part (normal impulse +
//      * friction impulse) formula as applyImpulse() below, specialized for an immovable
//      * floor. Pulled into its own function so floorCollision can call it once per touching
//      * corner, per solver iteration, instead of duplicating this logic per contact point.
//      * 
//      * Unlike the old single-contact version, vRelN here is measured at THIS corner
//      * (body.velocity + angularVelocity x r), not just the body's linear velocity. That's
//      * necessary once there's more than one contact per point: a spinning body has a different
//      * velocity at each of its corners, so reusing a single linear vRelN for every point would
//      * make the points indistinguishable and defeat the purpose of resolving them separately.
//      */
//     const glm::vec3 normal (0.0f, 1.0f, 0.0f);
//     const glm::vec3 r = contactPoint - body.position;
//     const glm::vec3 velAtContact = body.velocity + glm::cross(body.angularVelocity, r);

//     const float vRelN = glm::dot(velAtContact, normal); // this corner's velocity along the floor's normal --> negative means it's moving down into the floor

//     if (vRelN >= 0.0f) {
//         return; // this point isn't moving into the floor --> no normal impulse, and no normal force for friction to act against
//     }

//     float e = body.restitution;

//     if (std::abs(vRelN) < REST_THRESHOLD) {
//         e = 0.0f; // resting contact --> suppress the bounce so the body doesn't micro-vibrate forever
//     }

//     const glm::vec3 rxN = glm::cross(r, normal);
//     const float angularTerm = glm::dot(rxN, body.inverseInertiaWorld * rxN); // this corner's own leverage against a normal push, same role as angularTermA/B in applyImpulse()
//     const float j = -(1.0f + e) * vRelN / (body.inverseMass + angularTerm);

//     body.velocity += (j * normal) * body.inverseMass; // normal impulse changes linear velocity only, matching applyImpulse()'s convention; only the friction impulse below produces torque

//     // vvvvv Friction Impulse vvvvv //

//     const glm::vec3 velAtContact2 = body.velocity + glm::cross(body.angularVelocity, r);
//     const glm::vec3 tangentialVel = velAtContact2 - glm::dot(velAtContact2, normal) * normal; // removes the vertical component, leaving whatever horizontal sliding motion remains at this corner
//     const float tangentialSpeed = glm::length(tangentialVel);

//     if (tangentialSpeed > 0.0001f) {
//         const glm::vec3 tangent = tangentialVel / tangentialSpeed; // unit vector
//         const float vRelT = glm::dot(velAtContact2, tangent);

//         const glm::vec3 rxT = glm::cross(r, tangent);
//         const float angularTermT = glm::dot(rxT, body.inverseInertiaWorld * rxT);
//         const float jt = -vRelT / (body.inverseMass + angularTerm); // tangential impulse magnitude needed to fully stop sliding at this corner

//         const float maxFriction = body.friction * j; // Coulomb's law: capped by the normal impulse at this same corner
//         const float frictionMag = std::min(std::abs(jt), maxFriction);
//         // ^^^ Static case: (|jt| <= maxFriction): enough friction to stop sliding completely this step
//         // Kinetic case: (|jt| > maxFriction): friction only removes part of the sliding speed

//         const glm::vec3 frictionImpulse = -frictionMag * tangent;
        
//         body.velocity += frictionImpulse * body.inverseMass;
//         body.angularVelocity += body.inverseInertiaWorld * glm::cross(r, frictionImpulse); // this is what lets a resting/sliding cube start to tip or roll, rather than only ever sliding
//     }

//     // ^^^^^ Friction Impulse ^^^^^ //
// }

// Cube-to-cube detection and position correction
void PhysicsSolver::detectAndResolve(std::vector<RigidBody>& bodies) {
    // reset all collision flags
    for (auto& body : bodies) {
        body.isColliding = false;
    }

    std::vector<Contact> contacts;

    // body-body detection: untouched -- still Collision::test() on AABBs, single
    // contact point per overlapping pair, exactly as before.
    for (std::size_t i = 0; i < bodies.size(); i++) {
        for (std::size_t j = i + 1; j < bodies.size(); j++) {
            const AABB aabbA = AABB::fromRigidBody(bodies[i]);
            const AABB aabbB = AABB::fromRigidBody(bodies[j]);

            const CollisionInfo info = Collision::test(aabbA, aabbB);

            if (info.collided) {
                contacts.push_back({&bodies[i], &bodies[j], info});
            }
        }
    }

    // body-floor detection: NOT part of the loop above, since floorBody isn't in
    // bodies at all. Each dynamic body gets its own multi-corner manifold against
    // floorBody, appended into the exact same `contacts` list the body-body pairs use.
    for (auto& body : bodies) {
        for (const CollisionInfo& floorContact : generateFloorContacts(body)) {
            contacts.push_back({&body, &floorBody, floorContact});
        }
    }

    lastContactCount = static_cast<int>(contacts.size()); // now also counts floor contacts

    for (const auto& c : contacts) {
        if (c.b == &floorBody) continue; // floor is excluded from turning red
        c.a->isColliding = true;
        c.b->isColliding = true;
    }

    /**
     * Position correction: one call per contact, same as the pre-existing body-body
     * path. For a flat 4-corner floor contact this now runs up to 4 times per body
     * per step instead of the single deepest-corner correction floorCollision() used
     * to do on purpose (see that function's old comment). Sharing one correction loop
     * for every contact, floor included, is the whole point of this refactor, so this
     * is an accepted, minor over-correction tradeoff rather than something worth a
     * floor-specific exception.
     */
    for (const auto& c : contacts) {
        const float excess = c.info.penetration - PENETRATION_SLOP;

        if (excess > 0.0f) {
            CollisionInfo corrected = c.info;
            corrected.penetration = excess * PENETRATION_CORRECTION;
            Collision::resolvePenetration(*c.a, *c.b, corrected);
        }
    }

    /**
     * Impulse resolution: iterated across all contacts, floor and body-body alike,
     * through the single applyImpulse() path. No floor-specific branch needed --
     * floorBody's zeroed inverseMass and inverseInertiaWorld already make it behave
     * as immovable inside the existing math.
     */
    for (int iter = 0; iter < SOLVER_ITERATIONS; ++iter) {
        for (const auto& c : contacts) {
            applyImpulse(*c.a, *c.b, c.info);
        }
    }

    /**
     * AUDIT: DISABLED (angular rest threshold) 
     * See settleFlatIfResting() comment for more info
     */
    // for (auto& body : bodies) {
    //     settleFlatIfResting(body);
    // }
}

void PhysicsSolver::applyImpulse(RigidBody& a, RigidBody& b, const CollisionInfo& info) {
    const float invMassSum = a.inverseMass + b.inverseMass;
    if (invMassSum == 0.0f) return; // both bodies are static, no impulse can move them

    const glm::vec3 rA = info.point - a.position;
    const glm::vec3 rB = info.point - b.position;

    // relative velocity
    const glm::vec3 velAtA = a.velocity + glm::cross(a.angularVelocity, rA);
    const glm::vec3 velAtB = b.velocity + glm::cross(b.angularVelocity, rB);
    const glm::vec3 relVel = velAtB - velAtA;

    const float vRelN = glm::dot(relVel, info.normal); // component of relative velocity along the collision normal (negative is approaching, positive is separating)

    // ignore separating contacts (already moving apart, applying impulse would incorrectly pull them back together)
    if (vRelN > 0.0f) return;

    // impulse magnitude
    float e = std::min(a.restitution, b.restitution); // less bouncy of the two determines the combined bounce

    if (std::abs(vRelN) < REST_THRESHOLD) e = 0.0f; // resting contact --> zero out restitution to stop the micro-bounces that cause jitter in stable stacks

    const glm::vec3 rAxN = glm::cross(rA, info.normal);
    const glm::vec3 rBxN = glm::cross(rB, info.normal);
    const float angularTermA = glm::dot(rAxN, a.inverseInertiaWorld * rAxN); // was a.inverseInertia * dot(rAxN, rAxN); full tensor makes resistance direction-dependent
    const float angularTermB = glm::dot(rBxN, b.inverseInertiaWorld * rBxN); // same for b

    // impulse scalar: derived from the restitution condition dot(vB' - vA', n) = -e * vRelN
    const float j = -(1.0f + e) * vRelN / (invMassSum + angularTermA + angularTermB);

    // apply impulse
    const glm::vec3 impulse = j * info.normal;

    a.velocity -= impulse * a.inverseMass; // pushed opposite to the normal (away from B)
    b.velocity += impulse * b.inverseMass; // pushed along the normal (away from A)

    // in charge of slowing down the rolling moment vvvv
    a.angularVelocity -= a.inverseInertiaWorld * glm::cross(rA, impulse);
    b.angularVelocity += b.inverseInertiaWorld * glm::cross(rB, impulse);

    // friction impulse, now also at the contact point + angular
    const glm::vec3 velAtA2 = a.velocity + glm::cross(a.angularVelocity, rA);
    const glm::vec3 velAtB2 = b.velocity + glm::cross(b.angularVelocity, rB);

    const glm::vec3 relVelAfterNormal = velAtB2 - velAtA2;
    const glm::vec3 tangentialVel = relVelAfterNormal - glm::dot(relVelAfterNormal, info.normal) * info.normal; // removes the normal component from the relative velocity, leaving only the sliding component
    
    const float tangentialSpeed = glm::length(tangentialVel);

    if (tangentialSpeed > 0.0001f) {
        const glm::vec3 tangent = tangentialVel / tangentialSpeed; // unit vector along the direction B is sliding relative to A
        const float vRelT = glm::dot(relVelAfterNormal, tangent); // by construction this equals tangentialSpeed (positive value), since tangent was built to point exactly along that motion
        
        const glm::vec3 rAxT = glm::cross(rA, tangent);
        const glm::vec3 rBxT = glm::cross(rB, tangent);
        const float angularTermA_t = glm::dot(rAxT, a.inverseInertiaWorld * rAxT); // was a.inverseInertia * dot(rAxT, rAxT)
        const float angularTermB_t = glm::dot(rBxT, b.inverseInertiaWorld * rBxT); // was b.inverseInertia * dot(rBxT, rBxT); same reasoning as the normal impulse's angular terms, but along the tangent direction
        
        const float jt = -vRelT / (invMassSum + angularTermA_t + angularTermB_t); // tangential impulse mag needed to fully cancel the sliding, using the same invMassSum formula as the normal impulse
        const float mu = std::sqrt(a.friction * b.friction); // combined friction coefficient: geometric mean of both friction values rather than a plain average (this means if either surface is very low-friction, the combined contact stays low friction too)
        const float maxFriction = mu * j; // Coulomb's law: friction impulse is capped by coefficient * normal impulse. j is always >= here since vrelN <= 0
        const float frictionMag = std::min(std::abs(jt), maxFriction);
        // ^^^^ Static case: enough friction to stop sliding outright this step
        // Kinetic case: friction only removes part of the sliding speed

        const glm::vec3 frictionImpulse = -frictionMag * tangent; // always opposes the sliding direction

        a.velocity -= frictionImpulse * a.inverseMass;
        b.velocity += frictionImpulse * b.inverseMass;
        // same push/pull pattern as the normal impulse, just along the tangent axis instead

        a.angularVelocity -= a.inverseInertiaWorld * glm::cross(rA, frictionImpulse); // was a.inverseInertia * cross(blah-blah-blah)
        b.angularVelocity += b.inverseInertiaWorld * glm::cross(rB, frictionImpulse); // same with b
        // friction applied off-center; spins the body just like the normal impulse does
    }

    /**
     * AUDIT: DISABLED
     * Same issue as the floorCollision copy of this mechanism:
     * a hand-authored decelerating torque not derived from
     * actual geometry, applied per-body regardless of whether
     * the real underlying contact model justifies it. Disabled
     * as two cubes that reach pure rolling against each other
     * keep rolling, rather than being artificially slowed by a
     * force with no geometric basis
     */
    // auto applyRollingResistance = [j](RigidBody& body) {
    //     if (body.inverseInertia == 0.0f) return; // static body

    //     const float spinSpeed = glm::length(body.angularVelocity);
    //     if (spinSpeed <= 0.0001f) return;

    //     const glm::vec3 spinDir = body.angularVelocity / spinSpeed;
    //     const float requiredImpulse = spinSpeed / body.inverseInertia;
    //     const float maxRollingImpulse = body.rollingResistance * j;
    //     const float rollingImpulseMag = std::min(requiredImpulse, maxRollingImpulse);

    //     body.angularVelocity -= body.inverseInertia * rollingImpulseMag * spinDir;
    // };

    // applyRollingResistance(a);
    // applyRollingResistance(b);
}