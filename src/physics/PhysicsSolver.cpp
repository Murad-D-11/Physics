#include "physicssolver.h"
#include "aabb.h"
#include "collision.h"
#include <algorithm>
#include <cmath>

PhysicsSolver::PhysicsSolver() {} // nothing to initalize (yet): no GPU resources, no memory, etc.
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
}

void PhysicsSolver::floorCollision(RigidBody& body) {
    if (body.inverseMass == 0.0f) {
        return; // static body --> infinite mass, nothing can push it. Dividing by inverseMass below would be a division by zero
    }

    const float halfHeight = body.scale.y * 0.5f;
    const float bottom = body.position.y - halfHeight; // the object's lowest point's y-coordinate

    if (bottom >= FLOOR_Y) {
        return; // no penetration, nothing to resolve
    }

    // build collision info for this contact
    CollisionInfo info;
    info.collided = true;
    info.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    info.penetration = FLOOR_Y - bottom; // how far below the floor plane the body's bottom edge currently sits

    // position correction
    const float excess = info.penetration - PENETRATION_SLOP;
    
    if (excess > 0.0f) {
        body.position.y += excess * PENETRATION_CORRECTION;
    }

    // normal impulse
    /**
     * General Two-body impulse formula, specialized for an immovable floor:
     * invMassSum = body.inverseMass + floorInverseMass(=0) = body.inverseMass
     * j = -(1+e) * vRelN / invMassSum
     * body.velocity += j * normal * body.inverseMass
     */
    const float vRelN = glm::dot(body.velocity, info.normal); // body's velocity component along the floor's normal (+y) --> negative means the body is moving down into the floor

    if (vRelN >= 0.0f) {
        return; // body isn't moving into the floor --> no normal impulse, and therefore no normal force for friction to act against either
    }

    float e = body.restitution;

    if (std::abs(vRelN) < REST_THRESHOLD) {
        e = 0.0f; // resting contact --> suppress the bounce so the body doesn't micro-vibrate forever
    }

    const float j = -(1.0f + e) * vRelN / body.inverseMass; // scalar impulse magnitude; dividing by inverseMass here is what makes this the floor-specialized case
    body.velocity += (j * info.normal) * body.inverseMass; // applying and unapplying body.inverseMass looks redundant, but keeping it explicit means j is available below in proper impulse units for the friction Coulomb-law cap

    const glm::vec3 tangentialVel = body.velocity - glm::dot(body.velocity, info.normal) * info.normal; // removes the vertical component from velocity (normal), leaving whatever horizontal sliding motion remains
    const float tangentialSpeed = glm::length(tangentialVel);

    if (tangentialSpeed > 0.0001f) {
        const glm::vec3 tangent = tangentialVel / tangentialSpeed; // unit vector pointing in direction the body is sliding
        const float vRelT = tangentialSpeed; // relative to a stationary floor, the tangential speed IS the relative tangential velocity along the tangent by construction
        const float jt = -vRelT / body.inverseMass; // tangential impulse magnitude needed to fully stop the sliding (mirrors the normal impulse derivation above)
        const float maxFriction = body.friction * j; // Coulomb's law: the maximum friction impulse is proportional to the normal impulse --> more downward force = more normal force = more friction generated
        const float frictionMag = std::min(std::abs(jt), maxFriction);
        // ^^^^ Static case (|jt| <= maxFriction): enough friction to stop sliding completely this step
        // Kinetic case (|jt| > maxFriction): friction can only remove part of the sliding speed, so the body keeps sliding but slower

        const glm::vec3 frictionImpulse = -frictionMag * tangent; // opposes the sliding direction

        body.velocity += frictionImpulse * body.inverseMass;
    }
}

// Cube-to-cube detection and position correction
void PhysicsSolver::detectAndResolve(std::vector<RigidBody>& bodies) {
    // reset all collision flags
    for (auto& body : bodies) {
        body.isColliding = false;
    }

    // detect all colliding pairs, appending them to a list
    std::vector<Contact> contacts;
    for (std::size_t i = 0; i < bodies.size(); i++) {
        for (std::size_t j = i + 1; j < bodies.size(); j++) {
            // j starts at i + 1 for two reasons:
            // 1. j == i would be a body testing against itself --> always "colliding"
            // 2. j < i would re-test pairs already checked

            const AABB aabbA = AABB::fromRigidBody(bodies[i]);
            const AABB aabbB = AABB::fromRigidBody(bodies[j]);

            const CollisionInfo info = Collision::test(aabbA, aabbB); // test whether the two overlap, computes separation normal and penetration depth if so

            if (info.collided) {
                contacts.push_back({i, j, info});
            }
        }
    }

    lastContactCount = static_cast<int>(contacts.size()); // stored for debug stats output in main.cpp

    // set visual debug flags (bodies in a colliding pair turn red)
    for (const auto& c : contacts) {
        bodies[c.indexA].isColliding = true;
        bodies[c.indexB].isColliding = true;
    }

    // penetration correction (once per contact)
    for (const auto& c : contacts) {
        const float excess = c.info.penetration - PENETRATION_SLOP; // ignore penetration smaller than the slop threshold entirely

        if (excess > 0.0f) {
            CollisionInfo corrected = c.info;
            corrected.penetration = excess * PENETRATION_CORRECTION; // only correct 80% of the excess this step, leaving the rest for next steps to handle more gently
            Collision::resolvePenetration(bodies[c.indexA], bodies[c.indexB], corrected);
        }
    }

    // impulse resolution (iterated for stack stability)
    for (int iter = 0; iter < SOLVER_ITERATIONS; ++iter) {
        for (const auto& c : contacts) {
            applyImpulse(bodies[c.indexA], bodies[c.indexB], c.info);
        }
    }
}

void PhysicsSolver::applyImpulse(RigidBody& a, RigidBody& b, const CollisionInfo& info) {
    const float invMassSum = a.inverseMass + b.inverseMass;
    if (invMassSum == 0.0f) return; // both bodies are static, no impulse can move them

    // relative velocity
    const glm::vec3 relVel = b.velocity - a.velocity; 
    const float vRelN = glm::dot(relVel, info.normal); // component of relative velocity along the collision normal (negative is approaching, positive is separating)

    // ignore separating contacts (already moving apart, applying impulse would incorrectly pull them back together)
    if (vRelN > 0.0f) return;

    // impulse magnitude
    float e = std::min(a.restitution, b.restitution); // less bouncy of the two determines the combined bounce

    if (std::abs(vRelN) < REST_THRESHOLD) e = 0.0f; // resting contact --> zero out restitution to stop the micro-bounces that cause jitter in stable stacks
    if (std::abs(vRelN) < REST_THRESHOLD) e = 0.0f; // zero out restitution to stop the micro-vibrations

    // impulse scalar: derived from the restitution condition dot(vB' - vA', n) = -e * vRelN
    const float j = -(1.0f + e) * vRelN / invMassSum;

    // apply impulse
    const glm::vec3 impulse = j * info.normal;

    a.velocity -= impulse * a.inverseMass; // pushed opposite to the normal (away from B)
    b.velocity += impulse * b.inverseMass; // pushed along the normal (away from A)

    // friction impulse along the tangent
    const glm::vec3 relVelAfterNormal = b.velocity - a.velocity;
    const glm::vec3 tangentialVel = relVelAfterNormal - glm::dot(relVelAfterNormal, info.normal) * info.normal; // removes the normal component from the relative velocity, leaving only the sliding component
    const float tangentialSpeed = glm::length(tangentialVel);

    if (tangentialSpeed > 0.0001f) {
        const glm::vec3  tangent = tangentialVel / tangentialSpeed; // unit vector along the direction B is sliding relative to A
        
        const float vRelT = glm::dot(relVelAfterNormal, tangent); // by construction this equals tangentialSpeed (positive value), since tangent was built to point exactly along that motion
        const float jt = -vRelT / invMassSum; // tangential impulse mag needed to fully cancel the sliding, using the same invMassSum formula as the normal impulse
        const float mu = std::sqrt(a.friction * b.friction); // combined friction coefficient: geometric mean of both friction values rather than a plain average (this means if either surface is very low-friction, the combined contact stays low friction too)
        const float maxFriction = mu * j; // Coulomb's law: friction impulse is capped by coefficient * normal impulse. j is always >= here since vrelN <= 0
        const float frictionMag = std::min(std::abs(jt), maxFriction);
        // ^^^^ Static case: enough friction to stop sliding outright this step
        // Kinetic case: friction only removes part of the sliding speed

        const glm::vec3 frictionImpulse = -frictionMag * tangent; // always opposes the sliding direction

        a.velocity -= frictionImpulse * a.inverseMass;
        b.velocity += frictionImpulse * b.inverseMass;
        // same push/pull pattern as the normal impulse, just along the tangent axis instead
    }
}