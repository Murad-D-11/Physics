#include "physicssolver.h"
#include "aabb.h"
#include "collision.h"

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
    const float halfHeight = body.scale.y * 0.5f;
    const float bottom = body.position.y - halfHeight; // the object's lowest point's y-coordinate

    // if object touches the ground
    if (bottom < FLOOR_Y) {
        body.position.y = FLOOR_Y + halfHeight;

        if (body.velocity.y < 0.0f) {
            float e = body.restitution; // use this body's own restitution value rather than its global constant, as each material bounces differently

            // kill the bounce to prevent endless micro-vibrations
            if (std::abs(body.velocity.y) < REST_THRESHOLD) {
                e = 0.0f;
            }

            body.velocity.y *= -e; // allows the object to "bounce" off the ground until it stops (velocity.y = 0)
        }
    }
}

// Cube-to-cube detection and position correction
void PhysicsSolver::detectAndResolve(std::vector<RigidBody>& bodies) {
    // reset all collision flags
    for (auto& body : bodies) {
        body.isColliding = false;
    }

    // detect all colliding pairs
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
    const glm::vec3 relVel = b.velocity - a.velocity; // component of relative velocity along the collision normal (negative is approaching, positive is separating)
    const float vRelN = glm::dot(relVel, info.normal);

    // ignore separating contacts (already moving apart, applying impulse would incorrectly pull them back together)
    if (vRelN > 0.0f) return;

    // impulse magnitude
    float e = std::min(a.restitution, b.restitution); // less bouncy of the two determines the combined bounce

    if (std::abs(vRelN) < REST_THRESHOLD) e = 0.0f; // zero out restitution to stop the micro-vibrations

    // impulse scalar
    const float j = -(1.0f + e) * vRelN / invMassSum;

    // apply impulse
    const glm::vec3 impulse = j * info.normal;
    a.velocity -= impulse * a.inverseMass;
    b.velocity += impulse * b.inverseMass;
}