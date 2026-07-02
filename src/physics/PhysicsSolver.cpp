#include "physicssolver.h"
#include "aabb.h"
#include "collision.h"
#include "collisioninfo.h"

PhysicsSolver::PhysicsSolver() {} // nothing to initalize (yet): no GPU resources, no memory, etc.
PhysicsSolver::~PhysicsSolver() {} // same ye olde reasoning for the destructor

void PhysicsSolver::integrate(RigidBody& body, float deltaTime) {
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
    if (bottom < floorY) {
        body.position.y = floorY + halfHeight;
        body.velocity.y *= -restitution; // allows the object to "bounce" off the ground until it stops (velocity.y = 0)
    }
}

// Cube-to-cube detection and position correction
void PhysicsSolver::detectAndResolve(std::vector<RigidBody>& bodies) {
    for (auto& body : bodies) {
        body.isColliding = false;
    }

    for (std::size_t i = 0; i < bodies.size(); i++) {
        for (std::size_t j = i + 1; j < bodies.size(); j++) {
            // j starts at i + 1 for two reasons:
            // 1. j == i would be a body testing against itself --> always "colliding"
            // 2. j < i would re-test pairs already checked

            const AABB aabbA = AABB::fromRigidBody(bodies[i]);
            const AABB aabbB = AABB::fromRigidBody(bodies[j]);

            const CollisionInfo info = Collision::test(aabbA, aabbB); // test whether the two overlap, computes separation normal and penetration depth if so

            if (info.collided) {
                bodies[i].isColliding = true;
                bodies[j].isColliding = true;

                Collision::resolvePenetration(bodies[i], bodies[j], info); // push bodies apart so they no longer overlap (velocity not modified for now)
            }
        }
    }
}