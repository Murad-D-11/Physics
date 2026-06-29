#include "physicssolver.h"

PhysicsSolver::PhysicsSolver() {} // nothing to initalize (yet): no GPU resources, no memory, etc.
PhysicsSolver::~PhysicsSolver() {} // same ye olde reasoning for the destructor

/**
 * This function is the entire physics pipeline for one fixed timestep.
 * The order matters: force transforms into acceleration, acceleration into velocity, velocity into change in position.
 * Produces wrong results if in an different order.
 */
void PhysicsSolver::step(RigidBody& body, float deltaTime) {
    // vvvvv Gravity + Euler integration vvvvv

    const glm::vec3 gravity(0.0f, -9.81f, 0.0f); // this is the gravitation vector, it only affects the y-component (9.81 m/s^2 pulling downwards)
    
    body.acceleration = gravity; // sets the acceleration vector as the gravity vector when falling
    body.velocity += body.acceleration * deltaTime; // simple velocity from acceleration conversion
    body.position += body.velocity * deltaTime; // simple position from velocity conversion

    // ^^^^^ Gravity + Euler integration ^^^^^

    // vvvvv Floor collision detection + response vvvvv

    const float bottom = body.position.y - halfHeight; // represents the bottom point of the cube

    // Cube penetrates the floor, its bottom edge is lower than y = 0
    if (bottom < floorY) {
        body.position.y = floorY + halfHeight; // pushes the cube back up, exactly so that it sits on the floor
        body.velocity.y *= -restitution; // reverses vertical velocity, creating a "bounce" effect (Newton's third law!11!!11!)
    }

    // the cube eventually comes down to rest on the floor

    // ^^^^^ Floor collision detection + response ^^^^^
}