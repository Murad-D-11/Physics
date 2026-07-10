#pragma once
#include <glm/glm.hpp>

class RigidBody {
    public:
        glm::vec3 position = glm::vec3(0.0f, 25.0f, 0.0f); // starts at y = 5, so the cube has room to fall
        glm::vec3 velocity = glm::vec3(0.0f); // how fast and which direction object is heading
        glm::vec3 acceleration = glm::vec3(0.0f); // rate of change of velocity, set by the PhysicsSolver class
        glm::vec3 scale = glm::vec3(1.0f); // the world-space size of this body on each axis, used to compute AABB

        float mass = 1.0f;
        float inverseMass = 1.0f; // reciprocal of mass, which is 1 / mass

        float restitution = 0.4f; // how much kinetic energy is preserved on impact

        bool isColliding = false;
};