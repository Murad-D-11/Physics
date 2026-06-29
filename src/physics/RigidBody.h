#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class RigidBody {
    public:
        glm::vec3 position = glm::vec3(0.0f, 25.0f, 0.0f); // starts at y = 5, so the cube has room to fall
        glm::vec3 velocity = glm::vec3(0.0f); // how fast and which direction object is heading
        glm::vec3 acceleration = glm::vec3(0.0f); // rate of change of velocity, set by the PhysicsSolver class

        float mass = 1.0f;
};