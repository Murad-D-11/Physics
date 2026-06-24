#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class RigidBody {
    public:
        glm::vec3 position = glm::vec3 {0, 0, 0};
        glm::vec3 velocity = glm::vec3 {0, 0, 0};
        glm::vec3 acceleration = glm::vec3 {0, 0, 0};

        float mass = 1.0f;
};