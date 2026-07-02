#pragma once
#include <glm/glm.hpp>

struct CollisionInfo {
    bool collided = false; // true if the two AABBs overlap on all three axes
    glm::vec3 normal = glm::vec3(0.0f); // the axis along which the two bodies should be separated, pointing from body A toward body B, tells which direction to push each body
    float penetration = 0.0f; // how deeply two AABBs overlap on the separation axis, smaller values tell that the two should be pushed apart less
};