#pragma once
#include <glm/glm.hpp>

/**
 * All the info produced by one collision test between two AABBs
 */
struct CollisionInfo {
    bool collided = false; // true if the two AABBs overlap on all three axes

    float penetration = 0.0f; // how deeply two AABBs overlap on the separation axis, smaller values tell that the two should be pushed apart less

    glm::vec3 point = glm::vec3(0.0f); // an approximate world-space contact point, used to compute the lever arm for torque and angular impulses (computed as centroid of the AABB overlap region on all three axes, good enough to produce believable off-center torque)
    glm::vec3 normal = glm::vec3(0.0f); // the axis along which the two bodies should be separated, pointing from body A toward body B, tells which direction to push each body
};