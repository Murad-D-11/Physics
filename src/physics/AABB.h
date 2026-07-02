#pragma once
#include <glm/glm.hpp>

struct RigidBody;

/**
 * Axis-Aligned Bounding Box: the simplest possible collision volume
 * Axis-aligned means its faces are always parallel to the world x/y/z planes; it can't rotate
 * It is defined by just two corners. The advantage is that it is fast to compute and test for intersections.
 */
struct AABB {
    glm::vec3 min;
    glm::vec3 max;

    static AABB fromRigidBody(const RigidBody& body); // computes an AABB that exactly wraps a RigidBody at its current position and scale
};