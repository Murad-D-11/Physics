#include "collision.h"
#include "rigidbody.h"
#include <algorithm>

CollisionInfo Collision::test(const AABB& a, const AABB& b) {
    CollisionInfo info;

    // vvvvv Separating Axis Theorem (simplified!!) vvvvv //
    /**
     * For axis-aligned boxes, we only need to check 3 axes
     * If there is a gap between the boxes on any axis, they cannot be colliding
     * Only if they overlap on all three axes simultaneously, then they are colliding
     */

    const float xOverlap = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x); // how much the two penetrate along x
    const float yOverlap = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y); // same with y
    const float zOverlap = std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z); // same with z

    // no collision, do not proceed
    if (xOverlap <= 0.0f || yOverlap <= 0.0f || zOverlap <= 0.0f) {
        return info;
    }

    info.collided = true; // collision happened

    // ^^^^^ Separating Axis Theorem (simplified!!) ^^^^^ //

    // vvvvv Minimum Translation Vector vvvvv //

    /**
     * Three overlap amounts exist. The axis with the smallest overlap is
     * the one that requires the least movement to perform a resolve a collision.
     * This is the axis that we will separate along (which is the collision normal)
     */

    // centers of the two objects
    const glm::vec3 centerA = (a.min + a.max) * 0.5f;
    const glm::vec3 centerB = (b.min + b.max) * 0.5f;

    const glm::vec3 fromAtoB = centerB - centerA; // direction of the collision normal

    // determining the unit vector of the collision normal 
    if (xOverlap <= yOverlap && xOverlap <= zOverlap) {
        info.penetration = xOverlap;
        info.normal = glm::vec3(fromAtoB.x > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
    } else if (yOverlap <= xOverlap && yOverlap <= zOverlap) {
        info.penetration = yOverlap;
        info.normal = glm::vec3(0.0f, fromAtoB.y > 0.0f ? 1.0f : -1.0f, 0.0f);
    } else {
        info.penetration = zOverlap; // z-axis has the smallest overlap
        info.normal = glm::vec3(0.0f, 0.0f, fromAtoB.z > 0.0f ? 1.0f : -1.0f); // unit vector of the normal
    }

    return info;
}

/**
 * Position correction from a collision
 */
void Collision::resolvePenetration(RigidBody& a, RigidBody& b, const CollisionInfo& info) {
    const float totalMass = a.mass + b.mass;
    const float ratioA = b.mass / totalMass; // how much of correction should A receive from collision between A and B
    const float ratioB = a.mass / totalMass; // similar process but with B

    a.position -= info.normal * info.penetration * ratioA; // pushes A in the opposite direction of the normal, away from B
    b.position += info.normal * info.penetration * ratioB; // pushes B in the direction of the normal, away from A
}