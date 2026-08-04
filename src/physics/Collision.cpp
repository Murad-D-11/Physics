#include "collision.h"
#include "rigidbody.h"
#include <algorithm>

std::vector<CollisionInfo> Collision::test(const AABB& a, const AABB& b) {
    std::vector<CollisionInfo> manifold;

    // vvvvv Separating Axis Theorem (simplified!!) vvvvv //
    const float xOverlap = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
    const float yOverlap = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
    const float zOverlap = std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z);

    if (xOverlap <= 0.0f || yOverlap <= 0.0f || zOverlap <= 0.0f) {
        return manifold; // no collision
    }

    const glm::vec3 centerA = (a.min + a.max) * 0.5f;
    const glm::vec3 centerB = (b.min + b.max) * 0.5f;
    const glm::vec3 fromAtoB = centerB - centerA;

    glm::vec3 normal;
    float penetration;
    int axis; // 0 = x, 1 = y, 2 = z -- which axis the contact "face" is perpendicular to

    if (xOverlap <= yOverlap && xOverlap <= zOverlap) {
        axis = 0;
        penetration = xOverlap;
        normal = glm::vec3(fromAtoB.x > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
    } else if (yOverlap <= xOverlap && yOverlap <= zOverlap) {
        axis = 1;
        penetration = yOverlap;
        normal = glm::vec3(0.0f, fromAtoB.y > 0.0f ? 1.0f : -1.0f, 0.0f);
    } else {
        axis = 2;
        penetration = zOverlap;
        normal = glm::vec3(0.0f, 0.0f, fromAtoB.z > 0.0f ? 1.0f : -1.0f);
    }
    // ^^^^^ Separating Axis Theorem (simplified!!) ^^^^^ //

    /**
     * The contact "manifold" is the overlap rectangle on the two axes
     * perpendicular to the normal -- same rectangle the old single-point
     * version collapsed down to its centroid. Every AABB is axis-aligned,
     * so unlike generateFloorContacts (rotated box vs. plane, where each
     * corner has its own depth), all 4 corners here share one penetration
     * and one normal -- only their position differs, which is exactly
     * what gives multi-point contact its resistance to torque.
     */
    const glm::vec3 overlapMin(std::max(a.min.x, b.min.x), std::max(a.min.y, b.min.y), std::max(a.min.z, b.min.z));
    const glm::vec3 overlapMax(std::min(a.max.x, b.max.x), std::min(a.max.y, b.max.y), std::min(a.max.z, b.max.z));
    const glm::vec3 mid = (overlapMin + overlapMax) * 0.5f;

    glm::vec3 corners[4];
    if (axis == 0) {
        corners[0] = {mid.x, overlapMin.y, overlapMin.z};
        corners[1] = {mid.x, overlapMax.y, overlapMin.z};
        corners[2] = {mid.x, overlapMax.y, overlapMax.z};
        corners[3] = {mid.x, overlapMin.y, overlapMax.z};
    } else if (axis == 1) {
        corners[0] = {overlapMin.x, mid.y, overlapMin.z};
        corners[1] = {overlapMax.x, mid.y, overlapMin.z};
        corners[2] = {overlapMax.x, mid.y, overlapMax.z};
        corners[3] = {overlapMin.x, mid.y, overlapMax.z};
    } else {
        corners[0] = {overlapMin.x, overlapMin.y, mid.z};
        corners[1] = {overlapMax.x, overlapMin.y, mid.z};
        corners[2] = {overlapMax.x, overlapMax.y, mid.z};
        corners[3] = {overlapMin.x, overlapMax.y, mid.z};
    }

    manifold.reserve(4);
    for (const auto& corner : corners) {
        CollisionInfo info;
        info.collided = true;
        info.point = corner;
        info.normal = normal;
        info.penetration = penetration;
        manifold.push_back(info);
    }

    return manifold;
}

/**
 * Position correction from a collision
 */
void Collision::resolvePenetration(RigidBody& a, RigidBody& b, const CollisionInfo& info) {
    const float invMassSum = a.inverseMass + b.inverseMass;
    if (invMassSum == 0.0f) return; // both static, nothing to correct

    const float ratioA = a.inverseMass / invMassSum; // heavier / more-immovable body corrects less
    const float ratioB = b.inverseMass / invMassSum;

    a.position -= info.normal * info.penetration * ratioA;
    b.position += info.normal * info.penetration * ratioB;
}