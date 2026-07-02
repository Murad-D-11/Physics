#include "aabb.h"
#include "rigidbody.h"

AABB AABB::fromRigidBody(const RigidBody& body) {
    const glm::vec3 halfSize = body.scale * 0.5f;
    return { body.position - halfSize, body.position + halfSize }; // returns the min and max, respectively
}