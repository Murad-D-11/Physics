#include "aabb.h"
#include "rigidbody.h"

AABB AABB::fromRigidBody(const RigidBody& body) {
    const glm::vec3 halfSize = body.scale * 0.5f;
    return { body.position - halfSize, body.position + halfSize }; // returns the min and max, respectively

    /**
     * Note:
     * This AABB is built from position and scale only, body.orientation is deliberately not applied here.
     * This is a known, intentional simplification: true rotated-box OBB collision belongs to a later pass
     * once linear + angular dynamics are working. In the meantime, this means a tilted cube's actual corners
     * can poke slightly outside its own (still axis-aligned) collision volume, or contacts can register
     * slightly early/late compared to the rendered shape: acceptable for now, not physically exact.
     */
}