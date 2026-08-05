#include "obb.h"
#include "rigidbody.h"

OBB OBB::fromRigidBody(const RigidBody& body) {
    OBB obb;
    obb.center = body.position;
    obb.halfExtents = body.scale * 0.5;
    obb.axes = glm::mat3_cast(body.orientation); // columns = local x, y, z in world space

    return obb;
}

void OBB::getCorners(glm::vec3 out[8]) const {
    // each corner is center + sum of (+/-halfExtent[i] * axis[i])
    const glm::vec3 ax = axes[0] * halfExtents.x;
    const glm::vec3 ay = axes[1] * halfExtents.y;
    const glm::vec3 az = axes[2] * halfExtents.z;

    out[0] = center - ax - ay - az;
    out[1] = center + ax - ay - az;
    out[2] = center + ax - ay + az;
    out[3] = center - ax - ay + az;
    out[4] = center - ax + ay - az;
    out[5] = center + ax + ay - az;
    out[6] = center + ax + ay + az;
    out[7] = center - ax + ay + az;
}

void OBB::getFaceVertices(int axisIndex, float sign, glm::vec3 out[4]) const {
    /**
     * Returns the 4 vertices of one face of the OBB.
     * The face is perpendicular to local axis 'axisIndex', offset in direction 'sign'.
     * 
     * Convention: vertices are ordered counter-clockwise when viewed from outside.
     */
    // the two tangent axes (the axes that span the face)
    const int t1 = (axisIndex + 1) % 3;
    const int t2 = (axisIndex + 2) % 3;

    const glm::vec3 faceCenter = center + axes[axisIndex] * (halfExtents[axisIndex]) * sign;
    const glm::vec3 u = axes[t1] * halfExtents[t1];
    const glm::vec3 v = axes[t2] * halfExtents[t2];

    out[0] = faceCenter - u - v;
    out[1] = faceCenter + u - v;
    out[2] = faceCenter + u + v;
    out[3] = faceCenter - u + v;
}