#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct RigidBody;

/**
 * Oriented Bounding Box: a box collision volume that rotates with the body.
 * The box is defined by its center, half-extents along the local axes, and 
 * an orientation matrix whose columns are the box's local axes expressed in
 * world space
 */
struct OBB {
    glm::vec3 center; // world-space center
    glm::vec3 halfExtents; // half-size along each local axis
    glm::mat3 axes; // columns are the box's local x/y/z axes in world space

    static OBB fromRigidBody(const RigidBody& body);

    // returns the 8 world-space corners of this OBB
    void getCorners(glm::vec3 out[8]) const;

    /**
     * Returns the 4 world-space vertices of the face whose outward normal
     * is along local axis 'axisIndex' (0=x, 1=y, 2=z) in direction 'sign' (+1 or -1).
     */
    void getFaceVertices(int axisIndex, float sign, glm::vec3 out[4]) const;
};