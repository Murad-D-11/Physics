#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class RigidBody {
    public:
        glm::vec3 position = glm::vec3(0.0f, 25.0f, 0.0f); // starts at y = 5, so the cube has room to fall
        glm::vec3 velocity = glm::vec3(0.0f); // how fast and which direction object is heading
        glm::vec3 acceleration = glm::vec3(0.0f); // rate of change of velocity, set by the PhysicsSolver class
        glm::vec3 scale = glm::vec3(1.0f); // the world-space size of this body on each axis, used to compute AABB

        float mass = 1.0f;
        float inverseMass = 1.0f; // reciprocal of mass, which is 1 / mass
        float restitution = 0.4f; // how much kinetic energy is preserved on impact
        float friction = 0.5f; // a single Coulomb friction coefficient --> when the two bodies touch, the solver averages their two friction values to get the combined friction for that contact

        bool isColliding = false; // this is when the two bodies overlap

        glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // the body's current orientation, as a unit quaterion (quaterion is used instead of Euler angles because it integrates smoothly without gimbal lock)
        glm::vec3 angularVelocity = glm::vec3(0.0f); // how fast body is spinning around each world axis, in radians per second, updated by torque
        glm::vec3 torque = glm::vec3(0.0f); // rotational analogue of acceleration

        float inertia = 1.0f;
        float inverseInertia = 1.0f;
        float rollingResistance = 0.02f; 
        /**
         * AUDIT: DISABLED ^^^
         * kinetic energy loss from a non-spherical object rolling 
        */ 
};