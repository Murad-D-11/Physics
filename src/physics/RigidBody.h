#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class RigidBody {
    public:
        glm::vec3 position = glm::vec3(0.0f, 25.0f, 0.0f);
        glm::vec3 velocity = glm::vec3(0.0f);
        glm::vec3 acceleration = glm::vec3(0.0f);
        glm::vec3 scale = glm::vec3(1.0f);

        float mass = 1.0f;
        float inverseMass = 1.0f;
        float restitution = 0.4f;
        float friction = 0.5f;

        bool isColliding = false;

        glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        glm::vec3 angularVelocity = glm::vec3(0.0f);
        glm::vec3 torque = glm::vec3(0.0f);

        // Split-impulse position correction. These are NOT real velocities --
        // they only push bodies out of penetration during the position solve,
        // then are discarded each step, so overlap correction injects no energy.
        glm::vec3 pseudoLinearVel = glm::vec3(0.0f);
        glm::vec3 pseudoAngularVel = glm::vec3(0.0f);

        // vvvvv Inertia Tensor vvvvv //
        glm::mat3 inertiaLocal = glm::mat3(1.0f);
        glm::mat3 inverseInertiaLocal = glm::mat3(1.0f);
        glm::mat3 inverseInertiaWorld = glm::mat3(1.0f);

        void updateInertiaTensor() {
            if (inverseMass == 0.0f) {
                inertiaLocal = glm::mat3(0.0f);
                inverseInertiaLocal = glm::mat3(0.0f);
                inverseInertiaWorld = glm::mat3(0.0f);
                return;
            }

            const float Ixx = (mass * (scale.y * scale.y + scale.z * scale.z)) / 12.0f;
            const float Iyy = (mass * (scale.x * scale.x + scale.z * scale.z)) / 12.0f;
            const float Izz = (mass * (scale.x * scale.x + scale.y * scale.y)) / 12.0f;

            inertiaLocal = glm::mat3(0.0f);
            inertiaLocal[0][0] = Ixx;
            inertiaLocal[1][1] = Iyy;
            inertiaLocal[2][2] = Izz;

            inverseInertiaLocal = glm::mat3(0.0f);
            inverseInertiaLocal[0][0] = (Ixx > 0.0f) ? 1.0f / Ixx : 0.0f;
            inverseInertiaLocal[1][1] = (Iyy > 0.0f) ? 1.0f / Iyy : 0.0f;
            inverseInertiaLocal[2][2] = (Izz > 0.0f) ? 1.0f / Izz : 0.0f;

            inverseInertiaWorld = inverseInertiaLocal;
        }

        RigidBody() {
            updateInertiaTensor();
        }
        // ^^^^^ Inertia Tensor ^^^^^ //

        float rollingResistance = 0.02f;
        /**
         * AUDIT: DISABLED
         * kinetic energy loss from a non-spherical object rolling
         */
};
