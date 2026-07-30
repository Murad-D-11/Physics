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

        // vvvvv Inertia Tensor vvvvv //
        /**
         * Replaces the old scalar inverseInertia. A scalar is only correct for a sphere;
         * a box resists spinning differently about each of its own axes, so the moment
         * of inertia has to be a 3x3 matrix, not a single number.
         */

        glm::mat3 inertiaLocal = glm::mat3(1.0f); // inertia tensor in the body's own local frame
        glm::mat3 inverseInertiaLocal = glm::mat3(1.0f); // inverse of the above, precomputed once so per-step work is just a matrix multiplication

        /**
         * World-space inverse inertia tensor. As the body rotates, its resistance to spinning
         * about world-space axes changes even though nothing changed in its own local frame. PhysicsSolver
         * recomputes this every step from inverseInertiaLocal and the body's current orientation:
         * inverseInertiaWorld = R * inverseInertiaLocal * R^T
         */
        glm::mat3 inverseInertiaWorld = glm::mat3(1.0f);

        void updateInertiaTensor() {
            if (inverseMass == 0.0f) {
                // static body: infinite mass --> infinite inertia --> zero inverse inertia
                inertiaLocal = glm::mat3(0.0f);
                inverseInertiaLocal = glm::mat3(0.0f);
                inverseInertiaWorld = glm::mat3(0.0f);
                return;
            }

            // standard solid-cuboid moment of inertia about each local axis
            // (scale here is the full extent along each axis, not the half-extent)
            const float Ixx = (mass * (scale.y * scale.y + scale.z * scale.z)) / 12.0f;
            const float Iyy = (mass * (scale.x * scale.x + scale.z * scale.z)) / 12.0f;
            const float Izz = (mass * (scale.x * scale.x + scale.y * scale.y)) / 12.0f;

            inertiaLocal = glm::mat3(0.0f);
            inertiaLocal[0][0] = Ixx;
            inertiaLocal[1][1] = Iyy;
            inertiaLocal[2][2] = Izz;

            inverseInertiaLocal = glm::mat3(0.0f);
            inverseInertiaLocal[0][0] = (Ixx > 0.0f) ? 1.0f / Ixx : 0.0f; // takes inverse inertia for each axis + avoids dividing by zero
            inverseInertiaLocal[1][1] = (Iyy > 0.0f) ? 1.0f / Iyy : 0.0f;
            inverseInertiaLocal[2][2] = (Izz > 0.0f) ? 1.0f / Izz : 0.0f;

            // world tensor starts equal to the local one (identity orientation);
            // PhysicsSolver keeps it up to date every step after this
            inverseInertiaWorld = inverseInertiaLocal;
        }

        RigidBody() {
            updateInertiaTensor();
        }

        // ^^^^^ Inertia Tensor ^^^^^ //

        float rollingResistance = 0.02f; 
        /**
         * AUDIT: DISABLED ^^^
         * kinetic energy loss from a non-spherical object rolling 
        */ 
};