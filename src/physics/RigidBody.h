#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Shape type determines collision geometry and inertia computation.
enum class ShapeType : int {
    Box = 0,
    Sphere = 1
};

class RigidBody {
    public:
        // --- Shape ---
        ShapeType shape = ShapeType::Box;

        // For Box: scale defines the full extents (halfExtents = scale * 0.5).
        // For Sphere: radius is the collision radius; scale is used only for
        //             rendering (set scale = vec3(radius*2) for visual match).
        float radius = 0.5f;

        glm::vec3 position = glm::vec3(0.0f, 25.0f, 0.0f);
        glm::vec3 velocity = glm::vec3(0.0f);
        glm::vec3 acceleration = glm::vec3(0.0f);
        glm::vec3 scale = glm::vec3(1.0f);

        float mass = 1.0f;
        float inverseMass = 1.0f;
        float restitution = 0.4f;
        float friction = 0.5f;

        bool isColliding = false;

        // Sleeping / island state (managed by PhysicsSolver).
        bool asleep = false;
        float sleepTimer = 0.0f;
        int islandId = -1;

        glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        glm::vec3 angularVelocity = glm::vec3(0.0f);
        glm::vec3 torque = glm::vec3(0.0f);

        // Split-impulse position correction. NOT real velocity -- these only
        // push bodies out of penetration during the position solve, then are
        // discarded each step, so overlap correction injects no energy (no creep).
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

            float Ixx, Iyy, Izz;

            if (shape == ShapeType::Sphere) {
                // Solid sphere: I = (2/5) m r² (isotropic)
                const float I = (2.0f / 5.0f) * mass * radius * radius;
                Ixx = Iyy = Izz = I;
            } else {
                // Box: I_axis = (m/12)(side_b² + side_c²)
                Ixx = (mass * (scale.y * scale.y + scale.z * scale.z)) / 12.0f;
                Iyy = (mass * (scale.x * scale.x + scale.z * scale.z)) / 12.0f;
                Izz = (mass * (scale.x * scale.x + scale.y * scale.y)) / 12.0f;
            }

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
};
