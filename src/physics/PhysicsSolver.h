#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <csdint>
#include "rigidbody.h"
#include "collisioninfo.h"

class PhysicsSolver {
    public:
        PhysicsSolver();
        ~PhysicsSolver();

        void integrate(RigidBody& body, float deltaTime);
        // void floorCollision(RigidBody& body);
        void detectAndResolve(std::vector<RigidBody>& bodies);

        int lastContactCount = 0; // updated by detectAndResolve() --> number of active contacts

    private:
        /**
         * A single contact point with all precomputed solver data and accumulated impulses.
         * One colliding pair may produce multiple Contact instances (one per manifold point).
         */
        struct Contact {
            RigidBody* a;
            RigidBody* b;
            CollisionInfo info;

            // Precomputed geometry (set once per frame after contact generation)
            glm::vec3 rA; // contact point - a.position
            glm::vec3 rB; // contact point - b.position
            
            float effectiveMassNormal; // 1 / denominator for normal impulse
            float effectiveMassTangent1; // 1 / denominator for friction along tangent1
            float effectiveMassTangent2; // 1 / denominator for friction along tangent2

            glm::vec3 tangent1; // first friction direction
            glm::vec3 tangent2; // second friction direction (= cross(normal, tangent1))

            float initialRelVelN; // relative velocity along normal at contact creation (for restitution)

            // Accumulated impulses (warm-started from cache, updated each iteration)
            float accumulatedNormalImpulse = 0.0f;
            float accumulatedTangentImpulse1 = 0.0f;
            float accumulatedTangentImpulse2 = 0.0f;
        };


        /**
         * Cached contact from the previous frame, used for warm-starting
         */
        struct CachedContact {
            RigidBody* a;
            RigidBody* b;

            uint32_t featureId;
            float accumulatedNormalImpulse;
            float accumulatedTangentImpulse1;
            float accumulatedTangentImpulse2;
        };

        // --- Methods ---
        std::vector<CollisionInfo> generateFloorContacts(const RigidBody& body) const;
        void precomputeContact(Contact& c);
        void warmStart(std::vector<Contact>& contacts);
        void solveVelocities(std::vector<Contact>& contacts);
        void matchAndLoadCache(std::vector<Contact>& contacts);
        void storeCache(const std::vector<Contact>& contacts);

        // --- State ---
        RigidBody floorBody;
        std::vector<CachedContact> contactCache;

        // --- Constants ---
        static constexpr int SOLVER_ITERATIONS = 10;
        static constexpr float FLOOR_Y = 0.0f;
        static constexpr float FLOOR_THICKNESS = 1.0f;
        static constexpr float FLOOR_HALF_EXTENT = 500.0f;
        static constexpr float REST_THRESHOLD = 0.5f; // relative velocity below which restitution is zeroed
        static constexpr float BAUMGARTE_FACTOR = 0.1f; // fraction of penetration corrected per timestep via bias
        static constexpr float PENETRATION_SLOP = 0.005f; // penetration below this is tolerated (prevents contact flicker)
        static constexpr float FACE_CONTACT_EPSILON = 0.005f;
        static constexpr float WARM_START_SCALE = 0.8f; // damping on cached impulses to account for geometry drift
        static constexpr float FIXED_DT = 1.0f / 60.0f; // needed for Baumgarte bias computation
}        