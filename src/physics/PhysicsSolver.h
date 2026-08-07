#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <cstdint>
#include <unordered_map>
#include "rigidbody.h"
#include "collisioninfo.h"

class PhysicsSolver {
    public:
        PhysicsSolver();
        ~PhysicsSolver();

        void integrate(RigidBody& body, float deltaTime);
        void detectAndResolve(std::vector<RigidBody>& bodies);

        int lastContactCount = 0; // number of active contacts (updated by detectAndResolve)

    private:
        /**
         * A single contact point with all precomputed solver data and accumulated impulses.
         * One colliding pair may produce multiple Contact instances (one per manifold point).
         */
        struct Contact {
            RigidBody* a;
            RigidBody* b;
            CollisionInfo info;

            glm::vec3 rA; // contact point - a.position
            glm::vec3 rB; // contact point - b.position

            float effectiveMassNormal;
            float effectiveMassTangent1;
            float effectiveMassTangent2;

            glm::vec3 tangent1;
            glm::vec3 tangent2;

            float initialRelVelN;

            float accumulatedNormalImpulse = 0.0f;
            float accumulatedTangentImpulse1 = 0.0f;
            float accumulatedTangentImpulse2 = 0.0f;
        };

        /**
         * Accumulated impulses kept between frames for warm starting.
         */
        struct CachedImpulse {
            float normal = 0.0f;
            float tangent1 = 0.0f;
            float tangent2 = 0.0f;
        };

        /**
         * Identity of a contact point across frames: the body pair plus the
         * geometric feature ID that produced it.
         */
        struct ContactKey {
            RigidBody* a;
            RigidBody* b;
            uint32_t featureId;

            bool operator==(const ContactKey& o) const {
                return a == o.a && b == o.b && featureId == o.featureId;
            }
        };

        struct ContactKeyHash {
            std::size_t operator()(const ContactKey& k) const noexcept {
                std::size_t h = std::hash<const void*>()(k.a);
                h ^= std::hash<const void*>()(k.b) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
                h ^= std::hash<uint32_t>()(k.featureId) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
                return h;
            }
        };

        // --- Methods ---
        std::vector<CollisionInfo> generateFloorContacts(const RigidBody& body) const;
        void precomputeContact(Contact& c);
        void warmStart(std::vector<Contact>& contacts);
        void solveVelocities(std::vector<Contact>& contacts);
        void matchAndLoadCache(std::vector<Contact>& contacts);
        void storeCache(const std::vector<Contact>& contacts);

        // Spatial-hash broadphase: fills outPairs with candidate (i, j) index pairs
        // whose world AABBs share at least one grid cell.
        void buildBroadphasePairs(const std::vector<RigidBody>& bodies,
                                  const std::vector<glm::vec3>& aabbMin,
                                  const std::vector<glm::vec3>& aabbMax,
                                  std::vector<std::pair<int, int>>& outPairs) const;

        // --- State ---
        RigidBody floorBody;
        std::unordered_map<ContactKey, CachedImpulse, ContactKeyHash> contactCache;

        // --- Constants ---
        static constexpr int SOLVER_ITERATIONS = 10;
        static constexpr float FLOOR_Y = 0.0f;
        static constexpr float FLOOR_THICKNESS = 1.0f;
        static constexpr float FLOOR_HALF_EXTENT = 500.0f;
        static constexpr float REST_THRESHOLD = 0.5f;
        static constexpr float BAUMGARTE_FACTOR = 0.1f;
        static constexpr float PENETRATION_SLOP = 0.005f;
        static constexpr float FACE_CONTACT_EPSILON = 0.005f;
        static constexpr float WARM_START_SCALE = 0.8f;
        static constexpr float FIXED_DT = 1.0f / 60.0f;
        static constexpr float SPATIAL_CELL_SIZE = 2.0f; // grid cell edge; ~= largest body extent
};
