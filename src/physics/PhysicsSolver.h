#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <utility>
#include <cstdint>
#include <unordered_map>
#include "obb.h"
#include "rigidbody.h"
#include "collisioninfo.h"

class PhysicsSolver {
    public:
        PhysicsSolver();
        ~PhysicsSolver();

        void integrate(RigidBody& body, float deltaTime);
        void detectAndResolve(std::vector<RigidBody>& bodies);
        void step(std::vector<RigidBody>& bodies, float dt);

        int lastContactCount = 0;

    private:
        struct Contact {
            RigidBody* a;
            RigidBody* b;
            CollisionInfo info;

            glm::vec3 rA;
            glm::vec3 rB;

            float effectiveMassNormal;
            float effectiveMassTangent1;
            float effectiveMassTangent2;

            glm::vec3 tangent1;
            glm::vec3 tangent2;

            float initialRelVelN;

            float accumulatedNormalImpulse = 0.0f;
            float accumulatedTangentImpulse1 = 0.0f;
            float accumulatedTangentImpulse2 = 0.0f;
            float accumulatedPositionImpulse = 0.0f; // split-impulse position solve
        };

        struct CachedImpulse {
            float normal = 0.0f;
            float tangent1 = 0.0f;
            float tangent2 = 0.0f;
        };

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

        struct TOIResult {
            bool hit = false;
            float toi = 0.0f;
            float closingSpeed = 0.0f;
        };

        // --- Discrete solver internals ---
        std::vector<CollisionInfo> generateFloorContacts(const RigidBody& body) const;
        void precomputeContact(Contact& c);
        void warmStart(std::vector<Contact>& contacts);
        void solveVelocities(std::vector<Contact>& contacts);
        void solvePositions(std::vector<Contact>& contacts);
        void integratePseudoVelocities(std::vector<RigidBody>& bodies);
        void matchAndLoadCache(std::vector<Contact>& contacts);
        void storeCache(const std::vector<Contact>& contacts);
        void buildBroadphasePairs(const std::vector<RigidBody>& bodies,
                                  const std::vector<glm::vec3>& aabbMin,
                                  const std::vector<glm::vec3>& aabbMax,
                                  std::vector<std::pair<int, int>>& outPairs) const;

        // --- CCD internals ---
        static OBB predictOBB(const RigidBody& b, float t);
        void applyGravity(RigidBody& body, float dt) const;
        void integratePositions(std::vector<RigidBody>& bodies, float dt);
        bool isCCDCandidate(const RigidBody& b, float dt) const;
        TOIResult computePairTOI(const RigidBody& a, const RigidBody& b, float dt) const;
        TOIResult computeFloorTOI(const RigidBody& b, float dt) const;
        TOIResult findEarliestTOI(const std::vector<RigidBody>& bodies, float dt) const;

        // --- Sleeping / islands ---
        void updateSleeping(std::vector<RigidBody>& bodies, float dt);
        void wakeIsland(std::vector<RigidBody>& bodies, int islandId);

        // --- State ---
        RigidBody floorBody;
        std::unordered_map<ContactKey, CachedImpulse, ContactKeyHash> contactCache;
        std::vector<std::pair<int, int>> islandEdges;
        float sceneMinThickness = 1.0f; // smallest collider extent in the scene (set each step)

        // --- Constants ---
        static constexpr int SOLVER_ITERATIONS = 10;
        static constexpr int POSITION_ITERATIONS = 4;
        static constexpr float FLOOR_Y = 0.0f;
        static constexpr float FLOOR_THICKNESS = 1.0f;
        static constexpr float FLOOR_HALF_EXTENT = 500.0f;
        static constexpr float REST_THRESHOLD = 0.5f;
        static constexpr float POSITION_BETA = 0.2f;
        static constexpr float PENETRATION_SLOP = 0.005f;
        static constexpr float FACE_CONTACT_EPSILON = 0.005f;
        static constexpr float WARM_START_SCALE = 0.8f;
        static constexpr float FIXED_DT = 1.0f / 60.0f;
        static constexpr float SPATIAL_CELL_SIZE = 2.0f;

        // CCD tuning
        static constexpr int   CCD_MAX_SUBSTEPS   = 4;
        static constexpr int   CCD_MAX_ITERATIONS = 32;
        static constexpr float CCD_TOLERANCE      = 0.01f;
        static constexpr float CCD_TIME_EPS       = 1e-5f;
        static constexpr float CCD_MOTION_FACTOR  = 0.8f;

        // Sleeping tuning
        static constexpr float SLEEP_LINEAR_THRESHOLD  = 0.15f;
        static constexpr float SLEEP_ANGULAR_THRESHOLD = 0.20f;
        static constexpr float SLEEP_TIME             = 0.5f;
        static constexpr float ISLAND_CONTACT_MARGIN  = 0.02f;
        static constexpr float REST_DAMPING   = 0.80f; // per-frame velocity retention for near-rest bodies
        static constexpr float SETTLE_LINEAR  = 0.30f; // below this speed, near-rest damping engages (m/s)
        static constexpr float SETTLE_ANGULAR = 0.40f; // below this spin, near-rest damping engages (rad/s)
        static constexpr float SLEEP_DECAY_RATE = 3.0f; // a "fast" frame costs 3 slow frames of accumulated rest
};
