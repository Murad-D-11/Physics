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
#include "Constraint.h"
#include "Telemetry.h"

class PhysicsSolver {
    public:
        PhysicsSolver();
        ~PhysicsSolver();

        void integrate(RigidBody& body, float deltaTime);
        void detectAndResolve(std::vector<RigidBody>& bodies);
        void step(std::vector<RigidBody>& bodies, float dt);

        // Projected (silhouette) area of a body perpendicular to unit flow
        // direction `flowDir` (world space). Sphere: pi r^2 (orientation-free).
        // Box: orientation-dependent silhouette Sum(faceArea * |flowDir.axis|).
        // Public + static: an AI-readable geometric query independent of solver
        // state, useful for observations and offline analysis.
        static float projectedArea(const RigidBody& body, const glm::vec3& flowDir);

        int lastContactCount = 0;

        // ====================================================================
        // Static planes (slopes, walls, ramps). Each has an arbitrary normal.
        // Contact generation derives the normal from actual plane geometry,
        // NOT from hard-coded (0,1,0) assumptions.
        // ====================================================================
        struct StaticPlane {
            glm::vec3 point    = glm::vec3(0.0f); // any point on the plane (centre of the surface)
            glm::vec3 normal   = glm::vec3(0.0f, 1.0f, 0.0f); // outward normal (unit)
            float friction     = 0.6f;
            float restitution  = 0.3f;

            // Finite extent: if halfExtent > 0, the plane is bounded to a rectangle
            // of size (2*halfExtent.x) × (2*halfExtent.y) centred at `point`, measured
            // along the plane's two tangent axes. Set halfExtent = (0,0) for infinite.
            glm::vec2 halfExtent = glm::vec2(0.0f); // (0,0) = infinite plane

            // Tangent axes spanning the plane surface (auto-computed if zero).
            glm::vec3 tangent1 = glm::vec3(0.0f);
            glm::vec3 tangent2 = glm::vec3(0.0f);
        };
        std::vector<StaticPlane> planes; // user-facing: add slopes here

        // ====================================================================
        // Constraints (springs, hinges). Add to these vectors before stepping.
        // ====================================================================
        std::vector<SpringConstraint> springs;
        std::vector<HingeConstraint>  hinges;
        std::vector<RopeConstraint>   ropes;
        std::vector<PulleyConstraint> pulleys;

        // --- Opt-in solver diagnostics (used by the headless contact tests) ---
        // When captureDiagnostics is true, detectAndResolve records the solved
        // contact set (post velocity+position solve) so tests can audit normals,
        // penetration, and accumulated normal/friction impulses. Zero cost when
        // left false.
        struct ContactDebug {
            const void* a = nullptr;
            const void* b = nullptr;
            glm::vec3 point = glm::vec3(0.0f);
            glm::vec3 normal = glm::vec3(0.0f);
            float penetration = 0.0f;
            float normalImpulse = 0.0f;
            float frictionImpulse = 0.0f;
            bool floorContact = false;
        };
        bool captureDiagnostics = false;
        std::vector<ContactDebug> lastSolvedContacts;

        // ====================================================================
        // Research telemetry (opt-in). When captureTelemetry is true, step()
        // fills `lastTelemetry` with a complete value-typed snapshot of the
        // frame (object states, contacts, impulses, forces, torques, constraint
        // errors, energy, momentum, sleep transitions). Enabling it also turns
        // on contact capture so the frame's contact list is populated.
        //
        // Zero cost when disabled. The snapshot lives entirely in owned value
        // types, so Day 33's telemetry system can read solver.lastTelemetry (or
        // copy it into a rolling history buffer) without touching the physics.
        // ====================================================================
        bool captureTelemetry = false;
        TelemetryFrame lastTelemetry;

        // Sleeping is a performance optimization, not a stabilization mechanism.
        // Disable it to audit the raw long-term stability of the constraint
        // solver (a sleeping stack is frozen and would hide any solver drift).
        bool sleepingEnabled = true;

        // Gravity toggle. The validation suite disables gravity for free-space
        // mechanics tests (momentum / angular-momentum conservation, torque
        // response, gyroscopic) so those results are not polluted by weight.
        bool gravityEnabled = true;

        // ====================================================================
        // Aerodynamic environment (physically based, opt-in).
        //
        // Disabled by default so existing behaviour/tests are unaffected. When
        // enabled, applyAerodynamics() applies quadratic drag
        //     F_d = 1/2 * rho * Cd * A(orientation) * |v_rel| * v_rel
        // to every dynamic body, where v_rel = windVelocity - v_body is the
        // relative airflow. For boxes the drag is split across the windward
        // faces so an off-axis body experiences a naturally-arising torque; the
        // model never injects energy (pure drag removes mechanical energy).
        //
        // These are real physical quantities (SI units) intended to become AI
        // observations / learnable parameters.
        // ====================================================================
        bool  aerodynamicsEnabled = false;
        float airDensity          = 1.225f;            // rho at sea level, 15C (kg/m^3)
        glm::vec3 windVelocity    = glm::vec3(0.0f);   // ambient air velocity (m/s)

        // Runtime-adjustable iteration counts for convergence experiments.
        // Default to the compile-time constants; tests can override per-instance.
        int solverIterations = SOLVER_ITERATIONS;
        int positionIterations = POSITION_ITERATIONS;

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
        std::vector<CollisionInfo> generatePlaneContacts(const RigidBody& body, const StaticPlane& plane) const;
        void precomputeContact(Contact& c);
        void warmStart(std::vector<Contact>& contacts);
        void solveVelocities(std::vector<Contact>& contacts, bool reverse);
        void solvePositions(std::vector<Contact>& contacts, bool reverse);
        void integratePseudoVelocities(std::vector<RigidBody>& bodies);
        void precomputeHinges();
        void solveHingeVelocities();
        void solveHingePositions();
        void prepareRopes();
        void warmStartRopes();
        void solveRopeVelocities();
        void solveRopePositions();
        void preparePulleys();
        void warmStartPulleys();
        void solvePulleyVelocities();
        void matchAndLoadCache(std::vector<Contact>& contacts);
        void storeCache(const std::vector<Contact>& contacts);
        void buildBroadphasePairs(const std::vector<RigidBody>& bodies,
                                  const std::vector<glm::vec3>& aabbMin,
                                  const std::vector<glm::vec3>& aabbMax,
                                  std::vector<std::pair<int, int>>& outPairs) const;

        // --- CCD internals ---
        static OBB predictOBB(const RigidBody& b, float t);
        void applyGravity(RigidBody& body, float dt) const;
        void applyAerodynamics(RigidBody& body, float dt) const;
        void integratePositions(std::vector<RigidBody>& bodies, float dt);
        bool isCCDCandidate(const RigidBody& b, float dt) const;
        TOIResult computePairTOI(const RigidBody& a, const RigidBody& b, float dt) const;
        TOIResult computeFloorTOI(const RigidBody& b, float dt) const;
        TOIResult findEarliestTOI(const std::vector<RigidBody>& bodies, float dt) const;

        // --- Sleeping / islands ---
        void updateSleeping(std::vector<RigidBody>& bodies, float dt);
        void wakeIsland(std::vector<RigidBody>& bodies, int islandId);

        // --- Telemetry ---
        // Populates lastTelemetry from the current post-step state. justSlept /
        // justWoke are the sleep transitions counted by the caller (step()),
        // which has the pre-updateSleeping asleep flags available.
        void captureTelemetryFrame(const std::vector<RigidBody>& bodies, float dt,
                                   int justSlept, int justWoke);
        std::uint64_t telemetryFrameIndex = 0;
        double telemetrySimTime = 0.0;
        double telemetryAeroWork = 0.0; // cumulative integral of aero power (J)

        // --- State ---
        RigidBody floorBody;
        RigidBody planeBody; // static body used as B-side for all plane contacts
        std::unordered_map<ContactKey, CachedImpulse, ContactKeyHash> contactCache;
        std::vector<std::pair<int, int>> islandEdges;
        float sceneMinThickness = 1.0f; // smallest collider extent in the scene (set each step)

        // --- Constants ---
        static constexpr int SOLVER_ITERATIONS = 40;
        static constexpr int POSITION_ITERATIONS = 8;
        static constexpr float FLOOR_Y = 0.0f;
        static constexpr float FLOOR_THICKNESS = 1.0f;
        static constexpr float FLOOR_HALF_EXTENT = 500.0f;
        static constexpr float REST_THRESHOLD = 0.5f;
        static constexpr float POSITION_BETA = 0.2f;
        // Cap on how fast the (frictionless) split-impulse position solve may
        // remove overlap, in metres/second. Deep penetration is then resolved
        // gently over several frames instead of one violent lateral shove --
        // the source of angled dominoes sliding off each other. Below the slop,
        // no correction runs at all, so a settled pile goes completely still.
        static constexpr float MAX_CORRECTION_SPEED = 0.4f;
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
