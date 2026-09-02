#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <cstdint>

// ============================================================================
// Telemetry — a research-grade, per-step snapshot of the entire simulation.
//
// The PhysicsSolver populates a single TelemetryFrame at the end of step()
// when captureTelemetry is enabled (zero cost otherwise). It aggregates data
// that already lives on the bodies / constraints / contact set into ONE flat,
// value-typed, copyable record so an experiment (or Day 33's telemetry system)
// can read `solver.lastTelemetry` — or keep a rolling history of frames —
// WITHOUT reaching into solver internals or modifying the physics engine.
//
// Design goals:
//   * Self-contained (no pointers into live state): every field is a value, so
//     a frame can be copied, buffered, serialised, or sent across a boundary.
//   * Complete: object states, contacts, impulses, forces, torques, per-
//     constraint errors, system energy, momentum, and sleep transitions.
//   * Forward-compatible: adding fields here never requires touching the solver
//     loop beyond the single fill-in site, and never changes physics results.
//
// Units are SI throughout (m, m/s, rad/s, kg, N, N*m, J, kg*m/s).
// ============================================================================

// Stable identity for a body within a frame. `index` is the body's position in
// the std::vector<RigidBody> passed to step(); it is the caller's job to keep
// that vector stable across frames if they want index continuity.
struct BodyTelemetry {
    int       index        = -1;
    int       shape        = 0;              // ShapeType as int (0=Box, 1=Sphere)
    bool      isStatic      = false;         // inverseMass == 0
    bool      asleep        = false;

    glm::vec3 position       = glm::vec3(0.0f);
    glm::vec3 velocity       = glm::vec3(0.0f);
    glm::vec3 angularVelocity = glm::vec3(0.0f);
    glm::quat orientation    = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    float     mass           = 0.0f;
    float     kineticLinear  = 0.0f;         // 1/2 m v^2
    float     kineticRotational = 0.0f;      // 1/2 w . (I_world w)
    float     potential      = 0.0f;         // m g y  (relative to y=0)

    // Aerodynamic state this step (0 when aero disabled / static / asleep).
    glm::vec3 aeroForce      = glm::vec3(0.0f);
    glm::vec3 aeroTorque     = glm::vec3(0.0f);
    glm::vec3 relativeAirVelocity = glm::vec3(0.0f);
    float     aeroPower      = 0.0f;         // F.v (<=0 for pure drag)
};

// One solved contact (mirrors PhysicsSolver::ContactDebug, but value-typed and
// with the involved body indices resolved where possible).
struct ContactTelemetry {
    glm::vec3 point          = glm::vec3(0.0f);
    glm::vec3 normal         = glm::vec3(0.0f); // A -> B convention
    float     penetration    = 0.0f;
    float     normalImpulse   = 0.0f;          // accumulated over the step
    float     frictionImpulse = 0.0f;
    bool      floorContact    = false;         // contact against floor/plane
    int       bodyA          = -1;             // dynamic body index, or -1 if static/plane
    int       bodyB          = -1;
};

// Per-constraint error / load. `type` distinguishes the four constraint kinds.
struct ConstraintTelemetry {
    enum class Type { Spring, Hinge, Rope, Pulley };
    Type      type           = Type::Spring;

    // Generic "how far is this constraint from satisfied" scalar (SI):
    //   Spring : |extension|            (m)  -- distance from rest length
    //   Hinge  : |positionError|        (m)  -- world anchor separation
    //   Rope   : max(0, len - maxLen)   (m)  -- overshoot past the rope limit
    //   Pulley : |lenA+lenB - L|        (m)  -- rope-length residual
    float     error          = 0.0f;

    // Generic load carried this step (SI):
    //   Spring : |force|  (N)
    //   Hinge  : |accumulated linear impulse| (N*s)  [reported as-is]
    //   Rope   : tension  (impulse magnitude)
    //   Pulley : tension
    float     load           = 0.0f;

    bool      active         = true;           // rope/pulley taut; spring/hinge always true
};

struct TelemetryFrame {
    // --- Time ---
    std::uint64_t frameIndex = 0;   // monotonically increasing step counter
    double        simTime    = 0.0; // accumulated simulated time (s)
    float         dt         = 0.0f; // timestep of THIS step (s)

    // --- Per-object state ---
    std::vector<BodyTelemetry> bodies;

    // --- Contacts solved this step (empty unless contact capture is on) ---
    std::vector<ContactTelemetry> contacts;
    int  contactCount = 0;          // solver's reported contact count (always set)

    // --- Constraints ---
    std::vector<ConstraintTelemetry> constraints;

    // --- System aggregates ---
    float kineticLinear     = 0.0f;
    float kineticRotational = 0.0f;
    float potential         = 0.0f;
    float springEnergy      = 0.0f; // 1/2 k x^2 summed over springs
    float mechanicalEnergy  = 0.0f; // KE_lin + KE_rot + PE + springEnergy
    float aeroPower         = 0.0f; // sum of F.v over bodies (rate of aero work, W)
    double aeroWorkCumulative = 0.0; // running integral of aeroPower*dt (J, <=0)

    glm::vec3 linearMomentum  = glm::vec3(0.0f);
    glm::vec3 angularMomentum = glm::vec3(0.0f); // about the world origin

    // --- Sleeping ---
    int awakeCount   = 0;
    int sleepingCount = 0;
    int justSlept    = 0; // bodies that transitioned awake->asleep this step
    int justWoke     = 0; // bodies that transitioned asleep->awake this step

    // --- Solver configuration snapshot (so a frame is self-describing) ---
    int   solverIterations   = 0;
    int   positionIterations = 0;
    bool  gravityEnabled     = true;
    bool  aerodynamicsEnabled = false;
    float airDensity         = 0.0f;
    glm::vec3 windVelocity   = glm::vec3(0.0f);

    // Largest contact penetration this step (quick numerical-health signal).
    float maxPenetration     = 0.0f;
};
