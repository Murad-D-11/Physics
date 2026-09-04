#pragma once
// ===========================================================================
// Scene — a self-contained, reloadable description of a physics world.
//
// A Scene owns the full contents of one demo: its rigid bodies AND its
// constraints (springs / hinges / ropes / pulleys) and static planes, plus the
// per-scene aerodynamic environment. The SceneManager makes a scene's data the
// live simulation state and copies its constraint definitions into the solver
// (see SceneManager::applyToSolver). Nothing here touches the physics solver's
// implementation — scenes only *describe* a world.
//
// Constraints are stored by BODY INDEX rather than by raw RigidBody* so a scene
// is relocatable and pointer-safe: the manager resolves indices to stable
// addresses in the scene's own `bodies` vector at load time. Use index -1 to
// mean "world / static anchor" (matching the solver's nullptr convention).
// ===========================================================================

#include <vector>
#include <string>
#include <glm/glm.hpp>

#include "../physics/rigidbody.h"
#include "../physics/physicssolver.h"
#include "../physics/Constraint.h"

// ---------------------------------------------------------------------------
// Shared mass-property helpers (single definition, reused by every scene so
// there is no per-scene duplication). Header-inline to keep this drop-in.
// ---------------------------------------------------------------------------
inline void sceneSetCubeMass(RigidBody& body, float mass) {
    body.shape = ShapeType::Box;
    body.mass = mass;
    body.inverseMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    body.updateInertiaTensor();
}

inline void sceneSetSphereMass(RigidBody& body, float mass, float radius) {
    body.shape = ShapeType::Sphere;
    body.radius = radius;
    body.scale = glm::vec3(radius * 2.0f); // visual scale matches collision radius
    body.mass = mass;
    body.inverseMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    body.updateInertiaTensor();
}

inline void sceneMakeStatic(RigidBody& body) {
    body.inverseMass = 0.0f;
    body.inverseInertiaLocal = glm::mat3(0.0f);
    body.inverseInertiaWorld = glm::mat3(0.0f);
}

// ---------------------------------------------------------------------------
// Reusable APPARATUS builders. Every mechanical scene is assembled from real
// physical parts (frames, posts, rails, guide walls, ramps) rather than
// invisible anchors, so the geometry itself constrains the intended motion.
// Each helper appends a static RigidBody to `bodies` and returns its index.
// (Static bodies are rendered, so the apparatus is visible.)
// ---------------------------------------------------------------------------

// A static box part (beam / post / rail / wall / base). `fullExtents` is the
// full size in metres; `orientation` lets it be tilted (e.g. an A-frame leg or
// a ramp). Returns the new body index.
inline int sceneAddStaticBox(std::vector<RigidBody>& bodies,
                             const glm::vec3& center,
                             const glm::vec3& fullExtents,
                             const glm::quat& orientation = glm::quat(1, 0, 0, 0),
                             float friction = 0.7f) {
    RigidBody b;
    b.scale = fullExtents;
    b.position = center;
    b.orientation = orientation;
    sceneSetCubeMass(b, 1.0f);   // mass is irrelevant once static
    sceneMakeStatic(b);
    b.friction = friction;
    b.restitution = 0.0f;
    bodies.push_back(b);
    return static_cast<int>(bodies.size()) - 1;
}

// A pair of static walls forming a VERTICAL GUIDE SHAFT centred on x=`cx`,
// z=0, running from y=`yBottom` to y=`yTop`. A body of half-width `slot` in X
// dropped between them can only move vertically (the walls physically stop
// lateral drift). `gap` is the clear channel width. Appends two walls.
inline void sceneAddGuideShaft(std::vector<RigidBody>& bodies,
                               float cx, float yBottom, float yTop,
                               float gap, float depth = 1.2f,
                               float wallThickness = 0.15f,
                               float friction = 0.05f) {
    const float h = yTop - yBottom;
    const float cy = 0.5f * (yBottom + yTop);
    const float half = gap * 0.5f + wallThickness * 0.5f;
    sceneAddStaticBox(bodies, glm::vec3(cx - half, cy, 0.0f),
                      glm::vec3(wallThickness, h, depth), glm::quat(1,0,0,0), friction);
    sceneAddStaticBox(bodies, glm::vec3(cx + half, cy, 0.0f),
                      glm::vec3(wallThickness, h, depth), glm::quat(1,0,0,0), friction);
}

// A static angled ramp slab whose TOP surface passes through `surfacePoint`
// with unit up-normal `normal`, extent `alongLen` down-slope and `width`
// across, and thickness `thick`. Returns the body index. The matching
// collision StaticPlane should use the same point/normal so the visible slab
// and the contact surface coincide.
inline int sceneAddRampSlab(std::vector<RigidBody>& bodies,
                            const glm::vec3& surfacePoint,
                            const glm::vec3& normal,
                            float alongLen, float width, float thick,
                            const glm::quat& orientation,
                            float friction = 0.5f) {
    const glm::vec3 n = glm::normalize(normal);
    // Slab centre sits half a thickness BELOW the surface along the normal.
    const glm::vec3 center = surfacePoint - n * (thick * 0.5f);
    return sceneAddStaticBox(bodies, center, glm::vec3(alongLen, thick, width),
                             orientation, friction);
}

// ---------------------------------------------------------------------------
// Index-based constraint descriptions (pointer-safe, copyable). Body indices
// refer to positions in Scene::bodies; -1 means a fixed world anchor.
// ---------------------------------------------------------------------------
struct SpringDesc {
    int   bodyA = -1, bodyB = -1;
    glm::vec3 localAnchorA = glm::vec3(0.0f);
    glm::vec3 localAnchorB = glm::vec3(0.0f);
    float restLength = 1.0f;
    float stiffness  = 50.0f;
    float damping    = 2.0f;
};

struct HingeDesc {
    int   bodyA = -1, bodyB = -1;
    glm::vec3 localAnchorA = glm::vec3(0.0f);
    glm::vec3 localAnchorB = glm::vec3(0.0f);
    glm::vec3 localAxisA   = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 localAxisB   = glm::vec3(0.0f, 0.0f, 1.0f);
    // Optional revolute limits (radians). If min >= max (default), the hinge is
    // free to spin. Used to build axles that only swing through a range: a
    // cantilever segment that can droop but not fold back, a trebuchet arm, a
    // Rube-Goldberg gate/pin. Maps straight onto HingeConstraint.
    float angleLimitMin = 0.0f;
    float angleLimitMax = 0.0f;         // min >= max => no limit (free spin)
    // Optional motor: if maxMotorTorque > 0 the hinge drives toward
    // targetAngularVelocity about its axis (e.g. a powered gate).
    float targetAngularVelocity = 0.0f;
    float maxMotorTorque        = 0.0f; // 0 => motor disabled
};

struct RopeDesc {
    int   bodyA = -1, bodyB = -1;
    glm::vec3 localAnchorA = glm::vec3(0.0f);
    glm::vec3 localAnchorB = glm::vec3(0.0f);
    float maxLength = 2.0f;
};

struct PulleyDesc {
    int   bodyA = -1, bodyB = -1;
    glm::vec3 localAnchorA = glm::vec3(0.0f);
    glm::vec3 localAnchorB = glm::vec3(0.0f);
    glm::vec3 pulleyPos = glm::vec3(0.0f, 5.0f, 0.0f);
    float totalRopeLength = 4.0f;
    float pulleyRadius    = 0.25f;
};

// ---------------------------------------------------------------------------
// Scene base class.
// ---------------------------------------------------------------------------
class Scene {
public:
    virtual ~Scene() = default;

    // Human-readable name (shown in the window / logs). Override optionally.
    virtual const char* name() const { return "Scene"; }

    // Short human description of what the scene shows. Override optionally.
    virtual const char* description() const { return ""; }

    // The physical principle the scene demonstrates. Override optionally.
    virtual const char* principle() const { return ""; }

    // Adjustable parameters exposed to the GUI / hotkeys. A scene registers
    // named scalar knobs here in its constructor; changing a value and calling
    // reset() rebuilds the scene with the new setting (load() reads them). This
    // keeps parameters data-driven without a bespoke UI per scene.
    struct Param {
        std::string name;
        float value;
        float minValue;
        float maxValue;
    };
    std::vector<Param> params;

    // Look up a parameter's current value by name, or `fallback` if absent.
    float param(const std::string& key, float fallback) const {
        for (const auto& p : params) if (p.name == key) return p.value;
        return fallback;
    }
    // Set a parameter by name (clamped to its range). No-op if unknown.
    void setParam(const std::string& key, float v) {
        for (auto& p : params) {
            if (p.name == key) {
                p.value = (v < p.minValue) ? p.minValue : (v > p.maxValue ? p.maxValue : v);
                return;
            }
        }
    }

protected:
    // Register an adjustable parameter (call from a scene constructor).
    void addParam(const std::string& n, float value, float minV, float maxV) {
        params.push_back({n, value, minV, maxV});
    }
public:

    // Build the scene from scratch. Implementations should clear() then fill
    // the `bodies`/constraint description vectors. Called by reset().
    virtual void load() = 0;

    // Per-frame hook for scripted/animated scenes. Default: nothing (physics
    // runs in the solver, not here). dt is the fixed physics timestep.
    virtual void update(float /*dt*/) {}

    // Deterministically recreate the scene. Because load() rebuilds everything
    // from fixed constants (no RNG, no time dependence), reset == reload and is
    // fully reproducible.
    virtual void reset() { clear(); load(); }

    // --- Per-scene environment (applied to the solver on load) ---
    bool      aerodynamicsEnabled = false;
    float     airDensity          = 1.225f;
    glm::vec3 windVelocity        = glm::vec3(0.0f);
    bool      gravityEnabled      = true;

    // --- Owned world contents ---
    std::vector<RigidBody> bodies;
    std::vector<SpringDesc> springs;
    std::vector<HingeDesc>  hinges;
    std::vector<RopeDesc>   ropes;
    std::vector<PulleyDesc> pulleys;
    std::vector<PhysicsSolver::StaticPlane> planes;

protected:
    // Wipe all owned contents so load() starts from a clean slate.
    void clear() {
        bodies.clear();
        springs.clear();
        hinges.clear();
        ropes.clear();
        pulleys.clear();
        planes.clear();
        aerodynamicsEnabled = false;
        airDensity = 1.225f;
        windVelocity = glm::vec3(0.0f);
        gravityEnabled = true;
    }
};
