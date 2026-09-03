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
