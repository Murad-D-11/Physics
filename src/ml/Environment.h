#pragma once
// ===========================================================================
// Environment — a headless, rendering-independent wrapper around the physics
// solver, shaped for machine-learning use.
//
// It owns a set of rigid bodies and a PhysicsSolver and exposes a minimal,
// gym-style API:
//
//   reset()                 — restore the world to its initial state
//   step(dt)                — advance the simulation by one fixed step
//   getObservation()        — read every body as structured numerical data
//   applyAction(id, impulse)— apply a linear impulse to one body
//
// There is NO dependency on GLFW, OpenGL, or the renderer here: the whole
// class compiles against the physics translation units alone, so the same code
// runs in a training loop or an offline dataset generator.
//
// The Observation is a flat, value-typed struct (no pointers into the solver)
// so it is safe to copy, store in a buffer, or serialise.
// ===========================================================================

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../physics/rigidbody.h"
#include "../physics/physicssolver.h"

// One rigid body exposed as structured numerical data for ML.
struct Observation {
    int       id;                 // body index within the environment
    float     mass;               // kg (0 => static/infinite mass)
    int       shape;              // 0 = box, 1 = sphere
    int       material;           // MaterialType as int (-1 = unassigned)
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 angularVelocity;
    glm::quat orientation;        // (w, x, y, z)
    bool      sleeping;
};

class Environment {
public:
    Environment() { solver_.gravityEnabled = true; }

    // --- World construction (called by the dataset generator / a user) ------

    // Replace the world with `bodies` and remember them as the reset state.
    void setBodies(const std::vector<RigidBody>& bodies) {
        initial_ = bodies;
        bodies_  = bodies;
        time_    = 0.0f;
    }

    // Direct access for building a scene in place before the first reset().
    std::vector<RigidBody>& bodies() { return bodies_; }
    const std::vector<RigidBody>& bodies() const { return bodies_; }
    PhysicsSolver& solver() { return solver_; }

    // Snapshot the CURRENT bodies as the state reset() will restore to.
    void commitAsInitialState() { initial_ = bodies_; time_ = 0.0f; }

    std::size_t size() const { return bodies_.size(); }
    float time() const { return time_; }

    // --- Gym-style API ------------------------------------------------------

    // Restore the world to the committed initial state (deterministic).
    void reset() {
        bodies_ = initial_;
        time_   = 0.0f;
    }

    // Advance the simulation by one fixed timestep.
    void step(float dt) {
        solver_.step(bodies_, dt);
        time_ += dt;
    }

    // Apply a linear impulse J (kg*m/s) to body `id`: dv = J / m. Static bodies
    // (inverseMass == 0) are unaffected. Wakes the body so the impulse lands.
    void applyAction(int id, const glm::vec3& impulse) {
        if (id < 0 || id >= static_cast<int>(bodies_.size())) return;
        RigidBody& b = bodies_[id];
        if (b.inverseMass <= 0.0f) return;
        b.velocity += impulse * b.inverseMass;
        b.asleep = false;
        b.sleepTimer = 0.0f;
    }

    // Read one body as structured data.
    Observation getObservation(int id) const {
        const RigidBody& b = bodies_[id];
        Observation o;
        o.id              = id;
        o.mass            = b.mass;
        o.shape           = static_cast<int>(b.shape);
        o.material        = b.materialType;
        o.position        = b.position;
        o.velocity        = b.velocity;
        o.angularVelocity = b.angularVelocity;
        o.orientation     = b.orientation;
        o.sleeping        = b.asleep;
        return o;
    }

    // Read the whole world as a vector of observations.
    std::vector<Observation> getObservation() const {
        std::vector<Observation> obs;
        obs.reserve(bodies_.size());
        for (int i = 0; i < static_cast<int>(bodies_.size()); ++i) {
            obs.push_back(getObservation(i));
        }
        return obs;
    }

private:
    PhysicsSolver          solver_;
    std::vector<RigidBody> bodies_;   // live state
    std::vector<RigidBody> initial_;  // state reset() restores to
    float                  time_ = 0.0f;
};
