#pragma once
// ===========================================================================
// SceneManager — a lightweight registry + switcher for Scenes.
//
// Responsibilities:
//   * registerScene(name, Scene*)  — add a named scene (non-owning pointer).
//   * loadScene(name)              — make a scene active: (re)build it and push
//                                    its contents into the solver.
//   * restartCurrentScene()        — deterministically reload the active scene.
//   * nextScene() / previousScene()— cycle through registered scenes in order.
//   * bodies()                     — the live RigidBody vector the solver steps
//                                    and the renderer draws (owned by the scene).
//
// The manager does NOT modify the physics solver's implementation. It only
// clears and repopulates the solver's public constraint/plane vectors and
// environment flags, and re-points each constraint at the active scene's own
// (stable) body storage. This keeps constraint RigidBody* pointers valid.
// ===========================================================================

#include <string>
#include <vector>
#include <utility>
#include <iostream>

#include "Scene.h"
#include "../physics/physicssolver.h"
#include "../physics/rigidbody.h"

class SceneManager {
public:
    explicit SceneManager(PhysicsSolver& solver) : solver_(solver) {}

    // Register a scene under a name. The manager does not take ownership;
    // callers keep the Scene objects alive (e.g. as locals in main()).
    void registerScene(const std::string& name, Scene* scene) {
        scenes_.push_back({name, scene});
    }

    // Load a scene by name. Returns false if the name is unknown.
    bool loadScene(const std::string& name) {
        for (std::size_t i = 0; i < scenes_.size(); ++i) {
            if (scenes_[i].first == name) { activate(static_cast<int>(i)); return true; }
        }
        std::cerr << "[SceneManager] unknown scene: " << name << "\n";
        return false;
    }

    // Load a scene by registration index (used by number keys). No-op if out of range.
    bool loadSceneIndex(int index) {
        if (index < 0 || index >= static_cast<int>(scenes_.size())) return false;
        activate(index);
        return true;
    }

    // Deterministically rebuild and reload the active scene.
    void restartCurrentScene() {
        if (activeIndex_ >= 0) activate(activeIndex_);
    }

    void nextScene() {
        if (scenes_.empty()) return;
        activate((activeIndex_ + 1) % static_cast<int>(scenes_.size()));
    }

    void previousScene() {
        if (scenes_.empty()) return;
        const int n = static_cast<int>(scenes_.size());
        activate((activeIndex_ - 1 + n) % n);
    }

    // Per-frame passthrough for scripted scenes.
    void update(float dt) {
        if (active_) active_->update(dt);
    }

    // The live simulation state: the active scene's own bodies vector. The main
    // loop steps and renders this. Returns an empty static vector if no scene.
    std::vector<RigidBody>& bodies() {
        static std::vector<RigidBody> empty;
        return active_ ? active_->bodies : empty;
    }

    Scene*      activeScene() const { return active_; }
    int         activeIndex() const { return activeIndex_; }
    std::size_t count()       const { return scenes_.size(); }
    const std::string& activeName() const {
        static const std::string none = "(none)";
        return (activeIndex_ >= 0) ? scenes_[activeIndex_].first : none;
    }

private:
    // Build the scene fresh and mirror its contents into the solver.
    void activate(int index) {
        activeIndex_ = index;
        active_ = scenes_[index].second;
        active_->reset();      // deterministic (re)build from constants
        applyToSolver();
        std::cout << "[SceneManager] loaded scene " << (index + 1) << ": "
                  << scenes_[index].first << "  (" << active_->bodies.size() << " bodies)\n";
    }

    // Copy the active scene's constraint descriptions into the solver, resolving
    // body indices to addresses in the scene's OWN body storage (stable for the
    // lifetime of the loaded scene, since load() reserves and never reallocates
    // afterwards). Clears any previous scene's constraints/planes first.
    void applyToSolver() {
        solver_.springs.clear();
        solver_.hinges.clear();
        solver_.ropes.clear();
        solver_.pulleys.clear();
        solver_.planes.clear();

        std::vector<RigidBody>& bs = active_->bodies;
        auto body = [&](int i) -> RigidBody* {
            return (i >= 0 && i < static_cast<int>(bs.size())) ? &bs[i] : nullptr;
        };

        for (const auto& d : active_->springs) {
            SpringConstraint c;
            c.bodyA = body(d.bodyA); c.bodyB = body(d.bodyB);
            c.localAnchorA = d.localAnchorA; c.localAnchorB = d.localAnchorB;
            c.restLength = d.restLength; c.stiffness = d.stiffness; c.damping = d.damping;
            solver_.springs.push_back(c);
        }
        for (const auto& d : active_->hinges) {
            HingeConstraint c;
            c.bodyA = body(d.bodyA); c.bodyB = body(d.bodyB);
            c.localAnchorA = d.localAnchorA; c.localAnchorB = d.localAnchorB;
            c.localAxisA = d.localAxisA; c.localAxisB = d.localAxisB;
            solver_.hinges.push_back(c);
        }
        for (const auto& d : active_->ropes) {
            RopeConstraint c;
            c.bodyA = body(d.bodyA); c.bodyB = body(d.bodyB);
            c.localAnchorA = d.localAnchorA; c.localAnchorB = d.localAnchorB;
            c.maxLength = d.maxLength;
            solver_.ropes.push_back(c);
        }
        for (const auto& d : active_->pulleys) {
            PulleyConstraint c;
            c.bodyA = body(d.bodyA); c.bodyB = body(d.bodyB);
            c.localAnchorA = d.localAnchorA; c.localAnchorB = d.localAnchorB;
            c.pulleyPos = d.pulleyPos; c.totalRopeLength = d.totalRopeLength;
            c.pulleyRadius = d.pulleyRadius;
            solver_.pulleys.push_back(c);
        }
        for (const auto& pl : active_->planes) solver_.planes.push_back(pl);

        // Per-scene environment.
        solver_.aerodynamicsEnabled = active_->aerodynamicsEnabled;
        solver_.airDensity          = active_->airDensity;
        solver_.windVelocity        = active_->windVelocity;
        solver_.gravityEnabled      = active_->gravityEnabled;
    }

    PhysicsSolver& solver_;
    std::vector<std::pair<std::string, Scene*>> scenes_;
    Scene* active_ = nullptr;
    int    activeIndex_ = -1;
};
