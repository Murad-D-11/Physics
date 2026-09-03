#pragma once
// ===========================================================================
// Concrete scenes for the Scene Manager.
//
//   1. DominoSpiralScene  — an Archimedean spiral of dominoes, tipped inward.
//   2. AtwoodMachineScene — two masses over an ideal pulley.
//   3. RopeBridgeScene    — plank walkway suspended by rope segments.
//   4. SpringPendulumScene— a bob on a spring hung from a fixed anchor.
//   5. EmptySandboxScene  — bare ground; a clean slate to drop bodies into.
//
// Every scene is fully deterministic: load() builds from fixed constants only
// (no RNG, no wall-clock), so reset()/restart reproduces it exactly.
// ===========================================================================

#include <cmath>
#include <vector>
#include <glm/gtc/quaternion.hpp>
#include "Scene.h"

// ---------------------------------------------------------------------------
// 1. Domino Spiral
// ---------------------------------------------------------------------------
class DominoSpiralScene : public Scene {
public:
    const char* name() const override { return "Domino Spiral"; }

    void load() override {
        const int count = 150;
        const glm::vec3 dominoScale(0.15f, 0.9f, 0.45f); // thin, tall, wide
        const float halfHeight = dominoScale.y * 0.5f;
        const float halfThick  = dominoScale.x * 0.5f;

        const float spacing = 0.45f; // center-to-center arc length
        const float r0 = 1.5f;       // inner radius
        const float b  = 0.18f;      // radial growth per radian
        const glm::vec3 up(0.0f, 1.0f, 0.0f);

        bodies.reserve(count);

        // Walk the spiral at constant arc-length steps.
        std::vector<glm::vec3> pos;
        pos.reserve(count);
        float theta = 0.0f;
        for (int i = 0; i < count; ++i) {
            const float r = r0 + b * theta;
            pos.push_back(glm::vec3(r * std::cos(theta), halfHeight, r * std::sin(theta)));
            theta += spacing / std::sqrt(r * r + b * b);
        }

        for (int i = 0; i < count; ++i) {
            glm::vec3 tangent = (i < count - 1) ? (pos[i + 1] - pos[i]) : (pos[i] - pos[i - 1]);
            tangent.y = 0.0f;
            tangent = glm::normalize(tangent);

            RigidBody domino;
            domino.scale = dominoScale;
            domino.position = pos[i];
            domino.velocity = glm::vec3(0.0f);
            domino.orientation = glm::angleAxis(std::atan2(-tangent.z, tangent.x), up);
            sceneSetCubeMass(domino, 0.5f);
            domino.restitution = 0.03f;
            domino.friction = 0.5f;
            bodies.push_back(domino);
        }

        // Flick the first domino past its balance angle to start the cascade.
        const glm::vec3 tangent0 = glm::normalize(pos[1] - pos[0]);
        const glm::vec3 tiltAxis = glm::normalize(glm::cross(up, tangent0));
        const float tiltAngle = glm::radians(14.0f);
        bodies[0].orientation = glm::angleAxis(tiltAngle, tiltAxis) * bodies[0].orientation;
        bodies[0].position.y = halfHeight * std::cos(tiltAngle) + halfThick * std::sin(tiltAngle);
        bodies[0].angularVelocity = tiltAxis * 3.0f;
    }
};

// ---------------------------------------------------------------------------
// 2. Atwood Machine
// ---------------------------------------------------------------------------
class AtwoodMachineScene : public Scene {
public:
    const char* name() const override { return "Atwood Machine"; }

    void load() override {
        const glm::vec3 pulleyPos(0.0f, 8.0f, 0.0f);
        const float dropY = 5.0f;   // both masses start 3 m below the pulley
        const float offX  = 0.8f;

        bodies.reserve(2);

        RigidBody heavy;                                  // index 0
        heavy.position = glm::vec3(-offX, dropY, 0.0f);
        sceneSetSphereMass(heavy, 2.0f, 0.35f);
        heavy.restitution = 0.1f; heavy.friction = 0.3f;
        bodies.push_back(heavy);

        RigidBody light;                                  // index 1
        light.position = glm::vec3(offX, dropY, 0.0f);
        sceneSetSphereMass(light, 1.0f, 0.30f);
        light.restitution = 0.1f; light.friction = 0.3f;
        bodies.push_back(light);

        PulleyDesc p;
        p.bodyA = 0; p.bodyB = 1;
        p.pulleyPos = pulleyPos;
        // Total rope length = current dist(pulley,A) + dist(pulley,B).
        p.totalRopeLength = glm::length(bodies[0].position - pulleyPos)
                          + glm::length(bodies[1].position - pulleyPos);
        p.pulleyRadius = 0.25f;
        pulleys.push_back(p);
    }
};

// ---------------------------------------------------------------------------
// 3. Rope Bridge
//
// A horizontal walkway of plank cubes suspended between two fixed towers by
// rope segments (plank<->plank and tower<->end-plank). Ropes are one-sided
// distance constraints, so the bridge sags under gravity and swings when
// something lands on it, but never compresses.
// ---------------------------------------------------------------------------
class RopeBridgeScene : public Scene {
public:
    const char* name() const override { return "Rope Bridge"; }

    void load() override {
        const int planks = 10;
        const float plankGap = 1.0f;         // horizontal spacing
        const float y = 5.0f;                // suspension height
        const float span = (planks - 1) * plankGap;
        const float x0 = -span * 0.5f;
        const glm::vec3 plankScale(0.8f, 0.15f, 1.6f);

        bodies.reserve(planks + 3);

        // Two fixed anchor towers (static boxes) at the ends.
        RigidBody towerL;                     // index 0 (static)
        towerL.scale = glm::vec3(0.6f, 3.0f, 2.0f);
        towerL.position = glm::vec3(x0 - plankGap, y - 1.0f, 0.0f);
        sceneSetCubeMass(towerL, 1.0f);
        sceneMakeStatic(towerL);
        towerL.friction = 0.6f;
        bodies.push_back(towerL);

        RigidBody towerR;                     // index 1 (static)
        towerR.scale = glm::vec3(0.6f, 3.0f, 2.0f);
        towerR.position = glm::vec3(x0 + span + plankGap, y - 1.0f, 0.0f);
        sceneSetCubeMass(towerR, 1.0f);
        sceneMakeStatic(towerR);
        towerR.friction = 0.6f;
        bodies.push_back(towerR);

        // Plank deck (indices 2 .. 2+planks-1).
        const int firstPlank = 2;
        for (int i = 0; i < planks; ++i) {
            RigidBody plank;
            plank.scale = plankScale;
            plank.position = glm::vec3(x0 + i * plankGap, y, 0.0f);
            sceneSetCubeMass(plank, 0.5f);
            plank.restitution = 0.05f;
            plank.friction = 0.7f;
            bodies.push_back(plank);
        }

        // Rope: left tower top -> first plank.
        const glm::vec3 towerTopL(0.0f, 1.5f, 0.0f); // local top of tower box
        const glm::vec3 towerTopR(0.0f, 1.5f, 0.0f);
        {
            RopeDesc r;
            r.bodyA = 0; r.localAnchorA = glm::vec3(0.25f, 1.5f, 0.0f);
            r.bodyB = firstPlank; r.localAnchorB = glm::vec3(-plankScale.x * 0.5f, 0.0f, 0.0f);
            r.maxLength = plankGap * 1.1f;
            ropes.push_back(r);
        }
        // Plank <-> plank ropes.
        for (int i = 0; i < planks - 1; ++i) {
            RopeDesc r;
            r.bodyA = firstPlank + i;     r.localAnchorA = glm::vec3(plankScale.x * 0.5f, 0.0f, 0.0f);
            r.bodyB = firstPlank + i + 1; r.localAnchorB = glm::vec3(-plankScale.x * 0.5f, 0.0f, 0.0f);
            r.maxLength = plankGap * 1.05f;
            ropes.push_back(r);
        }
        // Rope: last plank -> right tower top.
        {
            RopeDesc r;
            r.bodyA = firstPlank + planks - 1; r.localAnchorA = glm::vec3(plankScale.x * 0.5f, 0.0f, 0.0f);
            r.bodyB = 1; r.localAnchorB = glm::vec3(-0.25f, 1.5f, 0.0f);
            r.maxLength = plankGap * 1.1f;
            ropes.push_back(r);
        }

        // A ball dropped onto the middle of the deck to make it swing.
        RigidBody ball;
        ball.position = glm::vec3(x0 + span * 0.5f, y + 4.0f, 0.0f);
        sceneSetSphereMass(ball, 2.0f, 0.5f);
        ball.restitution = 0.2f; ball.friction = 0.5f;
        bodies.push_back(ball);

        (void)towerTopL; (void)towerTopR;
    }
};

// ---------------------------------------------------------------------------
// 4. Spring Pendulum
//
// A bob hangs from a fixed world anchor by a damped spring. Released off-axis
// it both swings (pendulum) and bobs up/down (spring) — a coupled oscillator.
// ---------------------------------------------------------------------------
class SpringPendulumScene : public Scene {
public:
    const char* name() const override { return "Spring Pendulum"; }

    void load() override {
        const glm::vec3 anchor(0.0f, 8.0f, 0.0f);
        bodies.reserve(2);

        // Primary bob, released to the side so it swings + oscillates.
        RigidBody bob;                                   // index 0
        bob.position = glm::vec3(3.0f, 5.0f, 0.0f);
        sceneSetSphereMass(bob, 1.5f, 0.4f);
        bob.restitution = 0.3f; bob.friction = 0.4f;
        bodies.push_back(bob);

        SpringDesc s;
        s.bodyA = -1; s.localAnchorA = anchor;   // fixed world anchor
        s.bodyB = 0;  s.localAnchorB = glm::vec3(0.0f);
        s.restLength = 3.0f;
        s.stiffness  = 120.0f;
        s.damping    = 2.0f;
        springs.push_back(s);

        // A second bob chained below the first via another spring, for a
        // visibly richer (double-spring) motion.
        RigidBody bob2;                                  // index 1
        bob2.position = glm::vec3(3.0f, 2.0f, 0.0f);
        sceneSetSphereMass(bob2, 1.0f, 0.3f);
        bob2.restitution = 0.3f; bob2.friction = 0.4f;
        bodies.push_back(bob2);

        SpringDesc s2;
        s2.bodyA = 0; s2.localAnchorA = glm::vec3(0.0f);
        s2.bodyB = 1; s2.localAnchorB = glm::vec3(0.0f);
        s2.restLength = 2.0f;
        s2.stiffness  = 90.0f;
        s2.damping    = 1.5f;
        springs.push_back(s2);
    }
};

// ---------------------------------------------------------------------------
// 5. Empty Sandbox
//
// Just the ground plane the solver already provides — a blank world. A single
// resting cube marks the origin so the scene isn't visually empty; delete it
// for a truly bare slate.
// ---------------------------------------------------------------------------
class EmptySandboxScene : public Scene {
public:
    const char* name() const override { return "Empty Sandbox"; }

    void load() override {
        bodies.reserve(1);
        RigidBody marker;
        marker.scale = glm::vec3(1.0f);
        marker.position = glm::vec3(0.0f, 0.5f, 0.0f);
        sceneSetCubeMass(marker, 1.0f);
        marker.restitution = 0.2f;
        marker.friction = 0.6f;
        bodies.push_back(marker);
    }
};

// ---------------------------------------------------------------------------
// 6. Sandbox
//
// An interactive, AI-ready playground. It starts from a fixed, deterministic
// layout (one cube + one sphere resting on the ground) and exposes runtime
// mutators so the app can spawn / delete bodies from mouse + keyboard input:
//
//   spawnCube(pos)   — drop a unit cube at pos
//   spawnSphere(pos) — drop a sphere at pos
//   deleteBody(i)    — remove the body at index i
//
// The scene has NO constraints, so appending / erasing bodies at runtime never
// invalidates solver constraint pointers (there are none to invalidate). The
// app is responsible for clearing the SimulationRecorder on any structural
// change so recorded object ids stay consistent.
// ---------------------------------------------------------------------------
class SandboxScene : public Scene {
public:
    const char* name() const override { return "Sandbox"; }

    void load() override {
        // Reserve generously so early spawns don't force a reallocation. There
        // are no constraints referencing these bodies, so even a reallocation
        // would be safe; reserving simply avoids churn.
        bodies.reserve(64);

        RigidBody cube;
        cube.scale = glm::vec3(1.0f);
        cube.position = glm::vec3(-1.5f, 0.5f, 0.0f);
        sceneSetCubeMass(cube, 1.0f);
        cube.restitution = 0.2f;
        cube.friction = 0.6f;
        bodies.push_back(cube);

        RigidBody ball;
        ball.position = glm::vec3(1.5f, 0.5f, 0.0f);
        sceneSetSphereMass(ball, 1.0f, 0.5f);
        ball.restitution = 0.4f;
        ball.friction = 0.5f;
        bodies.push_back(ball);
    }

    // --- Runtime mutators (called from the app) ----------------------------

    // Drop a unit cube at world position `pos`. Returns the new body's index.
    int spawnCube(const glm::vec3& pos) {
        RigidBody cube;
        cube.scale = glm::vec3(1.0f);
        cube.position = pos;
        sceneSetCubeMass(cube, 1.0f);
        cube.restitution = 0.2f;
        cube.friction = 0.6f;
        bodies.push_back(cube);
        return static_cast<int>(bodies.size()) - 1;
    }

    // Drop a sphere at world position `pos`. Returns the new body's index.
    int spawnSphere(const glm::vec3& pos, float radius = 0.5f) {
        RigidBody ball;
        ball.position = pos;
        sceneSetSphereMass(ball, 1.0f, radius);
        ball.restitution = 0.4f;
        ball.friction = 0.5f;
        bodies.push_back(ball);
        return static_cast<int>(bodies.size()) - 1;
    }

    // Remove the body at `index`. Returns true if a body was removed.
    bool deleteBody(int index) {
        if (index < 0 || index >= static_cast<int>(bodies.size())) return false;
        bodies.erase(bodies.begin() + index);
        return true;
    }
};
