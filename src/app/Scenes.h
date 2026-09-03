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
    const char* description() const override { return "An Archimedean spiral of dominoes toppling in a cascade."; }
    const char* principle() const override { return "Sequential energy transfer; gravitational potential release."; }

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
    const char* description() const override { return "Two masses over an ideal pulley."; }
    const char* principle() const override { return "Net force = (m1-m2)g; shared acceleration via the rope."; }

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
    const char* description() const override { return "A plank walkway suspended by rope segments between towers."; }
    const char* principle() const override { return "One-sided distance constraints; sag under load."; }

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
    const char* description() const override { return "A bob on a damped spring hung from a fixed anchor."; }
    const char* principle() const override { return "Coupled oscillation: swing (pendulum) + bob (Hooke's law)."; }

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
    const char* description() const override { return "A bare ground plane with a single marker cube."; }
    const char* principle() const override { return "A clean slate for free experimentation."; }

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
    const char* description() const override { return "Interactive playground: spawn, drag, and fling bodies."; }
    const char* principle() const override { return "Freeform rigid-body dynamics with live editing."; }

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

// ---------------------------------------------------------------------------
// 7. Newton's Cradle
//
// A row of equal spheres suspended as pendulums (rope to a fixed anchor). The
// end sphere is pulled aside; on impact momentum + energy transfer through the
// line and kick the far sphere out. Demonstrates conservation of momentum and
// kinetic energy in elastic collisions.
// ---------------------------------------------------------------------------
class NewtonsCradleScene : public Scene {
public:
    NewtonsCradleScene() {
        addParam("balls", 5.0f, 3.0f, 7.0f);
        addParam("pullBack", 2.0f, 0.0f, 3.0f); // how many end balls start raised
    }
    const char* name() const override { return "Newton's Cradle"; }
    const char* description() const override { return "A row of suspended balls exchanging momentum on impact."; }
    const char* principle() const override { return "Conservation of momentum and kinetic energy (elastic collision)."; }

    void load() override {
        const int balls = static_cast<int>(param("balls", 5.0f));
        const int pull  = static_cast<int>(param("pullBack", 2.0f));
        const float r = 0.5f;
        const float y = 6.0f;             // anchor height
        const float len = 3.0f;           // pendulum length
        const float spacing = 2.0f * r;   // touching at rest

        bodies.reserve(balls);
        const float x0 = -(balls - 1) * spacing * 0.5f;
        for (int i = 0; i < balls; ++i) {
            RigidBody ball;
            const float restX = x0 + i * spacing;
            ball.position = glm::vec3(restX, y - len, 0.0f);
            sceneSetSphereMass(ball, 1.0f, r);
            ball.restitution = 0.98f;  // near-elastic so the click transmits
            ball.friction = 0.0f;
            bodies.push_back(ball);
        }
        // Raise the leftmost `pull` balls to a starting angle.
        for (int i = 0; i < pull && i < balls; ++i) {
            const float angle = glm::radians(50.0f);
            const float ax = x0 + i * spacing;
            bodies[i].position = glm::vec3(ax - len * std::sin(angle), y - len * std::cos(angle), 0.0f);
        }
        // Rope from anchor (world) straight down to each ball.
        for (int i = 0; i < balls; ++i) {
            RopeDesc rope;
            rope.bodyA = -1; rope.localAnchorA = glm::vec3(x0 + i * spacing, y, 0.0f);
            rope.bodyB = i;  rope.localAnchorB = glm::vec3(0.0f);
            rope.maxLength = len;
            ropes.push_back(rope);
        }
    }
};

// ---------------------------------------------------------------------------
// 8. Inclined Plane
//
// A block released on a static ramp. Whether it slides or stays depends on the
// balance of gravity's downslope component and friction. Demonstrates static
// vs kinetic friction on a slope.
// ---------------------------------------------------------------------------
class InclinedPlaneScene : public Scene {
public:
    InclinedPlaneScene() {
        addParam("angleDeg", 25.0f, 5.0f, 60.0f);
        addParam("friction", 0.30f, 0.0f, 1.0f);
    }
    const char* name() const override { return "Inclined Plane"; }
    const char* description() const override { return "A block on an adjustable ramp; slides or grips by friction."; }
    const char* principle() const override { return "Gravity's downslope component vs. Coulomb friction."; }

    void load() override {
        const float angle = glm::radians(param("angleDeg", 25.0f));
        const float mu = param("friction", 0.30f);

        // Static ramp plane, tilted about the Z axis.
        PhysicsSolver::StaticPlane ramp;
        ramp.point = glm::vec3(0.0f, 1.0f, 0.0f);
        ramp.normal = glm::normalize(glm::vec3(std::sin(angle), std::cos(angle), 0.0f));
        ramp.friction = mu;
        ramp.restitution = 0.1f;
        ramp.halfExtent = glm::vec2(6.0f, 4.0f);
        planes.push_back(ramp);

        // Block resting on the ramp surface, nudged up-slope a bit.
        RigidBody block;
        block.scale = glm::vec3(1.0f);
        // Place it slightly above the ramp point along the normal.
        block.position = ramp.point + ramp.normal * 0.55f + glm::vec3(-std::cos(angle), 0.0f, 0.0f) * 2.0f;
        // Orient the block to lie flat on the slope.
        block.orientation = glm::angleAxis(-angle, glm::vec3(0.0f, 0.0f, 1.0f));
        sceneSetCubeMass(block, 1.0f);
        block.friction = mu;
        block.restitution = 0.1f;
        bodies.push_back(block);
    }
};

// ---------------------------------------------------------------------------
// 9. Suspension Bridge
//
// A deck of planks hung from two main cables (rope chains) strung between tall
// towers, with vertical hanger ropes to the deck. Loading the deck redistributes
// tension through the cables. Demonstrates tensile load paths in cable structures.
// ---------------------------------------------------------------------------
class SuspensionBridgeScene : public Scene {
public:
    SuspensionBridgeScene() { addParam("planks", 12.0f, 6.0f, 16.0f); }
    const char* name() const override { return "Suspension Bridge"; }
    const char* description() const override { return "A plank deck hung from cables between two towers."; }
    const char* principle() const override { return "Tensile load distribution through suspension cables."; }

    void load() override {
        const int planks = static_cast<int>(param("planks", 12.0f));
        const float gap = 1.0f;
        const float deckY = 4.0f;
        const float span = (planks - 1) * gap;
        const float x0 = -span * 0.5f;
        const glm::vec3 plankScale(0.8f, 0.15f, 2.0f);

        bodies.reserve(planks + 2 + (planks + 1));

        // Towers (static) at each end, taller than the deck.
        RigidBody towerL; towerL.scale = glm::vec3(0.6f, 6.0f, 2.5f);
        towerL.position = glm::vec3(x0 - gap, deckY + 1.0f, 0.0f);
        sceneSetCubeMass(towerL, 1.0f); sceneMakeStatic(towerL); towerL.friction = 0.7f;
        bodies.push_back(towerL);                              // index 0
        RigidBody towerR = towerL;
        towerR.position = glm::vec3(x0 + span + gap, deckY + 1.0f, 0.0f);
        bodies.push_back(towerR);                              // index 1

        const int firstPlank = 2;
        for (int i = 0; i < planks; ++i) {
            RigidBody plank; plank.scale = plankScale;
            plank.position = glm::vec3(x0 + i * gap, deckY, 0.0f);
            sceneSetCubeMass(plank, 0.4f);
            plank.restitution = 0.05f; plank.friction = 0.7f;
            bodies.push_back(plank);
        }

        // Plank-to-plank deck ropes.
        for (int i = 0; i < planks - 1; ++i) {
            RopeDesc r; r.bodyA = firstPlank + i;     r.localAnchorA = glm::vec3(plankScale.x * 0.5f, 0, 0);
            r.bodyB = firstPlank + i + 1; r.localAnchorB = glm::vec3(-plankScale.x * 0.5f, 0, 0);
            r.maxLength = gap * 1.05f; ropes.push_back(r);
        }
        // Tower-to-end-plank ropes (the main cable ends).
        RopeDesc lc; lc.bodyA = 0; lc.localAnchorA = glm::vec3(0, 3.0f, 0);
        lc.bodyB = firstPlank; lc.localAnchorB = glm::vec3(0, 0, 0); lc.maxLength = gap * 1.5f;
        ropes.push_back(lc);
        RopeDesc rc; rc.bodyA = 1; rc.localAnchorA = glm::vec3(0, 3.0f, 0);
        rc.bodyB = firstPlank + planks - 1; rc.localAnchorB = glm::vec3(0, 0, 0); rc.maxLength = gap * 1.5f;
        ropes.push_back(rc);

        // Vertical hanger ropes from the tower tops down to interior planks
        // (approximate suspension geometry with a shallow catenary of hangers).
        for (int i = 1; i < planks - 1; ++i) {
            RopeDesc h;
            const int tower = (i < planks / 2) ? 0 : 1;
            h.bodyA = tower; h.localAnchorA = glm::vec3(0, 3.0f, 0);
            h.bodyB = firstPlank + i; h.localAnchorB = glm::vec3(0, 0, 0);
            h.maxLength = 4.0f + std::abs(i - (planks - 1) * 0.5f) * 0.3f;
            ropes.push_back(h);
        }
    }
};

// ---------------------------------------------------------------------------
// 10. Cantilever Beam
//
// A horizontal chain of box segments hinged end-to-end, anchored to a wall at
// one end and unsupported at the other. Gravity makes it droop and reveals the
// bending load carried by each hinge. Demonstrates cantilever bending / hinge
// load accumulation.
// ---------------------------------------------------------------------------
class CantileverBeamScene : public Scene {
public:
    CantileverBeamScene() { addParam("segments", 8.0f, 3.0f, 12.0f); }
    const char* name() const override { return "Cantilever Beam"; }
    const char* description() const override { return "Hinged box segments fixed at one end, drooping under gravity."; }
    const char* principle() const override { return "Cantilever bending: hinge load grows toward the fixed end."; }

    void load() override {
        const int segs = static_cast<int>(param("segments", 8.0f));
        const float segLen = 1.0f;
        const float y = 6.0f;
        const glm::vec3 scale(segLen, 0.4f, 0.8f);
        const float x0 = -((segs - 1) * segLen) * 0.5f;

        bodies.reserve(segs);
        for (int i = 0; i < segs; ++i) {
            RigidBody s; s.scale = scale;
            s.position = glm::vec3(x0 + i * segLen, y, 0.0f);
            sceneSetCubeMass(s, 0.5f);
            s.friction = 0.5f; s.restitution = 0.05f;
            if (i == 0) sceneMakeStatic(s); // fixed to the wall
            bodies.push_back(s);
        }
        // Hinge each segment to the previous one along the Z axis.
        for (int i = 1; i < segs; ++i) {
            HingeDesc h;
            h.bodyA = i - 1; h.localAnchorA = glm::vec3(segLen * 0.5f, 0, 0);
            h.bodyB = i;     h.localAnchorB = glm::vec3(-segLen * 0.5f, 0, 0);
            h.localAxisA = glm::vec3(0, 0, 1); h.localAxisB = glm::vec3(0, 0, 1);
            hinges.push_back(h);
        }
    }
};

// ---------------------------------------------------------------------------
// 11. Hanging Chain
//
// A chain of small links hung from two fixed points settles into a catenary
// curve under gravity. Demonstrates the catenary: the shape a uniform flexible
// chain takes when supported at its ends.
// ---------------------------------------------------------------------------
class HangingChainScene : public Scene {
public:
    HangingChainScene() { addParam("links", 16.0f, 8.0f, 24.0f); }
    const char* name() const override { return "Hanging Chain"; }
    const char* description() const override { return "A flexible chain hung between two points."; }
    const char* principle() const override { return "The catenary curve of a uniform hanging chain."; }

    void load() override {
        const int links = static_cast<int>(param("links", 16.0f));
        const float linkLen = 0.5f;
        const float y = 7.0f;
        const float spanHalf = links * linkLen * 0.32f; // ends closer than laid-flat -> it sags
        const float x0 = -spanHalf;
        const float x1 =  spanHalf;

        bodies.reserve(links);
        for (int i = 0; i < links; ++i) {
            RigidBody link;
            const float t = static_cast<float>(i) / (links - 1);
            link.position = glm::vec3(x0 + (x1 - x0) * t, y, 0.0f);
            sceneSetSphereMass(link, 0.2f, 0.12f);
            link.friction = 0.4f; link.restitution = 0.05f;
            bodies.push_back(link);
        }
        // Rope between consecutive links (max length = link spacing).
        for (int i = 0; i < links - 1; ++i) {
            RopeDesc r; r.bodyA = i; r.bodyB = i + 1;
            r.localAnchorA = glm::vec3(0.0f); r.localAnchorB = glm::vec3(0.0f);
            r.maxLength = linkLen; ropes.push_back(r);
        }
        // Pin both ends to fixed world anchors.
        RopeDesc a0; a0.bodyA = -1; a0.localAnchorA = glm::vec3(x0, y, 0.0f);
        a0.bodyB = 0; a0.localAnchorB = glm::vec3(0.0f); a0.maxLength = 0.05f; ropes.push_back(a0);
        RopeDesc a1; a1.bodyA = -1; a1.localAnchorA = glm::vec3(x1, y, 0.0f);
        a1.bodyB = links - 1; a1.localAnchorB = glm::vec3(0.0f); a1.maxLength = 0.05f; ropes.push_back(a1);
    }
};

// ---------------------------------------------------------------------------
// 12. Double Pendulum
//
// Two rigid arms hinged in series from a fixed pivot. Its motion is chaotic:
// tiny changes in the start state diverge quickly. Demonstrates sensitive
// dependence on initial conditions in a simple mechanism.
// ---------------------------------------------------------------------------
class DoublePendulumScene : public Scene {
public:
    DoublePendulumScene() { addParam("startAngleDeg", 90.0f, 0.0f, 170.0f); }
    const char* name() const override { return "Double Pendulum"; }
    const char* description() const override { return "Two hinged arms swinging from a fixed pivot."; }
    const char* principle() const override { return "Chaotic dynamics: sensitivity to initial conditions."; }

    void load() override {
        const float startAngle = glm::radians(param("startAngleDeg", 90.0f));
        const float pivotY = 7.0f;
        const float armLen = 2.0f;
        const glm::vec3 armScale(armLen, 0.2f, 0.2f);

        bodies.reserve(2);

        // Arm 1: extends from the pivot outward at the start angle.
        RigidBody arm1; arm1.scale = armScale;
        const glm::vec3 pivot(0.0f, pivotY, 0.0f);
        const glm::vec3 dir1(std::sin(startAngle), -std::cos(startAngle), 0.0f);
        arm1.position = pivot + dir1 * (armLen * 0.5f);
        arm1.orientation = glm::angleAxis(std::atan2(dir1.y, dir1.x), glm::vec3(0, 0, 1));
        sceneSetCubeMass(arm1, 1.0f); arm1.friction = 0.0f; arm1.restitution = 0.0f;
        bodies.push_back(arm1);                                 // index 0

        // Arm 2: hangs straight down from the end of arm 1.
        RigidBody arm2; arm2.scale = armScale;
        const glm::vec3 joint = pivot + dir1 * armLen;
        arm2.position = joint + glm::vec3(0.0f, -armLen * 0.5f, 0.0f);
        arm2.orientation = glm::angleAxis(glm::radians(-90.0f), glm::vec3(0, 0, 1));
        sceneSetCubeMass(arm2, 1.0f); arm2.friction = 0.0f; arm2.restitution = 0.0f;
        bodies.push_back(arm2);                                 // index 1

        // Hinge arm1 to the world pivot, and arm2 to the end of arm1.
        HingeDesc h1; h1.bodyA = -1; h1.localAnchorA = pivot;
        h1.bodyB = 0; h1.localAnchorB = glm::vec3(-armLen * 0.5f, 0, 0);
        h1.localAxisA = glm::vec3(0, 0, 1); h1.localAxisB = glm::vec3(0, 0, 1);
        hinges.push_back(h1);

        HingeDesc h2; h2.bodyA = 0; h2.localAnchorA = glm::vec3(armLen * 0.5f, 0, 0);
        h2.bodyB = 1; h2.localAnchorB = glm::vec3(-armLen * 0.5f, 0, 0);
        h2.localAxisA = glm::vec3(0, 0, 1); h2.localAxisB = glm::vec3(0, 0, 1);
        hinges.push_back(h2);
    }
};

// ---------------------------------------------------------------------------
// 13. Trebuchet
//
// A counterweight arm pivoted on a hinge: a heavy weight on the short arm drops
// and swings the long arm up fast, flinging a projectile resting nearby.
// Demonstrates lever-arm mechanical advantage and energy transfer from a
// falling counterweight.
// ---------------------------------------------------------------------------
class TrebuchetScene : public Scene {
public:
    TrebuchetScene() { addParam("counterweight", 20.0f, 5.0f, 40.0f); }
    const char* name() const override { return "Trebuchet"; }
    const char* description() const override { return "A counterweight arm that flings a projectile."; }
    const char* principle() const override { return "Lever mechanical advantage; gravitational PE to projectile KE."; }

    void load() override {
        const float cwMass = param("counterweight", 20.0f);
        const float pivotY = 4.0f;
        const glm::vec3 pivot(0.0f, pivotY, 0.0f);

        bodies.reserve(4);

        // Support tower (static) holding the pivot.
        RigidBody tower; tower.scale = glm::vec3(0.6f, pivotY, 0.6f);
        tower.position = glm::vec3(0.0f, pivotY * 0.5f, 0.0f);
        sceneSetCubeMass(tower, 1.0f); sceneMakeStatic(tower); tower.friction = 0.8f;
        bodies.push_back(tower);                                // index 0

        // The throwing arm: long, pivoted off-centre so one side is short.
        const float armLen = 6.0f;
        RigidBody arm; arm.scale = glm::vec3(armLen, 0.25f, 0.25f);
        arm.position = glm::vec3(1.0f, pivotY, 0.0f); // shifted so pivot is near the short end
        sceneSetCubeMass(arm, 2.0f); arm.friction = 0.3f; arm.restitution = 0.1f;
        bodies.push_back(arm);                                  // index 1

        // Counterweight hanging off the short arm end.
        RigidBody cw; cw.position = glm::vec3(-1.6f, pivotY - 1.0f, 0.0f);
        sceneSetSphereMass(cw, cwMass, 0.6f); cw.friction = 0.4f; cw.restitution = 0.0f;
        bodies.push_back(cw);                                   // index 2

        // Projectile resting near the long arm tip.
        RigidBody proj; proj.position = glm::vec3(3.9f, pivotY - 1.2f, 0.0f);
        sceneSetSphereMass(proj, 0.5f, 0.3f); proj.friction = 0.4f; proj.restitution = 0.3f;
        bodies.push_back(proj);                                 // index 3

        // Hinge the arm to the tower pivot (Z axis).
        HingeDesc h; h.bodyA = 0; h.localAnchorA = glm::vec3(0.0f, pivotY * 0.5f, 0.0f);
        h.bodyB = 1; h.localAnchorB = glm::vec3(-1.0f, 0.0f, 0.0f);
        h.localAxisA = glm::vec3(0, 0, 1); h.localAxisB = glm::vec3(0, 0, 1);
        hinges.push_back(h);

        // Rope the counterweight to the short arm end.
        RopeDesc cwRope; cwRope.bodyA = 1; cwRope.localAnchorA = glm::vec3(-armLen * 0.5f, 0, 0);
        cwRope.bodyB = 2; cwRope.localAnchorB = glm::vec3(0.0f); cwRope.maxLength = 1.2f;
        ropes.push_back(cwRope);
    }
};
