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
    AtwoodMachineScene() { addParam("massRatio", 2.0f, 1.0f, 4.0f); }
    const char* name() const override { return "Atwood Machine"; }
    const char* description() const override { return "Two masses over an ideal pulley; the heavier one descends."; }
    const char* principle() const override { return "a = (m1-m2)/(m1+m2)*g; the shared rope couples the accelerations."; }

    void load() override {
        // A textbook Atwood machine built as a real apparatus. The pulley wheel
        // has width: the two rope runs leave OPPOSITE edges and drop straight
        // down. Each mass is a BOX riding inside its own VERTICAL GUIDE SHAFT
        // (two static walls), so it can only move vertically and physically
        // cannot drift sideways or cross the other mass -- exactly what the
        // shafts do on a real machine. Ground contact simply removes tension;
        // nothing teleports.
        const float ratio  = param("massRatio", 2.0f);
        const float wheelR = 0.6f;
        const glm::vec3 pulleyPos(0.0f, 12.0f, 0.0f);
        const float leftX  = -wheelR;   // shaft centre under the left wheel edge
        const float rightX =  wheelR;   // shaft centre under the right wheel edge
        const float heavyY = 6.5f;      // heavy starts higher -> room to descend
        const float lightY = 3.0f;      // light starts lower  -> room to rise
        const float massW  = 0.7f;      // mass box width (X)
        const float shaftGap = massW + 0.05f;   // clear channel: mass + small clearance
        const float shaftBottom = 0.6f;
        // Shaft tall enough that the rising mass NEVER leaves the top of its
        // channel: its highest point is lightY + (heavyY - floorRest) < shaftTop.
        const float shaftTop    = pulleyPos.y - 0.4f;

        bodies.reserve(3 + 4 + 2);

        // --- Masses first (indices 0,1) so index-based tests are stable -------
        // NOTE: the two hanging masses are indices 0 (heavy) and 1 (light).
        // The two masses' HORIZONTAL positions are swapped (heavy now under the
        // RIGHT wheel edge, light under the LEFT) versus the earlier layout.
        // The pulley rope is unchanged: its attachments (bodyA=heavy, bodyB=
        // light) and total length depend only on the vertical runs to the wheel
        // edges, not on which column is which, so nothing is detached/reattached.
        RigidBody heavy;                                   // index 0
        heavy.scale = glm::vec3(massW, massW, massW);
        heavy.position = glm::vec3(rightX, heavyY, 0.0f);  // swapped: right column
        sceneSetCubeMass(heavy, ratio);                    // heavier
        heavy.restitution = 0.0f; heavy.friction = 0.02f;  // low friction vs guide walls
        bodies.push_back(heavy);

        RigidBody light;                                   // index 1
        light.scale = glm::vec3(massW, massW, massW);
        light.position = glm::vec3(leftX, lightY, 0.0f);   // swapped: left column
        sceneSetCubeMass(light, 1.0f);                     // lighter
        light.restitution = 0.0f; light.friction = 0.02f;
        bodies.push_back(light);

        PulleyDesc p;
        p.bodyA = 0; p.bodyB = 1;
        p.pulleyPos = pulleyPos;
        p.pulleyRadius = wheelR;
        // Rope length = the two vertical runs to the wheel edges, so it is
        // satisfied at spawn (no start-up jerk).
        p.totalRopeLength = (pulleyPos.y - heavyY) + (pulleyPos.y - lightY);
        pulleys.push_back(p);

        // --- Apparatus (static, rendered) -------------------------------------
        // A vertical guide shaft around each mass column: the walls confine each
        // mass to purely vertical travel and keep the two columns from ever
        // intersecting. Low wall friction so the guiding does not sap the drop.
        sceneAddGuideShaft(bodies, leftX,  shaftBottom, shaftTop, shaftGap, 1.0f, 0.12f, 0.02f);
        sceneAddGuideShaft(bodies, rightX, shaftBottom, shaftTop, shaftGap, 1.0f, 0.12f, 0.02f);
        // Gantry: a top cross-beam carrying the wheel, on a side column.
        sceneAddStaticBox(bodies, glm::vec3(0.0f, pulleyPos.y + 0.35f, 0.0f),
                          glm::vec3(2.0f * wheelR + 1.6f, 0.25f, 0.6f));
        sceneAddStaticBox(bodies, glm::vec3(rightX + 1.6f, pulleyPos.y * 0.5f, 0.0f),
                          glm::vec3(0.3f, pulleyPos.y, 0.6f));
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
        const int   planks   = 11;
        const float plankLen = 1.0f;          // plank length ALONG the span (X)
        const float pitch    = plankLen;      // spacing == length -> planks touch,
                                              // so neighbouring hinge anchors at
                                              // +/-plankLen/2 are COINCIDENT. (The
                                              // old deck spaced planks wider than
                                              // they were long, so every hinge was
                                              // born stretched and fought the deck.)
        const float deckY    = 5.0f;          // rest height of the deck
        const float anchorY  = 8.0f;          // fixed hanger anchors above the deck
        const float span     = (planks - 1) * pitch;
        const float x0       = -span * 0.5f;
        const glm::vec3 plankScale(plankLen, 0.5f, 2.4f); // thick, wide walkway
                                                          // (thick so a resting
                                                          // sphere can't tunnel
                                                          // through the deck)
        const float hangerLen = anchorY - deckY;

        bodies.reserve(planks + 1);

        // Deck planks (indices 0..planks-1), laid end-to-end so they touch.
        for (int i = 0; i < planks; ++i) {
            RigidBody plank;
            plank.scale = plankScale;
            plank.position = glm::vec3(x0 + i * pitch, deckY, 0.0f);
            sceneSetCubeMass(plank, 0.8f);
            plank.restitution = 0.0f;
            plank.friction = 0.8f;
            bodies.push_back(plank);
        }

        // Vertical hanger ropes: a fixed world anchor above each plank holds it
        // at deckY. One-sided, so extra load lets the deck dip a little.
        for (int i = 0; i < planks; ++i) {
            RopeDesc h;
            h.bodyA = -1; h.localAnchorA = glm::vec3(x0 + i * pitch, anchorY, 0.0f);
            h.bodyB = i;  h.localAnchorB = glm::vec3(0.0f);
            h.maxLength = hangerLen;
            ropes.push_back(h);
        }

        // Deck hinges: revolute (Z) joints at the shared, coincident plank
        // edges. The deck now folds only along its length and cannot twist or
        // spin as a body.
        for (int i = 0; i < planks - 1; ++i) {
            HingeDesc h;
            h.bodyA = i;     h.localAnchorA = glm::vec3(plankLen * 0.5f, 0.0f, 0.0f);
            h.bodyB = i + 1; h.localAnchorB = glm::vec3(-plankLen * 0.5f, 0.0f, 0.0f);
            h.localAxisA = glm::vec3(0, 0, 1); h.localAxisB = glm::vec3(0, 0, 1);
            hinges.push_back(h);
        }

        // Pin the two END planks to fixed world abutments at deck height with a
        // taut short rope. This is what a real bridge's abutments do: they hold
        // the deck ends in place so the span cannot swing sideways or rotate as
        // a rigid body -- it can only sag between fixed ends.
        RopeDesc la; la.bodyA = -1; la.localAnchorA = glm::vec3(x0 - plankLen * 0.5f, deckY, 0.0f);
        la.bodyB = 0; la.localAnchorB = glm::vec3(-plankLen * 0.5f, 0.0f, 0.0f); la.maxLength = 0.04f;
        ropes.push_back(la);
        RopeDesc ra; ra.bodyA = -1; ra.localAnchorA = glm::vec3(x0 + span + plankLen * 0.5f, deckY, 0.0f);
        ra.bodyB = planks - 1; ra.localAnchorB = glm::vec3(plankLen * 0.5f, 0.0f, 0.0f); ra.maxLength = 0.04f;
        ropes.push_back(ra);

        // A ball resting on the MIDDLE plank. Placed exactly at that plank's x
        // (so it can't start in a between-plank gap) and just touching the top
        // surface (so it can't tunnel through the thin deck). It settles into
        // the deck and the walkway visibly takes the load.
        // A crate resting on the middle of the deck. A BOX load (not a sphere)
        // is used deliberately: box-on-box resting contact is the engine's most
        // stable manifold (flat face-face, two-point support), so the crate
        // sits solidly on the thin articulated deck instead of a sphere's
        // single point-contact slipping through. It spans two planks so its
        // weight is shared and the deck visibly dips under the load.
        const int   midPlank = planks / 2;
        const float midX     = x0 + midPlank * pitch;
        RigidBody crate;
        crate.scale = glm::vec3(1.4f, 0.8f, 1.4f);
        crate.position = glm::vec3(midX, deckY + plankScale.y * 0.5f + 0.4f, 0.0f);
        sceneSetCubeMass(crate, 0.5f);
        crate.restitution = 0.0f; crate.friction = 0.9f;
        bodies.push_back(crate);                 // index planks
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
    const char* name() const override { return "Spring Laboratory"; }
    const char* description() const override { return "A vertical oscillator, a horizontal oscillator, and a coupled two-mass pair."; }
    const char* principle() const override { return "Hooke's law simple harmonic motion; normal modes of coupled springs."; }

    void load() override {
        // Three simple-harmonic apparatuses side by side (offset in Z), each
        // constrained by geometry so it oscillates cleanly along ONE axis about
        // its equilibrium. Dynamic masses come first (index order below).
        bodies.reserve(12);

        // ---- 1. Vertical oscillator (index 0): mass on a vertical spring from
        //         a fixed overhead beam, riding in a vertical guide shaft so it
        //         bobs purely in Y. Started above equilibrium so it oscillates.
        const float zV = -3.0f;
        const glm::vec3 topV(-3.0f, 9.0f, zV);
        RigidBody vMass; vMass.scale = glm::vec3(0.7f);
        vMass.position = glm::vec3(-3.0f, 6.5f, zV);      // above rest -> SHM in Y
        sceneSetCubeMass(vMass, 1.0f); vMass.friction = 0.02f; vMass.restitution = 0.0f;
        bodies.push_back(vMass);                          // 0
        {
            SpringDesc s; s.bodyA = -1; s.localAnchorA = topV;
            s.bodyB = 0; s.localAnchorB = glm::vec3(0.0f);
            s.restLength = 3.5f; s.stiffness = 80.0f; s.damping = 0.15f;
            springs.push_back(s);
        }

        // ---- 2. Horizontal oscillator (index 1): mass on a horizontal spring
        //         from a fixed wall, riding in a horizontal channel so it moves
        //         purely in X. Started displaced along +X -> SHM in X.
        const float zH = 0.0f;
        const float railY = 4.0f;
        const glm::vec3 wallH(-6.0f, railY, zH);
        RigidBody hMass; hMass.scale = glm::vec3(0.7f);
        hMass.position = glm::vec3(-1.5f, railY, zH);     // displaced from rest -> SHM in X
        sceneSetCubeMass(hMass, 1.0f); hMass.friction = 0.02f; hMass.restitution = 0.0f;
        bodies.push_back(hMass);                          // 1
        {
            SpringDesc s; s.bodyA = -1; s.localAnchorA = wallH;
            s.bodyB = 1; s.localAnchorB = glm::vec3(0.0f);
            s.restLength = 3.0f; s.stiffness = 90.0f; s.damping = 0.15f;
            springs.push_back(s);
        }

        // ---- 3. Coupled two-mass system (indices 2,3): two masses joined to
        //         each other and to two fixed walls by three springs, in a
        //         horizontal channel -> normal-mode oscillation.
        const float zC = 3.0f;
        const glm::vec3 wallL(-6.0f, railY, zC), wallR(6.0f, railY, zC);
        RigidBody c1; c1.scale = glm::vec3(0.6f);
        c1.position = glm::vec3(-2.5f, railY, zC);        // displaced (asymmetric) -> mixed modes
        sceneSetCubeMass(c1, 1.0f); c1.friction = 0.02f; c1.restitution = 0.0f;
        bodies.push_back(c1);                             // 2
        RigidBody c2; c2.scale = glm::vec3(0.6f);
        c2.position = glm::vec3(2.0f, railY, zC);
        sceneSetCubeMass(c2, 1.0f); c2.friction = 0.02f; c2.restitution = 0.0f;
        bodies.push_back(c2);                             // 3
        {
            SpringDesc sl; sl.bodyA = -1; sl.localAnchorA = wallL; sl.bodyB = 2;
            sl.restLength = 3.0f; sl.stiffness = 70.0f; sl.damping = 0.1f; springs.push_back(sl);
            SpringDesc sm; sm.bodyA = 2; sm.bodyB = 3;
            sm.restLength = 3.5f; sm.stiffness = 70.0f; sm.damping = 0.1f; springs.push_back(sm);
            SpringDesc sr; sr.bodyA = -1; sr.localAnchorA = wallR; sr.bodyB = 3;
            sr.restLength = 3.0f; sr.stiffness = 70.0f; sr.damping = 0.1f; springs.push_back(sr);
        }

        // ---- Apparatus (static, rendered) ------------------------------------
        // Overhead beam for the vertical spring + a vertical guide shaft.
        sceneAddStaticBox(bodies, glm::vec3(-3.0f, 9.3f, zV), glm::vec3(1.2f, 0.3f, 1.2f));
        sceneAddGuideShaft(bodies, -3.0f, 0.5f, 8.6f, 0.8f, 1.0f, 0.12f, 0.02f);
        // Walls + a channel floor for the horizontal oscillator.
        sceneAddStaticBox(bodies, wallH + glm::vec3(-0.35f, 0.0f, 0.0f), glm::vec3(0.4f, 1.6f, 1.2f));
        sceneAddStaticBox(bodies, glm::vec3(0.0f, railY - 0.6f, zH), glm::vec3(10.0f, 0.2f, 1.2f), glm::quat(1,0,0,0), 0.02f);
        // Walls + a channel floor for the coupled pair.
        sceneAddStaticBox(bodies, wallL + glm::vec3(-0.35f, 0.0f, 0.0f), glm::vec3(0.4f, 1.6f, 1.2f));
        sceneAddStaticBox(bodies, wallR + glm::vec3( 0.35f, 0.0f, 0.0f), glm::vec3(0.4f, 1.6f, 1.2f));
        sceneAddStaticBox(bodies, glm::vec3(0.0f, railY - 0.6f, zC), glm::vec3(12.0f, 0.2f, 1.2f), glm::quat(1,0,0,0), 0.02f);
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
        // Default to the canonical ONE-in / ONE-out demo -- the cleanest
        // momentum transfer (0 -> 100 -> 0): the raised bead stops dead on
        // impact and the far bead swings out with the same energy, then it
        // reverses. More raised beads still work but read less cleanly.
        addParam("pullBack", 1.0f, 0.0f, 3.0f); // how many end balls start raised
    }
    const char* name() const override { return "Newton's Cradle"; }
    const char* description() const override { return "A row of suspended balls exchanging momentum on impact."; }
    const char* principle() const override { return "Conservation of momentum and kinetic energy (elastic collision)."; }

    void load() override {
        const int balls = static_cast<int>(param("balls", 5.0f));
        const int pull  = static_cast<int>(param("pullBack", 2.0f));
        const float r   = 0.5f;
        const float railY = 7.0f;         // height of the frame's top rails
        const float len   = 3.0f;         // vertical drop from rail to bead centre
        const float restY = railY - len;  // bead centre height at rest
        const float bifil = 0.7f;         // half-separation of the two ropes in Z

        // Bead centres sit essentially one diameter apart so neighbours just
        // touch. Only a hair of clearance (much smaller than before) so they are
        // not born interpenetrating -- but close enough that the impulse passes
        // through the line in a single step, so momentum transfers cleanly from
        // the struck end to the far end (0 -> 100 -> 0) with minimal loss.
        const float spacing = 2.0f * r + 0.0004f;
        const float x0 = -(balls - 1) * spacing * 0.5f;
        auto beadX = [&](int i) { return x0 + i * spacing; };

        // Reserve for beads + frame (4 posts + 2 rails + 2 feet).
        bodies.reserve(balls + 8);

        // --- Beads first (indices 0..balls-1) : collinear, touching row -------
        for (int i = 0; i < balls; ++i) {
            RigidBody ball;
            ball.position = glm::vec3(beadX(i), restY, 0.0f);
            sceneSetSphereMass(ball, 1.0f, r);
            ball.restitution = 1.0f;   // elastic click -> momentum passes through
            ball.friction = 0.0f;      // frictionless: no spin steals energy
            bodies.push_back(ball);
        }

        // Raise the leftmost `pull` beads along their pendulum arc about their
        // OWN suspension point, so on release they swing down and strike.
        const float a = glm::radians(52.0f);
        for (int i = 0; i < pull && i < balls; ++i) {
            bodies[i].position = glm::vec3(beadX(i) - len * std::sin(a),
                                           railY - len * std::cos(a), 0.0f);
        }

        // --- Dual parallel ropes per bead (bifilar suspension) ----------------
        // Each bead hangs from TWO anchors on the frame rails, offset +/-bifil
        // in Z, attaching to the bead's two sides. Two parallel vertical ropes
        // remove yaw and axial spin and lock every bead into ONE vertical swing
        // plane -- exactly how a real Newton's cradle is strung. Rope length is
        // the true anchor->attachment distance so both are taut at rest.
        const float ropeLen = std::sqrt(len * len + 0.0f); // vertical (anchors are directly above)
        for (int i = 0; i < balls; ++i) {
            RopeDesc front;
            front.bodyA = -1; front.localAnchorA = glm::vec3(beadX(i), railY,  bifil);
            front.bodyB = i;  front.localAnchorB = glm::vec3(0.0f, 0.0f,  bifil);
            front.maxLength = ropeLen;
            ropes.push_back(front);

            RopeDesc back;
            back.bodyA = -1; back.localAnchorA = glm::vec3(beadX(i), railY, -bifil);
            back.bodyB = i;  back.localAnchorB = glm::vec3(0.0f, 0.0f, -bifil);
            back.maxLength = ropeLen;
            ropes.push_back(back);
        }

        // --- Rigid steel frame (static, rendered) -----------------------------
        // Two horizontal top rails (front and back, at z = +/-bifil) carried on
        // four corner posts standing on two feet. The rope anchors sit on these
        // rails, so the whole cradle reads as a real bench-top apparatus.
        const float xEnd = std::abs(x0) + spacing;   // rails overhang the row a little
        const float postFriction = 0.8f;
        // Top rails (long in X, thin, at each z).
        sceneAddStaticBox(bodies, glm::vec3(0.0f, railY + 0.1f,  bifil),
                          glm::vec3(2.0f * xEnd, 0.2f, 0.2f), glm::quat(1,0,0,0), postFriction);
        sceneAddStaticBox(bodies, glm::vec3(0.0f, railY + 0.1f, -bifil),
                          glm::vec3(2.0f * xEnd, 0.2f, 0.2f), glm::quat(1,0,0,0), postFriction);
        // Four corner posts from the feet up to the rails.
        for (float sx : { -xEnd, xEnd })
            for (float sz : { -bifil, bifil })
                sceneAddStaticBox(bodies, glm::vec3(sx, (railY + 0.2f) * 0.5f, sz),
                                  glm::vec3(0.2f, railY + 0.2f, 0.2f), glm::quat(1,0,0,0), postFriction);
        // Two feet tying the posts together front-to-back.
        for (float sx : { -xEnd, xEnd })
            sceneAddStaticBox(bodies, glm::vec3(sx, 0.1f, 0.0f),
                              glm::vec3(0.5f, 0.2f, 2.0f * bifil + 0.4f), glm::quat(1,0,0,0), postFriction);
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
        addParam("angleDeg", 28.0f, 5.0f, 60.0f);
    }
    const char* name() const override { return "Inclined Plane"; }
    const char* description() const override { return "Three blocks of different friction race a rolling ball down a ramp."; }
    const char* principle() const override { return "Downslope pull mg*sinθ vs. friction μmg*cosθ; a ball rolls, blocks slide."; }

    void load() override {
        const float angle = glm::radians(param("angleDeg", 28.0f));
        const float half = 0.5f;              // block half-extent (unit cube)

        // The contact surface is a finite StaticPlane; a matching visible ramp
        // SLAB, triangular support and side rails are built around it so the
        // apparatus is a real bench, not an invisible plane. Surface passes
        // through `surf` with up-normal `n`; it tilts so +x is uphill.
        const glm::vec3 surf(0.0f, 1.2f, 0.0f);
        const glm::vec3 n = glm::normalize(glm::vec3(std::sin(angle), std::cos(angle), 0.0f));
        const float rampLen = 12.0f;          // down-slope length
        const float rampWid = 8.0f;           // across-slope width

        PhysicsSolver::StaticPlane ramp;
        ramp.point = surf; ramp.normal = n;
        ramp.friction = 0.5f; ramp.restitution = 0.05f;
        ramp.halfExtent = glm::vec2(rampLen * 0.5f, rampWid * 0.5f);
        planes.push_back(ramp);

        // In-plane basis: upSlope points uphill along the surface; across is +Z.
        const glm::vec3 upSlope = glm::normalize(glm::vec3(-std::cos(angle), std::sin(angle), 0.0f));
        const glm::vec3 across(0.0f, 0.0f, 1.0f);
        const glm::quat tilt = glm::angleAxis(-angle, glm::vec3(0.0f, 0.0f, 1.0f)); // lie flat on slope

        // --- Racers first (indices 0..3) : three friction blocks + one ball --
        // Released from the same up-slope line so their travel can be compared
        // directly. Low-mu slides farthest; the ball rolls (no sliding loss).
        struct Racer { float mu; float z; bool ball; };
        const Racer racers[4] = {
            { 0.05f, -2.4f, false },
            { 0.30f, -0.8f, false },
            { 0.70f,  0.8f, false },
            { 0.30f,  2.4f, true  },
        };
        for (const Racer& rc : racers) {
            const float rback = rc.ball ? 0.4f : half; // offset off the surface
            const glm::vec3 pos = surf + upSlope * 3.2f + n * (rback + 0.01f) + across * rc.z;
            if (rc.ball) {
                RigidBody b; b.position = pos;
                sceneSetSphereMass(b, 1.0f, 0.4f);
                b.friction = rc.mu; b.restitution = 0.1f;
                bodies.push_back(b);
            } else {
                RigidBody block; block.scale = glm::vec3(1.0f);
                block.orientation = tilt; block.position = pos;
                sceneSetCubeMass(block, 1.0f);
                block.friction = rc.mu; block.restitution = 0.05f;
                bodies.push_back(block);
            }
        }

        // --- Apparatus (static, rendered) -------------------------------------
        // Visible wooden ramp slab flush under the contact surface.
        sceneAddRampSlab(bodies, surf, n, rampLen, rampWid, 0.3f, tilt, 0.5f);

        // Side rails running down BOTH slope edges (at +/-rampWid/2 across),
        // standing proud of the surface, so nothing slides off sideways.
        for (float sz : { -rampWid * 0.5f, rampWid * 0.5f }) {
            const glm::vec3 railCenter = surf + across * sz + n * 0.25f;
            sceneAddStaticBox(bodies, railCenter, glm::vec3(rampLen, 0.5f, 0.25f), tilt, 0.4f);
        }

        // Triangular support wedge under the high (uphill) end + a foot beam,
        // so the ramp reads as sitting on a bench rather than hovering.
        const glm::vec3 highEnd = surf + upSlope * (rampLen * 0.5f);
        sceneAddStaticBox(bodies, glm::vec3(highEnd.x, highEnd.y * 0.5f, 0.0f),
                          glm::vec3(0.6f, highEnd.y, rampWid), glm::quat(1,0,0,0), 0.8f);
        // Angle-indicator marker: a small upright post at the pivot (low end).
        const glm::vec3 lowEnd = surf - upSlope * (rampLen * 0.5f);
        sceneAddStaticBox(bodies, glm::vec3(lowEnd.x, lowEnd.y + 0.4f, rampWid * 0.5f + 0.4f),
                          glm::vec3(0.15f, 0.8f, 0.15f), glm::quat(1,0,0,0), 0.8f);
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
        const int   planks   = static_cast<int>(param("planks", 12.0f));
        const float plankLen = 1.0f;            // plank length along the span (X)
        const float pitch    = plankLen;        // touch end-to-end -> coincident hinges
        const float deckY    = 4.0f;            // deck rest height
        const float span     = (planks - 1) * pitch;
        const float x0       = -span * 0.5f;
        const float deckHalfZ = 1.4f;           // half-width of the roadway
        const glm::vec3 plankScale(plankLen, 0.5f, 2.0f * deckHalfZ);

        // Real suspension bridge: TWO main cables (front at +cz, back at -cz)
        // draped in a parabola between the two towers. EVERY deck plank is held
        // by THREE things: a LEFT vertical hanger (to the front cable), a RIGHT
        // vertical hanger (to the back cable), AND hinges to both neighbours.
        // With a hanger on each side plus neighbour coupling, a plank cannot
        // roll or pendulum independently -- the deck behaves as one continuous
        // structure that sags smoothly and stays level. (A single central
        // hanger per plank, the old design, left each plank free to rock.)
        const float cz        = deckHalfZ - 0.15f;   // cable plane offset in Z
        const float towerTopY = deckY + 5.5f;
        const float cableSagY = deckY + 2.0f;

        // Layout (indices):
        //   0,1                       = towers (static)
        //   frontNode0 .. +planks-1   = front main-cable nodes
        //   backNode0  .. +planks-1   = back main-cable nodes
        //   firstPlank .. +planks-1   = deck planks
        //   last                      = crate load
        bodies.reserve(2 + 2 * planks + planks + 1);

        const float towerH = towerTopY + 1.0f;
        RigidBody towerL; towerL.scale = glm::vec3(0.7f, towerH, 2.0f * deckHalfZ + 1.0f);
        towerL.position = glm::vec3(x0 - pitch, towerH * 0.5f, 0.0f);
        sceneSetCubeMass(towerL, 1.0f); sceneMakeStatic(towerL); towerL.friction = 0.8f;
        bodies.push_back(towerL);                                   // 0
        const float saddleLocalY = towerTopY - towerL.position.y;
        RigidBody towerR = towerL;
        towerR.position = glm::vec3(x0 + span + pitch, towerL.position.y, 0.0f);
        bodies.push_back(towerR);                                   // 1

        auto parabY = [&](int i) {
            const float t = (planks > 1) ? float(i) / (planks - 1) : 0.0f;
            const float p = 4.0f * (t - 0.5f) * (t - 0.5f);        // 1 ends -> 0 mid
            return cableSagY + (towerTopY - cableSagY) * p;
        };

        // Front + back cable nodes (light spheres).
        const int frontNode = 2;
        for (int i = 0; i < planks; ++i) {
            RigidBody n; n.position = glm::vec3(x0 + i * pitch, parabY(i), cz);
            sceneSetSphereMass(n, 0.15f, 0.06f); n.friction = 0.3f; n.restitution = 0.0f;
            bodies.push_back(n);
        }
        const int backNode = frontNode + planks;
        for (int i = 0; i < planks; ++i) {
            RigidBody n; n.position = glm::vec3(x0 + i * pitch, parabY(i), -cz);
            sceneSetSphereMass(n, 0.15f, 0.06f); n.friction = 0.3f; n.restitution = 0.0f;
            bodies.push_back(n);
        }
        // Deck planks.
        const int firstPlank = backNode + planks;
        for (int i = 0; i < planks; ++i) {
            RigidBody p; p.scale = plankScale;
            p.position = glm::vec3(x0 + i * pitch, deckY, 0.0f);
            sceneSetCubeMass(p, 0.5f); p.friction = 0.8f; p.restitution = 0.0f;
            bodies.push_back(p);
        }

        // Saddle each cable's ends over the tower tops (front and back).
        auto saddle = [&](int tower, int nodeIdx, float zsign) {
            RopeDesc e; e.bodyA = tower; e.localAnchorA = glm::vec3(0.0f, saddleLocalY, zsign * cz);
            e.bodyB = nodeIdx; e.localAnchorB = glm::vec3(0.0f);
            e.maxLength = glm::length(bodies[nodeIdx].position
                        - glm::vec3(bodies[tower].position.x, towerTopY, zsign * cz)) + 0.04f;
            ropes.push_back(e);
        };
        saddle(0, frontNode, +1.0f); saddle(1, frontNode + planks - 1, +1.0f);
        saddle(0, backNode,  -1.0f); saddle(1, backNode  + planks - 1, -1.0f);

        // Main cables: node<->node inextensible segments, both cables.
        for (int i = 0; i < planks - 1; ++i) {
            for (int base : { frontNode, backNode }) {
                RopeDesc c; c.bodyA = base + i; c.bodyB = base + i + 1;
                c.maxLength = glm::length(bodies[base + i + 1].position - bodies[base + i].position) + 0.02f;
                ropes.push_back(c);
            }
        }
        // LEFT + RIGHT vertical hangers: front cable -> plank +Z edge, back
        // cable -> plank -Z edge. Two hangers per plank => no independent roll.
        for (int i = 0; i < planks; ++i) {
            const float dropF = bodies[frontNode + i].position.y - deckY;
            RopeDesc hf; hf.bodyA = frontNode + i; hf.localAnchorA = glm::vec3(0.0f);
            hf.bodyB = firstPlank + i; hf.localAnchorB = glm::vec3(0.0f, 0.0f, cz);
            hf.maxLength = dropF; ropes.push_back(hf);

            const float dropB = bodies[backNode + i].position.y - deckY;
            RopeDesc hb; hb.bodyA = backNode + i; hb.localAnchorA = glm::vec3(0.0f);
            hb.bodyB = firstPlank + i; hb.localAnchorB = glm::vec3(0.0f, 0.0f, -cz);
            hb.maxLength = dropB; ropes.push_back(hb);
        }
        // Deck hinges at the coincident touching plank edges (Z axis).
        for (int i = 0; i < planks - 1; ++i) {
            HingeDesc h;
            h.bodyA = firstPlank + i;     h.localAnchorA = glm::vec3(plankLen * 0.5f, 0, 0);
            h.bodyB = firstPlank + i + 1; h.localAnchorB = glm::vec3(-plankLen * 0.5f, 0, 0);
            h.localAxisA = glm::vec3(0, 0, 1); h.localAxisB = glm::vec3(0, 0, 1);
            hinges.push_back(h);
        }
        // Anchor the two END planks to the towers at deck height (fixed
        // abutments) so the span can only sag between fixed ends.
        {
            RopeDesc la; la.bodyA = 0;
            la.localAnchorA = glm::vec3(towerL.scale.x * 0.5f, deckY - towerL.position.y, 0.0f);
            la.bodyB = firstPlank; la.localAnchorB = glm::vec3(-plankLen * 0.5f, 0.0f, 0.0f);
            la.maxLength = 0.06f; ropes.push_back(la);
        }
        {
            RopeDesc ra; ra.bodyA = 1;
            ra.localAnchorA = glm::vec3(-towerR.scale.x * 0.5f, deckY - towerR.position.y, 0.0f);
            ra.bodyB = firstPlank + planks - 1; ra.localAnchorB = glm::vec3(plankLen * 0.5f, 0.0f, 0.0f);
            ra.maxLength = 0.06f; ropes.push_back(ra);
        }

        // A crate resting on the middle plank (box load -> stable face contact).
        const int   midPlank = planks / 2;
        const float midX     = x0 + midPlank * pitch;
        RigidBody crate;
        crate.scale = glm::vec3(1.2f, 0.8f, 1.2f);
        crate.position = glm::vec3(midX, deckY + plankScale.y * 0.5f + 0.4f, 0.0f);
        sceneSetCubeMass(crate, 0.5f);
        crate.restitution = 0.0f; crate.friction = 0.9f;
        bodies.push_back(crate);
    }
};

// ---------------------------------------------------------------------------
// 10. Cantilever Beam -- REMOVED
//
// This scene was removed by request ("the cantilever also just falls to the
// ground, instead of elegantly bending towards it ... please look into this,
// or else remove the scenario").
//
// Investigation conclusion (why it could not be made to bend elegantly):
// a graceful cantilever needs BENDING STIFFNESS -- a restoring torque that
// holds each joint near its rest angle against gravity. This rigid-body
// solver provides no such primitive:
//   * The revolute-hinge ANGLE LIMIT is computed as the angle between the two
//     bodies' hinge AXES. When a beam bends about that shared axis the axes
//     stay nearly parallel, so the measured angle barely changes and the limit
//     never fires -- the beam folds freely to the floor.
//   * The hinge MOTOR is a pure VELOCITY constraint (drives relative bending
//     rate to zero). It has no positional restoring term, so it only DAMPS the
//     sag; gravity integrates a little droop each step and the beam still
//     creeps down to the floor (raising the torque cap changed nothing, since
//     the motor already fully zeroed the bending velocity each step).
//   * Pull-only ROPE ties (top-chord truss, or a cable-stay fan) either
//     compound and stretch under the cumulative load, or let the segments
//     swing inboard like pendulums.
// An elegant continuously-bending beam would require adding an angular-spring
// constraint (torque proportional to angle error, with a position bias) to the
// solver, which is out of scope for a scene change. Rather than ship a scene
// that visibly collapses to the floor, it was removed.
// ---------------------------------------------------------------------------
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
    const char* description() const override { return "Two balls on inextensible strings swinging from a fixed pivot."; }
    const char* principle() const override { return "Chaotic dynamics: sensitivity to initial conditions."; }

    void load() override {
        // Two balls (point masses) joined by NON-STRETCHY strings: string 1
        // from the fixed pivot to ball 1, string 2 from ball 1 to ball 2. Ropes
        // are one-sided inextensible distance constraints, so each string holds
        // its exact length in tension -- the classic double-pendulum idealisation
        // of two bobs on rigid, massless links. Released off-vertical so the
        // motion is chaotic.
        const float startAngle = glm::radians(param("startAngleDeg", 90.0f));
        const glm::vec3 pivot(0.0f, 7.0f, 0.0f);
        const float len1 = 2.0f, len2 = 2.0f;
        const float r = 0.3f;

        bodies.reserve(2);

        // Ball 1: placed along the start angle from the pivot (string 1 taut).
        const glm::vec3 dir1(std::sin(startAngle), -std::cos(startAngle), 0.0f);
        RigidBody bob1;
        bob1.position = pivot + dir1 * len1;
        sceneSetSphereMass(bob1, 1.0f, r);
        bob1.friction = 0.0f; bob1.restitution = 0.0f;
        bodies.push_back(bob1);                                 // index 0

        // Ball 2: hangs straight down from ball 1 (string 2 taut).
        RigidBody bob2;
        bob2.position = bob1.position + glm::vec3(0.0f, -len2, 0.0f);
        sceneSetSphereMass(bob2, 1.0f, r);
        bob2.friction = 0.0f; bob2.restitution = 0.0f;
        bodies.push_back(bob2);                                 // index 1

        // String 1: fixed pivot -> ball 1 (inextensible, length len1).
        RopeDesc s1; s1.bodyA = -1; s1.localAnchorA = pivot;
        s1.bodyB = 0; s1.localAnchorB = glm::vec3(0.0f); s1.maxLength = len1;
        ropes.push_back(s1);

        // String 2: ball 1 -> ball 2 (inextensible, length len2).
        RopeDesc s2; s2.bodyA = 0; s2.localAnchorA = glm::vec3(0.0f);
        s2.bodyB = 1; s2.localAnchorB = glm::vec3(0.0f); s2.maxLength = len2;
        ropes.push_back(s2);
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
    // Counterweight (the load that drives the throw). Threshold raised so a
    // much heavier load is allowed for a more powerful launch.
    TrebuchetScene() { addParam("counterweight", 24.0f, 8.0f, 120.0f); }
    const char* name() const override { return "Trebuchet"; }
    const char* description() const override { return "A counterweight arm on a fixed axle flings a projectile; the frame stays put."; }
    const char* principle() const override { return "Lever mechanical advantage; counterweight PE -> projectile KE via a sling."; }

    void load() override {
        const float cwMass = param("counterweight", 24.0f);
        const float pivotY = 5.0f;                     // axle height
        const glm::vec3 pivot(0.0f, pivotY, 0.0f);
        const float shortArm = 1.3f;                   // pivot -> counterweight side
        const float longArm  = 4.5f;                   // pivot -> sling side
        const float armLen   = shortArm + longArm;

        bodies.reserve(12);

        // --- Static A-frame + base (rendered, immovable) ----------------------
        // A heavy ground sill plus two angled legs meeting at the axle. The
        // pivot itself is anchored to the WORLD (bodyA = -1) so it is perfectly
        // fixed regardless of the frame; the frame is the visible, solid
        // structure the real machine stands on -- it never moves.
        sceneAddStaticBox(bodies, glm::vec3(0.0f, 0.3f, 0.0f), glm::vec3(4.0f, 0.6f, 3.0f), glm::quat(1,0,0,0), 0.9f);
        for (float sz : { -1.1f, 1.1f }) {
            // Two legs leaning in to the axle from front/back, on each side.
            sceneAddStaticBox(bodies, glm::vec3(-0.9f, pivotY * 0.5f + 0.3f, sz),
                              glm::vec3(0.3f, pivotY + 0.6f, 0.3f), glm::angleAxis(glm::radians(12.0f), glm::vec3(0,0,1)), 0.8f);
            sceneAddStaticBox(bodies, glm::vec3( 0.9f, pivotY * 0.5f + 0.3f, sz),
                              glm::vec3(0.3f, pivotY + 0.6f, 0.3f), glm::angleAxis(glm::radians(-12.0f), glm::vec3(0,0,1)), 0.8f);
        }
        // A small axle housing block at the top for looks + a bearing surface.
        sceneAddStaticBox(bodies, glm::vec3(0.0f, pivotY, 0.0f), glm::vec3(0.5f, 0.4f, 2.6f), glm::quat(1,0,0,0), 0.8f);

        // --- Throwing arm (dynamic), hinged to the WORLD at the fixed axle ----
        RigidBody arm; arm.scale = glm::vec3(armLen, 0.22f, 0.22f);
        // Pivot is off-centre: place the arm so its centre sits (longArm-shortArm)/2 toward +x.
        arm.position = pivot + glm::vec3((longArm - shortArm) * 0.5f, 0.0f, 0.0f);
        // Start the long arm cocked DOWN toward +x (loaded position) by tilting.
        arm.orientation = glm::angleAxis(glm::radians(-32.0f), glm::vec3(0,0,1));
        sceneSetCubeMass(arm, 2.0f); arm.friction = 0.3f; arm.restitution = 0.0f;
        const int armIdx = static_cast<int>(bodies.size());
        bodies.push_back(arm);

        // Counterweight (dynamic), hinged beneath the SHORT arm end so it can
        // swing (a hinged bucket, like a real trebuchet).
        RigidBody cw; 
        cw.position = pivot + glm::vec3(-shortArm, -1.2f, 0.0f);
        sceneSetSphereMass(cw, cwMass, 0.6f); cw.friction = 0.4f; cw.restitution = 0.0f;
        const int cwIdx = static_cast<int>(bodies.size());
        bodies.push_back(cw);

        // Projectile (dynamic) in a ground cradle under the long-arm tip; the
        // sling rope whips it up and it releases by geometry. MUST be last so
        // the validator reads it as bs.back().
        RigidBody proj;
        proj.position = pivot + glm::vec3(longArm, -pivotY + 0.4f, 0.0f);
        sceneSetSphereMass(proj, 0.5f, 0.28f); proj.friction = 0.3f; proj.restitution = 0.2f;
        const int projIdx = static_cast<int>(bodies.size());
        bodies.push_back(proj);

        // Arm hinged to the WORLD at the axle (fixed pivot, Z axis).
        HingeDesc h; h.bodyA = -1; h.localAnchorA = pivot;
        h.bodyB = armIdx; h.localAnchorB = glm::vec3(-(longArm - shortArm) * 0.5f - shortArm + shortArm, 0.0f, 0.0f);
        // localAnchorB must be the pivot point in the arm's local frame: the arm
        // centre is at (longArm-shortArm)/2 from the pivot, so the pivot is at
        // -(longArm-shortArm)/2 in local X.
        h.localAnchorB = glm::vec3(-(longArm - shortArm) * 0.5f, 0.0f, 0.0f);
        h.localAxisA = glm::vec3(0,0,1); h.localAxisB = glm::vec3(0,0,1);
        hinges.push_back(h);

        // Counterweight roped to the short-arm end (a swinging bucket).
        RopeDesc cwRope; cwRope.bodyA = armIdx; cwRope.localAnchorA = glm::vec3(-armLen * 0.5f, 0.0f, 0.0f);
        cwRope.bodyB = cwIdx; cwRope.localAnchorB = glm::vec3(0.0f); cwRope.maxLength = 1.2f;
        ropes.push_back(cwRope);

        // Sling: rope from the long-arm tip to the projectile.
        RopeDesc sling; sling.bodyA = armIdx; sling.localAnchorA = glm::vec3(armLen * 0.5f, 0.0f, 0.0f);
        sling.bodyB = projIdx; sling.localAnchorB = glm::vec3(0.0f); sling.maxLength = 2.4f;
        ropes.push_back(sling);
    }
};

// ---------------------------------------------------------------------------
// 14. Ballistics
//
// A fan of identical projectiles launched from a common point at the same
// speed but different angles. With gravity only (no drag), each follows the
// ideal parabola R = v² sin(2θ)/g, so the 45° shot lands farthest and shots at
// complementary angles (e.g. 30°/60°) land together — the classic ballistics
// demonstration.
//
// This scene is backed by the validation lab: EXPERIMENT 7 measures the same
// projectile kinematics against R = v²sin2θ/g and T = 2v sinθ/g to within
// ~0.6% (150/150 randomized variants stable), so what you see here is a
// quantitatively verified parabola, not an approximate-looking arc.
// ---------------------------------------------------------------------------
class BallisticsScene : public Scene {
public:
    BallisticsScene() {
        addParam("launchSpeed", 14.0f, 5.0f, 25.0f);
        addParam("shots",        5.0f, 1.0f, 7.0f);
        addParam("drag",         0.0f, 0.0f, 1.0f);   // 0 = vacuum, 1 = atmosphere
    }
    const char* name() const override { return "Ballistics"; }
    const char* description() const override { return "Cannons fire equal-speed shots at varying angles; 45 deg flies farthest."; }
    const char* principle() const override { return "Parabolic trajectory; range R = v^2 sin(2theta)/g is maximal at 45 deg."; }

    void load() override {
        const float v     = param("launchSpeed", 14.0f);
        const int   shots = static_cast<int>(param("shots", 5.0f));
        const bool  drag  = param("drag", 0.0f) > 0.5f;
        const float r     = 0.25f;
        const float barrelLen = 1.6f;
        const glm::vec3 breech(-9.0f, 0.6f, 0.0f);   // common breech (pivot) point

        // Turn on aerodynamic drag for the atmosphere comparison. With drag OFF
        // (vacuum) the ideal parabola applies and 45 deg gives the max range --
        // the regime the validation lab checks to ~0.6%. With drag ON, range
        // shrinks and the optimum angle drops below 45 deg.
        aerodynamicsEnabled = drag;
        airDensity = drag ? 1.225f : 0.0f;

        bodies.reserve(shots * 2 + 4);

        // Projectiles FIRST (indices 0..shots-1) so index-based checks hold.
        // Each is spawned at its cannon's MUZZLE with velocity along the barrel.
        std::vector<glm::vec3> muzzle(shots);
        std::vector<float> ang(shots);
        for (int i = 0; i < shots; ++i) {
            const float t = (shots > 1) ? float(i) / (shots - 1) : 0.5f;
            ang[i] = glm::radians(20.0f + t * 50.0f);          // 20..70 deg
            const glm::vec3 dir(std::cos(ang[i]), std::sin(ang[i]), 0.0f);
            muzzle[i] = breech + dir * barrelLen + glm::vec3(0.0f, 0.0f, (i - (shots - 1) * 0.5f) * 0.9f);

            RigidBody proj;
            proj.position = muzzle[i];
            sceneSetSphereMass(proj, 1.0f, r);
            proj.restitution = 0.3f; proj.friction = 0.4f;
            proj.dragCoefficient = 0.47f;                       // sphere Cd
            proj.velocity = dir * v;                            // fired along the barrel
            bodies.push_back(proj);
        }

        // Visible cannon barrels (static), one per shot, oriented to its angle
        // with the muzzle at the projectile's start. A short thick tube.
        for (int i = 0; i < shots; ++i) {
            const glm::vec3 dir(std::cos(ang[i]), std::sin(ang[i]), 0.0f);
            const glm::vec3 zoff(0.0f, 0.0f, (i - (shots - 1) * 0.5f) * 0.9f);
            const glm::vec3 center = breech + dir * (barrelLen * 0.5f) + zoff;
            const glm::quat rot = glm::angleAxis(ang[i], glm::vec3(0, 0, 1));
            sceneAddStaticBox(bodies, center, glm::vec3(barrelLen, 0.35f, 0.35f), rot, 0.5f);
        }

        // Target rings/markers downrange: three upright posts at representative
        // ranges so the landing points can be read off.
        for (float tx : { 4.0f, 10.0f, 16.0f })
            sceneAddStaticBox(bodies, glm::vec3(tx, 0.6f, 0.0f), glm::vec3(0.15f, 1.2f, 0.15f), glm::quat(1,0,0,0), 0.8f);
    }
};

// ---------------------------------------------------------------------------
// 15. Hanging Chain Wave
//
// A long horizontal chain of small spheres, each linked to its neighbour by a
// short rope and both ends pinned to fixed world anchors. One interior node is
// lifted at load time, so when the sim starts a transverse pulse travels along
// the chain, reflects off the fixed ends, and interferes with itself.
// Demonstrates transverse wave propagation and reflection on a discretised
// string. This is a QUALITATIVE wave lab: the medium is a mass-spring/rope
// chain, so the exact wave speed depends on link tension, not a closed-form
// c = sqrt(T/mu). It is deterministic and does not explode.
// ---------------------------------------------------------------------------
class HangingChainWaveScene : public Scene {
public:
    HangingChainWaveScene() {
        addParam("links",  70.0f, 40.0f, 100.0f);      // 40-100 beads
        // Sideways flick strength (m/s imparted to the kicked beads). This is
        // the "jerk force" slider: a wider range so the wave can be launched
        // gently or hard.
        addParam("jerkForce", 9.0f, 1.0f, 3000.0f);
    }
    const char* name() const override { return "Hanging Chain Wave"; }
    const char* description() const override { return "A chain lying on the ground; a sideways flick sends a wave along it."; }
    const char* principle() const override { return "Transverse wave propagation along a discrete chain."; }

    void load() override {
        // The chain lies FLAT ON THE GROUND in a straight line along X, each
        // bead resting on the floor, and is held TAUT between two fixed floor
        // anchors -- one at each end. Because the anchors are placed slightly
        // farther apart than the sum of the link lengths, every rope is
        // pre-tensioned (loaded in tension). A transverse wave can only travel
        // along a string that carries axial tension: the wave speed is
        // c = sqrt(T / mu). A bead near one end is then flicked SIDEWAYS (+Z);
        // the pre-tension in the neighbouring links resists that lateral
        // displacement and pulls the next bead along, so the kink propagates
        // bead-to-bead as a transverse pulse travelling toward the centre.
        //
        // (An earlier design left the chain SLACK and pinned only one end. That
        // does not work here: the rope constraints are ONE-SIDED -- they pull
        // when stretched but never push -- so a slack chain has no restoring
        // tension and the transverse pulse dies within a few beads. Real
        // rope-snapping waves rely on the rope being held taut; this rebuild
        // reproduces that by pre-tensioning the chain between two anchors.)
        const int   links = static_cast<int>(param("links", 70.0f));
        const float jerk  = param("jerkForce", 9.0f);
        const float span  = 24.0f;                  // anchor-to-anchor distance (X)
        const float x0    = -span * 0.5f;
        const float dx    = span / (links - 1);
        const float r     = dx * 0.3f;              // rests on the floor at y = r
        // Links are slightly SHORTER than the bead spacing so that, stretched
        // between the two fixed end anchors, every rope sits at its length
        // limit and carries tension -- the medium that propagates the wave.
        const float linkLen = dx * 0.97f;

        bodies.reserve(links);
        for (int i = 0; i < links; ++i) {
            RigidBody b;
            b.position = glm::vec3(x0 + i * dx, r, 0.0f);   // flat on the ground
            sceneSetSphereMass(b, 0.2f, r);
            b.friction = 0.02f;      // very low friction: free to slide sideways
            b.restitution = 0.0f;
            bodies.push_back(b);
        }

        // A sudden sideways JERK near one end (a few beads in from the left,
        // clear of the pinned corner): give those beads a lateral (+Z) velocity
        // so a transverse kink is launched down the taut chain.
        const int kickStart = std::max(1, links / 20);
        const int kicked    = std::max(3, links / 14);
        for (int i = kickStart; i < kickStart + kicked && i < links - 1; ++i) {
            bodies[i].velocity = glm::vec3(0.0f, 0.0f, jerk);
        }

        // Neighbour ropes tie the beads into a chain. Inextensible + slightly
        // pre-stretched (length = 0.97 * spacing) so the chain carries tension.
        for (int i = 0; i < links - 1; ++i) {
            RopeDesc rp;
            rp.bodyA = i; rp.bodyB = i + 1;
            rp.localAnchorA = glm::vec3(0.0f); rp.localAnchorB = glm::vec3(0.0f);
            rp.maxLength = linkLen;
            ropes.push_back(rp);
        }

        // Pin BOTH ends to fixed floor anchors. Holding both ends is what keeps
        // the chain under tension (the medium the wave travels through) and
        // gives the far end something to reflect off. The kicked bead sits a
        // few beads inboard of the left anchor, so the flick is not fighting the
        // anchor directly -- it deflects the taut span, launching a clean pulse.
        RopeDesc left; left.bodyA = -1;
        left.localAnchorA = glm::vec3(x0, r, 0.0f);
        left.bodyB = 0; left.localAnchorB = glm::vec3(0.0f); left.maxLength = 0.03f;
        ropes.push_back(left);

        RopeDesc right; right.bodyA = -1;
        right.localAnchorA = glm::vec3(x0 + (links - 1) * dx, r, 0.0f);
        right.bodyB = links - 1; right.localAnchorB = glm::vec3(0.0f); right.maxLength = 0.03f;
        ropes.push_back(right);
    }
};

// ---------------------------------------------------------------------------
// 16. Object Volume (many-body stress test)
//
// (Replaces the old "Domino Cathedral", removed by request.) A dense VOLUME of
// many rigid bodies -- a packed grid of boxes and spheres -- is dropped into a
// walled bin. It stress-tests the broad phase and the per-body contact
// resolution: with hundreds of simultaneous contacts every body's motion
// DIRECTION must be computed correctly (each settles by sliding/rolling along
// its real contact normals) and the whole pile must come to rest without
// exploding, tunnelling through the floor, or drifting out of the bin. This is
// the "many bodies and their direction being calculated" test.
//
// (Kept for reference: the old Domino Cathedral cascade below the fold.)
// ---------------------------------------------------------------------------
class ObjectVolumeScene : public Scene {
public:
    ObjectVolumeScene() {
        // Grid edge count per axis: bodies ~= count^2 * layers. Bounded so the
        // scene stays interactive.
        addParam("gridCount", 6.0f, 3.0f, 9.0f);
        addParam("layers",    3.0f, 1.0f, 5.0f);
    }
    const char* name() const override { return "Object Volume"; }
    const char* description() const override { return "A dense volume of many bodies dropped into a bin; stress-tests contact resolution."; }
    const char* principle() const override { return "Broad-phase + many-contact solve: every body's motion direction resolved from its real contacts."; }

    void load() override {
        const int   n      = static_cast<int>(param("gridCount", 6.0f));
        const int   layers = static_cast<int>(param("layers", 3.0f));
        const float cell   = 1.2f;                 // horizontal spacing of the grid
        const float half   = 0.4f;                 // body half-size
        const float span   = (n - 1) * cell;       // footprint of the grid
        const float x0     = -span * 0.5f;
        const float z0     = -span * 0.5f;

        bodies.reserve(n * n * layers + 8);

        // --- Static bin: floor is the solver's ground plane; add four low walls
        //     so the pile is confined and lateral drift is a real failure (not
        //     just "everything scattered off the edge"). ---
        const float wallH   = 3.0f;
        const float binHalf = span * 0.5f + cell;  // walls just outside the grid
        const float t       = 0.3f;                // wall thickness
        sceneAddStaticBox(bodies, glm::vec3(0.0f, wallH * 0.5f,  binHalf), glm::vec3(2.0f * binHalf + t, wallH, t), glm::quat(1,0,0,0), 0.6f);
        sceneAddStaticBox(bodies, glm::vec3(0.0f, wallH * 0.5f, -binHalf), glm::vec3(2.0f * binHalf + t, wallH, t), glm::quat(1,0,0,0), 0.6f);
        sceneAddStaticBox(bodies, glm::vec3( binHalf, wallH * 0.5f, 0.0f), glm::vec3(t, wallH, 2.0f * binHalf + t), glm::quat(1,0,0,0), 0.6f);
        sceneAddStaticBox(bodies, glm::vec3(-binHalf, wallH * 0.5f, 0.0f), glm::vec3(t, wallH, 2.0f * binHalf + t), glm::quat(1,0,0,0), 0.6f);

        // --- Many dynamic bodies stacked in `layers` above the bin. Alternate
        //     boxes and spheres so both collision paths are exercised, and give
        //     each layer a small horizontal OFFSET so the stack is not a perfect
        //     lattice -- forcing genuine sideways contact resolution as they
        //     settle (their motion direction must be derived from contacts, not
        //     luck). Deterministic (no RNG): offset is a fixed function of the
        //     grid indices. ---
        for (int ly = 0; ly < layers; ++ly) {
            const float y = half + 0.5f + ly * (2.0f * half + 0.25f);
            // Fixed per-layer offset (deterministic), a fraction of a cell.
            const float ox = ((ly % 2) ? 0.25f : -0.15f) * cell;
            const float oz = ((ly % 3) ? -0.2f : 0.3f) * cell;
            for (int ix = 0; ix < n; ++ix) {
                for (int iz = 0; iz < n; ++iz) {
                    const float x = x0 + ix * cell + ox;
                    const float z = z0 + iz * cell + oz;
                    RigidBody b;
                    b.position = glm::vec3(x, y, z);
                    // Alternate boxes/spheres in a checkerboard by (ix+iz+ly).
                    if (((ix + iz + ly) & 1) == 0) {
                        b.scale = glm::vec3(2.0f * half);
                        sceneSetCubeMass(b, 0.5f);
                    } else {
                        sceneSetSphereMass(b, 0.5f, half);
                    }
                    b.friction = 0.5f;
                    b.restitution = 0.05f;
                    bodies.push_back(b);
                }
            }
        }
    }
};

#if 0  // ---- old Domino Cathedral (removed, kept for reference) ----
// 16b. Domino Cathedral
//
// A large deterministic layout of dominoes — several curved spiral arms feeding
// into straight runs — with the first domino tipped so a cascade propagates
// through the whole structure. Demonstrates sequential energy transfer and the
// spacing/height stability condition for a toppling chain (a domino must fall
// far enough to strike its neighbour above the neighbour's centre of mass).
// ---------------------------------------------------------------------------
class DominoCathedralScene : public Scene {
public:
    DominoCathedralScene() {
        addParam("runLength", 22.0f, 10.0f, 40.0f);   // dominoes in the straight+spiral run
    }
    const char* name() const override { return "Domino Cathedral"; }
    const char* description() const override { return "A marble triggers a domino run + spiral that topples a final tower."; }
    const char* principle() const override { return "Engineered chain reaction; toppling condition spacing < domino height."; }

    void load() override {
        // Domino geometry. The critical design number is the SPACING: a domino
        // of height H, tipping about its base edge, sweeps forward by H, so to
        // guarantee it strikes its neighbour ABOVE that neighbour's centre of
        // mass the spacing must satisfy s < H. We derive s from H (not a magic
        // constant) and use 0.62*H for a reliable, energetic cascade.
        const glm::vec3 half(0.10f, 0.55f, 0.32f);   // half-extents (thin/tall/deep)
        const float H       = half.y * 2.0f;         // domino height
        const float spacing = 0.52f * H;             // derived toppling spacing:
                                                     // comfortably < H so each
                                                     // domino strikes the next
                                                     // well above its COM even
                                                     // through the spiral turn
        const int   dominoes = static_cast<int>(param("runLength", 22.0f));

        bodies.reserve(dominoes + 16);

        // Place a domino standing upright, facing along `heading` (about Y), at
        // ground level; `tipped` leans the starter forward to begin the chain.
        auto placeDomino = [&](const glm::vec3& groundPos, float heading, bool tipped) {
            RigidBody d; d.scale = half * 2.0f;
            glm::quat yaw = glm::angleAxis(heading, glm::vec3(0, 1, 0));
            // The forward (toppling) direction for this domino: it falls about
            // the lateral axis so its top sweeps along +local-X (the run dir).
            const glm::vec3 fwd  = yaw * glm::vec3(1, 0, 0);
            // Axis to rotate about so the top sweeps forward (+fwd): up x fwd.
            const glm::vec3 lateral = glm::normalize(glm::cross(glm::vec3(0, 1, 0), fwd));
            if (tipped) {
                // Lean the starter PAST its balance angle AND give it a forward
                // angular velocity, so it positively topples into its neighbour
                // and starts the chain (a small static lean can just settle back).
                const float lean = glm::radians(30.0f);
                d.orientation = glm::angleAxis(lean, lateral) * yaw;
                d.position = glm::vec3(groundPos.x, half.y * std::cos(lean) + 0.02f, groundPos.z);
                d.angularVelocity = lateral * 2.5f;        // topple forward
            } else {
                d.orientation = yaw;
                d.position = glm::vec3(groundPos.x, half.y, groundPos.z);
            }
            sceneSetCubeMass(d, 0.3f);
            d.friction = 0.55f; d.restitution = 0.0f;
            bodies.push_back(d);
        };

        // ---- Stage 1: curved marble entry track (static ramp) + a marble -----
        // A short downhill ramp aimed at the first domino. The marble rolls off
        // it and strikes the run's head, so the cascade is triggered by real
        // rolling motion rather than a hand-tipped domino alone.
        const glm::vec3 runStart(-9.0f, 0.0f, 0.0f);       // first domino ground pos
        const float rampAngle = glm::radians(20.0f);
        const glm::vec3 rn = glm::normalize(glm::vec3(-std::sin(rampAngle), std::cos(rampAngle), 0.0f));
        const glm::vec3 rampSurf(runStart.x - 2.4f, 0.9f, 0.0f);
        const glm::quat rampRot = glm::angleAxis(rampAngle, glm::vec3(0, 0, 1));
        sceneAddRampSlab(bodies, rampSurf, rn, 3.4f, 1.6f, 0.3f, rampRot, 0.3f);   // (body added, static)

        // ---- Stage 2: straight domino run heading +X ------------------------
        // First ~60% of the dominoes march straight toward +X; the STARTER (the
        // domino nearest the ramp) is tipped so the marble+lean begins the run.
        const int straightN = (dominoes * 3) / 5;
        for (int i = 0; i < straightN; ++i) {
            const glm::vec3 pos = runStart + glm::vec3(i * spacing, 0.0f, 0.0f);
            placeDomino(pos, 0.0f, i == 0);                 // heading +X, tip the first
        }

        // ---- Stage 3: the run curves into an inward spiral ------------------
        // The remaining dominoes bend the path into a tightening arc, feeding
        // the final tower at the centre. Heading turns a little each step; the
        // step length stays = spacing so the toppling condition still holds.
        glm::vec3 pos = runStart + glm::vec3((straightN - 1) * spacing, 0.0f, 0.0f);
        float heading = 0.0f;
        const float turnPerStep = glm::radians(7.0f);       // gentle inward curve
                                                            // (small so the swept
                                                            // domino still lands
                                                            // squarely on the next)
        for (int i = straightN; i < dominoes; ++i) {
            heading += turnPerStep;
            pos += glm::vec3(std::cos(heading), 0.0f, std::sin(heading)) * spacing;
            placeDomino(pos, heading, false);
        }

        // ---- Stage 4: final tower at the end of the spiral ------------------
        // A small stack of blocks the last domino topples into -- the climactic
        // "cathedral" that falls when the cascade arrives.
        const glm::vec3 towerBase = pos + glm::vec3(std::cos(heading), 0.0f, std::sin(heading)) * (spacing * 1.2f);
        float ty = 0.4f;
        for (int k = 0; k < 4; ++k) {
            RigidBody b; b.scale = glm::vec3(0.8f, 0.8f, 0.8f);
            b.position = glm::vec3(towerBase.x, ty, towerBase.z);
            sceneSetCubeMass(b, 0.5f); b.friction = 0.6f; b.restitution = 0.0f;
            bodies.push_back(b);
            ty += 0.82f;
        }

        // The marble that rolls down the entry ramp toward the run head. The
        // tipped starter domino guarantees the cascade begins; the marble is
        // the visible "trigger" that arrives from the ramp. Placed just above
        // the ramp's upper surface so it rolls down under gravity.
        RigidBody marble;
        marble.position = rampSurf + rn * 0.4f + glm::vec3(-1.0f, 0.6f, 0.0f);
        sceneSetSphereMass(marble, 1.2f, 0.35f);
        marble.friction = 0.3f; marble.restitution = 0.1f;
        bodies.push_back(marble);
    }
};
#endif  // old Domino Cathedral

// ---------------------------------------------------------------------------
// 17. Cable-Stayed Bridge
//
// A central pylon rises from a static pier. A fan of stay cables (ropes) runs
// from near the pylon top down to a series of deck planks, which are also roped
// to their neighbours and anchored at both abutments. The deck hangs in tension
// from the cables rather than resting on the ground. Demonstrates tensile load
// paths: deck weight is carried up the inclined cables into the pylon and down
// to the foundation.
// ---------------------------------------------------------------------------
class CableStayedBridgeScene : public Scene {
public:
    CableStayedBridgeScene() {
        addParam("planks", 12.0f, 6.0f, 20.0f);
    }
    const char* name() const override { return "Cable-Stayed Bridge"; }
    const char* description() const override { return "A roadway deck suspended in tension from fanned stay cables."; }
    const char* principle() const override { return "Tensile load paths: deck weight carried up inclined cables into the pylon."; }

    void load() override {
        // Deck geometry. Planks must be an even count so the pylon sits at the
        // gap in the middle of the span.
        int planks = static_cast<int>(param("planks", 12.0f));
        if (planks % 2 != 0) planks += 1;

        const float deckY    = 6.0f;                 // suspended height of the deck
        const float plankLen = 1.6f;                 // full length of each plank (X)
        const float pitch    = plankLen;             // touch end-to-end -> hinge
                                                     // anchors at +/-plankLen/2
                                                     // are COINCIDENT (no fight)
        const float span     = planks * pitch;
        const float xStart   = -span * 0.5f + plankLen * 0.5f;
        const float pylonTopY = deckY + 6.0f;        // cable anchor height on the pylon

        bodies.reserve(planks + 2);

        // --- Static pier + pylon at mid-span (index 0) ---
        RigidBody pylon;
        pylon.scale = glm::vec3(0.6f, pylonTopY, 0.6f);
        pylon.position = glm::vec3(0.0f, pylonTopY * 0.5f, 0.0f);
        sceneSetCubeMass(pylon, 1.0f);
        sceneMakeStatic(pylon);
        pylon.friction = 0.8f;
        bodies.push_back(pylon);                     // index 0
        const int pylonIdx = 0;
        const glm::vec3 pylonTop(0.0f, pylonTopY, 0.0f);

        // --- Deck planks (indices 1..planks) ---
        const int firstPlank = 1;
        for (int i = 0; i < planks; ++i) {
            RigidBody p;
            p.scale = glm::vec3(plankLen, 0.15f, 2.0f);
            p.position = glm::vec3(xStart + i * pitch, deckY, 0.0f);
            sceneSetCubeMass(p, 0.8f);
            p.friction = 0.6f;
            p.restitution = 0.0f;
            bodies.push_back(p);
        }

        // --- Plank-to-plank HINGES (the deck acts as a continuous roadway) ---
        // Revolute hinges about Z keep the deck planks aligned and connected
        // (no free spinning) while still letting the span flex under load.
        for (int i = 0; i < planks - 1; ++i) {
            HingeDesc h;
            h.bodyA = firstPlank + i;     h.localAnchorA = glm::vec3(plankLen * 0.5f, 0.0f, 0.0f);
            h.bodyB = firstPlank + i + 1; h.localAnchorB = glm::vec3(-plankLen * 0.5f, 0.0f, 0.0f);
            h.localAxisA = glm::vec3(0, 0, 1); h.localAxisB = glm::vec3(0, 0, 1);
            hinges.push_back(h);
        }

        // --- Abutment anchors: pin the two end planks to fixed world points at
        //     deck height with taut short ropes so the span's ends are fixed
        //     and it cannot swing away or rotate as a rigid body. ---
        RopeDesc leftAb;  leftAb.bodyA = -1;
        leftAb.localAnchorA = glm::vec3(xStart - plankLen * 0.5f, deckY, 0.0f);
        leftAb.bodyB = firstPlank; leftAb.localAnchorB = glm::vec3(-plankLen * 0.5f, 0.0f, 0.0f);
        leftAb.maxLength = 0.05f; ropes.push_back(leftAb);

        RopeDesc rightAb; rightAb.bodyA = -1;
        rightAb.localAnchorA = glm::vec3(xStart + (planks - 1) * pitch + plankLen * 0.5f, deckY, 0.0f);
        rightAb.bodyB = firstPlank + planks - 1; rightAb.localAnchorB = glm::vec3(plankLen * 0.5f, 0.0f, 0.0f);
        rightAb.maxLength = 0.05f; ropes.push_back(rightAb);

        // --- Stay cables: fan from the pylon top to each plank. Cable length is
        //     set to the exact geometric distance so the deck hangs in tension
        //     at its intended height (no slack, no ground contact). ---
        for (int i = 0; i < planks; ++i) {
            const int plankIdx = firstPlank + i;
            const glm::vec3 plankPos(xStart + i * pitch, deckY, 0.0f);
            const float dist = glm::length(plankPos - pylonTop);
            RopeDesc stay;
            stay.bodyA = pylonIdx;  stay.localAnchorA = glm::vec3(0.0f, pylonTopY * 0.5f, 0.0f); // local top of pylon
            stay.bodyB = plankIdx;  stay.localAnchorB = glm::vec3(0.0f, 0.0f, 0.0f);
            stay.maxLength = dist;  // taut cable of exact geometric length
            ropes.push_back(stay);
        }

        // A wide pier base under the pylon so the tower reads as founded on the
        // ground rather than floating (static, rendered).
        sceneAddStaticBox(bodies, glm::vec3(0.0f, 0.4f, 0.0f), glm::vec3(2.0f, 0.8f, 3.0f));

        // A crate placed off-centre on the deck. Its weight loads the nearby
        // stays most, and the fan redistributes the rest into the pylon -- the
        // structural-redundancy point of a cable-stayed bridge. Off-centre so
        // asymmetric loading is visible. Placed on a specific plank's surface.
        const int   loadPlank = firstPlank + planks / 4;   // quarter-span
        const float loadX     = xStart + (planks / 4) * pitch;
        RigidBody crate;
        crate.scale = glm::vec3(1.2f, 0.8f, 1.2f);
        crate.position = glm::vec3(loadX, deckY + 0.15f * 0.5f + 0.4f, 0.0f);
        sceneSetCubeMass(crate, 0.6f);
        crate.restitution = 0.0f; crate.friction = 0.9f;
        bodies.push_back(crate);
    }
};

// ---------------------------------------------------------------------------
// 18. Truss Collapse -- REMOVED
//
// Removed by request ("remove the one where the structure is supposed to
// collapse, cuz it just sucks"). It relied on stacked beams losing a central
// support and sagging, which read as an unconvincing pile-up rather than a
// clean structural-failure demonstration.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 19. Gyroscope
//
// A flywheel (a flat, disk-like box) mounted on an offset hinge and spun up to a
// high angular velocity. A real gyroscope with an off-centre support precesses:
// its spin axis slowly rotates about the vertical under gravity's torque,
// Omega = tau / (I*omega).
//
// HONEST NOTE: the validation lab (EXPERIMENT 8) measured this engine's
// precession rate at ~0 rad/s versus a theoretical ~1.0 rad/s — the sequential
// impulse solver dissipates the gyroscopic coupling, so the flywheel here spins
// and stays up but does NOT reproduce quantitative precession. It is included
// as a stable, deterministic demonstrator and a standing regression benchmark
// for future angular-dynamics work, not as a validated precession lab.
// ---------------------------------------------------------------------------
class GyroscopeScene : public Scene {
public:
    GyroscopeScene() {
        addParam("rpm", 600.0f, 0.0f, 1500.0f);
    }
    const char* name() const override { return "Gyroscope"; }
    const char* description() const override { return "A spinning flywheel on an offset support (precession is not yet reproduced)."; }
    const char* principle() const override { return "Angular momentum of a spinning rotor; ideal precession Omega = tau/(I*omega)."; }

    void load() override {
        // Laboratory gyroscope apparatus: a wide pedestal base carries a bearing
        // housing; the axle runs on a fixed bearing (a world hinge) and a disk
        // flywheel spins at its tip inside a gimbal ring. The support stays put.
        //
        // HONEST LIMITATION: this engine's sequential-impulse angular integrator
        // dissipates gyroscopic coupling (the validation lab measured precession
        // ~0 vs ~1 rad/s theory), and there is no cylinder primitive (the wheel
        // is a thin disk-like box). So the flywheel spins and the rig stays
        // upright, but it does NOT reproduce quantitative precession. It ships
        // as a stable demonstrator + standing regression benchmark, not a
        // validated precession lab. The success test asserts only that.
        const float rpm   = param("rpm", 600.0f);
        const float omega = rpm * 2.0f * 3.14159265f / 60.0f;   // rad/s about spin axis
        const float pivotY = 4.5f;
        const glm::vec3 pivot(0.0f, pivotY, 0.0f);
        const float axleLen = 1.6f;

        bodies.reserve(10);

        // --- Dynamic parts FIRST so the wheel (heaviest) is easy to find ------
        // Axle (index 0): from the bearing out to the flywheel.
        RigidBody axle;
        axle.scale = glm::vec3(axleLen, 0.15f, 0.15f);
        axle.position = pivot + glm::vec3(axleLen * 0.5f, 0.0f, 0.0f);
        sceneSetCubeMass(axle, 0.3f); axle.friction = 0.0f; axle.restitution = 0.0f;
        const int axleIdx = 0; bodies.push_back(axle);

        // Flywheel (index 1): a thick disk-like box at the axle tip, spinning
        // about the axle (X) axis. Heaviest dynamic body.
        RigidBody wheel;
        wheel.scale = glm::vec3(0.3f, 1.8f, 1.8f);
        wheel.position = pivot + glm::vec3(axleLen, 0.0f, 0.0f);
        sceneSetCubeMass(wheel, 5.0f); wheel.friction = 0.0f; wheel.restitution = 0.0f;
        wheel.angularVelocity = glm::vec3(omega, 0.0f, 0.0f);
        const int wheelIdx = 1; bodies.push_back(wheel);

        // Axle on a FIXED bearing: world hinge at the pivot (Y axis = the
        // vertical precession axis of an ideal gyro).
        HingeDesc h; h.bodyA = -1; h.localAnchorA = pivot;
        h.bodyB = axleIdx; h.localAnchorB = glm::vec3(-axleLen * 0.5f, 0.0f, 0.0f);
        h.localAxisA = glm::vec3(0, 1, 0); h.localAxisB = glm::vec3(0, 1, 0);
        hinges.push_back(h);

        // Keep the spinning wheel located at the axle tip (short taut rope; the
        // solver has no weld joint, so the wheel spins freely about its own axis
        // while staying attached).
        RopeDesc weld; weld.bodyA = axleIdx; weld.localAnchorA = glm::vec3(axleLen * 0.5f, 0.0f, 0.0f);
        weld.bodyB = wheelIdx; weld.localAnchorB = glm::vec3(-0.15f, 0.0f, 0.0f);
        weld.maxLength = 0.05f; ropes.push_back(weld);

        // --- Apparatus (static, rendered) -------------------------------------
        // Wide pedestal base + column + bearing housing at the pivot.
        sceneAddStaticBox(bodies, glm::vec3(-0.3f, 0.4f, 0.0f), glm::vec3(2.4f, 0.8f, 2.4f)); // base
        sceneAddStaticBox(bodies, glm::vec3(-0.3f, pivotY * 0.5f + 0.4f, 0.0f),
                          glm::vec3(0.5f, pivotY, 0.5f));                                     // column
        sceneAddStaticBox(bodies, glm::vec3(-0.3f, pivotY, 0.0f), glm::vec3(0.7f, 0.6f, 0.7f)); // bearing housing
        // Gimbal ring around the flywheel: four thin static bars framing it (a
        // suggested ring; the wheel spins freely inside).
        const glm::vec3 wc = pivot + glm::vec3(axleLen, 0.0f, 0.0f);
        const float rr = 1.15f;
        sceneAddStaticBox(bodies, wc + glm::vec3(0.0f,  rr, 0.0f), glm::vec3(0.1f, 0.1f, 2.2f)); // top bar
        sceneAddStaticBox(bodies, wc + glm::vec3(0.0f, -rr, 0.0f), glm::vec3(0.1f, 0.1f, 2.2f)); // bottom bar
        sceneAddStaticBox(bodies, wc + glm::vec3(0.0f, 0.0f,  rr), glm::vec3(0.1f, 2.2f, 0.1f)); // front bar
        sceneAddStaticBox(bodies, wc + glm::vec3(0.0f, 0.0f, -rr), glm::vec3(0.1f, 2.2f, 0.1f)); // back bar
    }
};

// ---------------------------------------------------------------------------
// 20. Explosion
//
// A tight cluster of debris fragments at the centre is given a sudden RADIAL
// impulse -- a blast. Each fragment flies outward along the line from the blast
// centre through its own position, with speed set so the total kinetic energy
// equals the "energy" slider. Momentum is balanced (the cluster is symmetric
// about the centre) so the debris field expands outward without the whole pile
// translating. Fragments arc under gravity and settle. Demonstrates a radial
// impulse field and per-body direction from geometry (each body's launch
// direction is its offset from the blast centre).
// ---------------------------------------------------------------------------
class ExplosionScene : public Scene {
public:
    ExplosionScene() {
        // Blast energy in joules, shared across all fragments (KE = 0.5 m v^2).
        addParam("energy",    800.0f, 50.0f, 4000.0f);
        addParam("fragments", 40.0f,  8.0f,  80.0f);
    }
    const char* name() const override { return "Explosion"; }
    const char* description() const override { return "A cluster of fragments blown radially outward by an energy-controlled blast."; }
    const char* principle() const override { return "Radial impulse field: KE budget split across fragments; launch direction = offset from blast centre."; }

    void load() override {
        const float energy = param("energy", 800.0f);
        const int   frags  = static_cast<int>(param("fragments", 40.0f));
        const glm::vec3 centre(0.0f, 4.0f, 0.0f);   // blast origin, up off the floor
        const float fragMass = 0.4f;
        const float fragR    = 0.22f;

        bodies.reserve(frags);

        // Distribute the total blast energy equally among the fragments, then
        // solve each fragment's speed from its share: 0.5*m*v^2 = E/frags.
        const float perKE = energy / static_cast<float>(frags);
        const float speed = std::sqrt(2.0f * perKE / fragMass);

        // Lay the fragments on a small spherical shell around the centre using a
        // deterministic Fibonacci-sphere distribution (even coverage, no RNG),
        // and launch each one straight out along its radial direction. Because
        // the directions are symmetric over the sphere the net momentum is ~0,
        // so the debris field expands in place rather than drifting.
        const float shell = 0.6f;                    // initial cluster radius
        const float golden = 3.14159265f * (3.0f - std::sqrt(5.0f));  // golden angle
        for (int i = 0; i < frags; ++i) {
            // Even points on a unit sphere.
            const float t  = (frags > 1) ? static_cast<float>(i) / static_cast<float>(frags - 1) : 0.0f;
            const float y  = 1.0f - 2.0f * t;                 // y from +1 to -1
            const float rr = std::sqrt(std::max(0.0f, 1.0f - y * y));
            const float ph = golden * static_cast<float>(i);
            const glm::vec3 dir(std::cos(ph) * rr, y, std::sin(ph) * rr);

            RigidBody b;
            b.position = centre + dir * shell;
            sceneSetSphereMass(b, fragMass, fragR);
            b.velocity = dir * speed;                          // radial launch
            b.friction = 0.4f;
            b.restitution = 0.2f;
            bodies.push_back(b);
        }
    }
};
