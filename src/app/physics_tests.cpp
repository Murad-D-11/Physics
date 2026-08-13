// ===========================================================================
// Headless resting-contact audit harness (no OpenGL).
//
// Implements the controlled experiments from the resting-contact audit:
//   Test A  single domino -> floor (falls, must rest flat and go still)
//   Test C1 one already-horizontal domino on the floor (must not move)
//   Test C2 two stacked horizontal dominoes (must not drift)
//   Test C3 two crossed horizontal dominoes (angled contact region)
//   Test B  domino tipped onto a stationary domino (must settle)
//
// For each test we step the *real* PhysicsSolver at a fixed 1/60 s and track:
//   total kinetic energy (linear + rotational)
//   total potential energy
//   max linear / angular speed over all dynamic bodies
//   per-body centre-of-mass horizontal drift from the start
//   contact count and, for the first dynamic-dynamic contact, its normal,
//   penetration, accumulated normal impulse and accumulated friction impulse
//
// The point is to locate *where* residual motion comes from, using numbers,
// before changing the solver.
// ===========================================================================

#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../physics/rigidbody.h"
#include "../physics/physicssolver.h"

static constexpr float FIXED_DT = 1.0f / 60.0f;
static constexpr float G = 9.81f;

// Domino material identical to spawnDominoSpiral().
static const glm::vec3 DOMINO_SCALE(0.15f, 0.9f, 0.45f);

static void setMass(RigidBody& b, float mass) {
    b.mass = mass;
    b.inverseMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    b.updateInertiaTensor();
}

static RigidBody makeDomino(const glm::vec3& pos, const glm::quat& orient) {
    RigidBody d;
    d.scale = DOMINO_SCALE;
    d.position = pos;
    d.orientation = orient;
    d.velocity = glm::vec3(0.0f);
    d.angularVelocity = glm::vec3(0.0f);
    setMass(d, 0.5f);
    d.restitution = 0.03f;
    d.friction = 0.5f;
    return d;
}

// Rotational KE = 0.5 * w^T (R I_local R^T) w
static float rotationalKE(const RigidBody& b) {
    if (b.inverseMass == 0.0f) return 0.0f;
    const glm::mat3 R = glm::mat3_cast(b.orientation);
    const glm::mat3 Iworld = R * b.inertiaLocal * glm::transpose(R);
    const glm::vec3 w = b.angularVelocity;
    return 0.5f * glm::dot(w, Iworld * w);
}

struct Metrics {
    float ke = 0.0f;
    float pe = 0.0f;
    float maxLin = 0.0f;
    float maxAng = 0.0f;
    int contacts = 0;
};

static Metrics measure(const std::vector<RigidBody>& bodies, const PhysicsSolver& solver) {
    Metrics m;
    for (const auto& b : bodies) {
        if (b.inverseMass == 0.0f) continue;
        const float lin = glm::length(b.velocity);
        const float ang = glm::length(b.angularVelocity);
        m.maxLin = std::max(m.maxLin, lin);
        m.maxAng = std::max(m.maxAng, ang);
        m.ke += 0.5f * b.mass * glm::dot(b.velocity, b.velocity) + rotationalKE(b);
        m.pe += b.mass * G * b.position.y;
    }
    m.contacts = solver.lastContactCount;
    return m;
}

// Largest positive penetration currently reported by the solver (residual
// penetration across all contacts, floor + dynamic).
static float maxPenetration(const PhysicsSolver& solver) {
    float p = 0.0f;
    for (const auto& c : solver.lastSolvedContacts) p = std::max(p, c.penetration);
    return p;
}

// Angle (radians) between a body's current orientation and its start orientation.
static float orientationDrift(const glm::quat& start, const glm::quat& now) {
    float d = std::fabs(glm::dot(glm::normalize(start), glm::normalize(now)));
    d = std::min(1.0f, d);
    return 2.0f * std::acos(d);
}

static void printContacts(PhysicsSolver& solver, int maxRows) {
    int rows = 0;
    for (const auto& c : solver.lastSolvedContacts) {
        if (c.floorContact) continue; // focus on domino-domino contacts
        std::printf("      contact n=(% .3f,% .3f,% .3f) pen=% .4f  Jn=%.4f  Jt=%.4f\n",
                    c.normal.x, c.normal.y, c.normal.z, c.penetration,
                    c.normalImpulse, c.frictionImpulse);
        if (++rows >= maxRows) break;
    }
    if (rows == 0) std::printf("      (no dynamic-dynamic contacts this step)\n");
}

static void runTest(const std::string& name,
                    std::vector<RigidBody> bodies,
                    int steps,
                    int reportEvery,
                    bool disableSleep = false) {
    std::printf("\n================ %s ================\n", name.c_str());

    PhysicsSolver solver;
    solver.captureDiagnostics = true;
    if (disableSleep) {
        solver.sleepingEnabled = false;
        std::printf("  (sleeping DISABLED -- auditing raw solver stability)\n");
    }

    std::vector<glm::vec3> startPos(bodies.size());
    std::vector<glm::quat> startRot(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        startPos[i] = bodies[i].position;
        startRot[i] = bodies[i].orientation;
    }

    for (int s = 0; s <= steps; ++s) {
        if (s > 0) solver.step(bodies, FIXED_DT);

        if (s % reportEvery == 0 || s == steps) {
            const Metrics m = measure(bodies, solver);

            float maxDrift = 0.0f;
            int driftBody = -1;
            float maxRot = 0.0f;
            for (std::size_t i = 0; i < bodies.size(); ++i) {
                if (bodies[i].inverseMass == 0.0f) continue;
                const glm::vec3 d = bodies[i].position - startPos[i];
                const float horiz = std::sqrt(d.x * d.x + d.z * d.z);
                if (horiz > maxDrift) { maxDrift = horiz; driftBody = static_cast<int>(i); }
                maxRot = std::max(maxRot, orientationDrift(startRot[i], bodies[i].orientation));
            }

            std::printf("t=%7.3f  KE=%.6f  maxV=%.5f maxW=%.5f  contacts=%d  "
                        "maxPen=%.5f  horizDrift=%.5f(b%d)  rotDrift=%.5f rad\n",
                        s * FIXED_DT, m.ke, m.maxLin, m.maxAng, m.contacts,
                        maxPenetration(solver), maxDrift, driftBody, maxRot);
            if (s % (reportEvery * 5) == 0 || s == steps) {
                printContacts(solver, 4);
            }
        }
    }

    int awake = 0;
    for (const auto& b : bodies) if (b.inverseMass > 0.0f && !b.asleep) ++awake;
    std::printf("  final: awake=%d/%zu\n", awake, bodies.size());
}

int main() {
    // Orientation that lays a domino flat: rotate -90 deg about Z so the long
    // local +y axis (height 0.9) points along world +x. Resting thickness is
    // then 0.15 (local x) vertical -> top face at y = 0.075.
    const glm::quat lying = glm::angleAxis(glm::radians(-90.0f), glm::vec3(0, 0, 1));
    const float lyingHalfThick = DOMINO_SCALE.x * 0.5f; // 0.075

    // ---- Test A: single upright domino, nudged past balance, falls to floor.
    {
        std::vector<RigidBody> bodies;
        RigidBody d = makeDomino(glm::vec3(0.0f, DOMINO_SCALE.y * 0.5f, 0.0f), glm::quat(1, 0, 0, 0));
        d.angularVelocity = glm::vec3(0.0f, 0.0f, -3.0f); // tip about z
        bodies.push_back(d);
        runTest("Test A: single domino -> floor", bodies, 480, 30);
    }

    // ---- Test C1: one already-horizontal domino resting on the floor.
    {
        std::vector<RigidBody> bodies;
        bodies.push_back(makeDomino(glm::vec3(0.0f, lyingHalfThick, 0.0f), lying));
        runTest("Test C1: single horizontal domino at rest", bodies, 300, 30);
    }

    // ---- Test C2: two horizontal dominoes stacked (top resting on bottom).
    {
        std::vector<RigidBody> bodies;
        bodies.push_back(makeDomino(glm::vec3(0.0f, lyingHalfThick, 0.0f), lying));
        bodies.push_back(makeDomino(glm::vec3(0.0f, lyingHalfThick + DOMINO_SCALE.x, 0.0f), lying));
        runTest("Test C2: two stacked horizontal dominoes", bodies, 300, 30);
    }

    // ---- Test C3: two horizontal dominoes, top crossed 40 deg about vertical.
    {
        std::vector<RigidBody> bodies;
        bodies.push_back(makeDomino(glm::vec3(0.0f, lyingHalfThick, 0.0f), lying));
        const glm::quat crossed = glm::angleAxis(glm::radians(40.0f), glm::vec3(0, 1, 0)) * lying;
        bodies.push_back(makeDomino(glm::vec3(0.0f, lyingHalfThick + DOMINO_SCALE.x, 0.0f), crossed));
        runTest("Test C3: two crossed horizontal dominoes", bodies, 300, 30);
    }

    // ---- Test B: domino A tipped 25 deg falls onto stationary domino B.
    {
        std::vector<RigidBody> bodies;
        // B lying flat.
        bodies.push_back(makeDomino(glm::vec3(0.45f, lyingHalfThick, 0.0f), lying));
        // A upright but tilted 25 deg toward B, base near B's near edge.
        const glm::quat tilt = glm::angleAxis(glm::radians(-25.0f), glm::vec3(0, 0, 1));
        RigidBody a = makeDomino(glm::vec3(-0.2f, DOMINO_SCALE.y * 0.5f, 0.0f), tilt);
        bodies.push_back(a);
        runTest("Test B: domino tipped onto stationary domino", bodies, 600, 30);
    }

    // ---- Test F: straight chain of N upright dominoes, first one kicked.
    // Reproduces the spiral's many-body chain: after the cascade finishes the
    // whole row should reach rest. Residual KE / drift here == the reported bug.
    {
        std::vector<RigidBody> bodies;
        const int N = 30;
        const float spacing = 0.45f; // matches spawnDominoSpiral()
        for (int i = 0; i < N; ++i) {
            RigidBody d = makeDomino(glm::vec3(i * spacing, DOMINO_SCALE.y * 0.5f, 0.0f),
                                     glm::quat(1, 0, 0, 0));
            bodies.push_back(d);
        }
        bodies[0].angularVelocity = glm::vec3(0.0f, 0.0f, -3.0f); // topple toward +x
        runTest("Test F: 30-domino straight chain topple", bodies, 900, 30);
    }

    // ---- Test G: the actual 150-domino Archimedean spiral (mirrors
    // spawnDominoSpiral() exactly). This is the scene that misbehaves.
    {
        std::vector<RigidBody> bodies;
        const int count = 150;
        const float halfHeight = DOMINO_SCALE.y * 0.5f;
        const float halfThick  = DOMINO_SCALE.x * 0.5f;
        const float spacing = 0.45f;
        const float r0 = 1.5f;
        const float b  = 0.18f;
        const glm::vec3 up(0.0f, 1.0f, 0.0f);

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
            RigidBody d = makeDomino(pos[i], glm::angleAxis(std::atan2(-tangent.z, tangent.x), up));
            bodies.push_back(d);
        }
        const glm::vec3 tangent0 = glm::normalize(pos[1] - pos[0]);
        const glm::vec3 tiltAxis = glm::normalize(glm::cross(up, tangent0));
        const float tiltAngle = glm::radians(14.0f);
        bodies[0].orientation = glm::angleAxis(tiltAngle, tiltAxis) * bodies[0].orientation;
        bodies[0].position.y = halfHeight * std::cos(tiltAngle) + halfThick * std::sin(tiltAngle);
        bodies[0].angularVelocity = tiltAxis * 3.0f;

        runTest("Test G: 150-domino Archimedean spiral", bodies, 4200, 120);
    }

    // ---- Test H: tall vertical stack of unit cubes, SLEEPING DISABLED, run for
    // thousands of steps. This is the constraint-stabilization stress test:
    // residual penetration must stay bounded (~slop, not growing), and both
    // horizontal drift and rotational drift must stay ~0 over the whole run.
    auto makeCube = [](const glm::vec3& pos, float mass) {
        RigidBody c;
        c.scale = glm::vec3(1.0f);
        c.position = pos;
        c.orientation = glm::quat(1, 0, 0, 0);
        c.velocity = glm::vec3(0.0f);
        c.angularVelocity = glm::vec3(0.0f);
        c.mass = mass;
        c.inverseMass = 1.0f / mass;
        c.updateInertiaTensor();
        c.restitution = 0.1f;
        c.friction = 0.6f;
        return c;
    };

    {
        std::vector<RigidBody> bodies;
        const int H = 8;
        for (int i = 0; i < H; ++i)
            bodies.push_back(makeCube(glm::vec3(0.0f, 0.5f + i * 1.0f, 0.0f), 1.0f));
        runTest("Test H: 8-cube tower, no sleep (fine timeline)", bodies, 1800, 30, true);
    }

    // ---- Test H2/H3: shorter towers, no sleep -- is collapse height-dependent?
    {
        std::vector<RigidBody> bodies;
        for (int i = 0; i < 3; ++i)
            bodies.push_back(makeCube(glm::vec3(0.0f, 0.5f + i * 1.0f, 0.0f), 1.0f));
        runTest("Test H2: 3-cube tower, no sleep", bodies, 1800, 60, true);
    }
    {
        std::vector<RigidBody> bodies;
        for (int i = 0; i < 5; ++i)
            bodies.push_back(makeCube(glm::vec3(0.0f, 0.5f + i * 1.0f, 0.0f), 1.0f));
        runTest("Test H3: 5-cube tower, no sleep", bodies, 1800, 60, true);
    }

    // ---- Test I: same tower but WITH sleeping (production behaviour).
    {
        std::vector<RigidBody> bodies;
        const int H = 8;
        for (int i = 0; i < H; ++i)
            bodies.push_back(makeCube(glm::vec3(0.0f, 0.5f + i * 1.0f, 0.0f), 1.0f));
        runTest("Test I: 8-cube tower, 6000 steps, with sleep", bodies, 6000, 600, false);
    }

    return 0;
}
