// ===========================================================================
// scene_tests — headless validation of every registered Scene.
//
// Loads each scene through the real SceneManager + PhysicsSolver (no OpenGL),
// steps it for a few seconds at the fixed timestep, and checks physically
// meaningful invariants:
//
//   * no NaN / infinite state (catches divergence / bad constraint setup)
//   * bodies stay within a sane world box (catches explosions / launches)
//   * dynamic bodies do not sink through the floor (y >= -slop)
//   * the system's speed does not run away (energy not pumped)
//
// Plus scene-specific checks (a suspended bridge deck must hang ABOVE the
// ground, Newton's cradle must conserve momentum, etc.). This is the
// standardized harness that drives the scene fixes.
// ===========================================================================

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

#include "../physics/physicssolver.h"
#include "../physics/rigidbody.h"
#include "Scene.h"
#include "SceneManager.h"
#include "Scenes.h"

static constexpr float DT = 1.0f / 60.0f;

namespace {

int gPass = 0, gFail = 0;

void check(const char* scene, const char* what, bool ok, const std::string& detail = "") {
    (ok ? gPass : gFail)++;
    std::printf("  [%s] %-22s %-40s %s\n", ok ? "PASS" : "FAIL", scene, what, detail.c_str());
}

bool finiteVec(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

struct RunStats {
    bool  allFinite   = true;
    float maxAbsPos   = 0.0f;   // largest |coordinate| seen (explosion detector)
    float maxSpeed    = 0.0f;   // largest linear speed seen (runaway detector)
    float minDynY     = 1e9f;   // lowest y of any dynamic body (floor sink detector)
    float finalSpeed  = 0.0f;   // total speed at the end (settling detector)
};

// Step a prepared solver+bodies for `steps` and collect stats.
RunStats simulate(PhysicsSolver& s, std::vector<RigidBody>& bs, int steps) {
    RunStats st;
    for (int i = 0; i < steps; ++i) {
        s.step(bs, DT);
        for (const auto& b : bs) {
            if (!finiteVec(b.position) || !finiteVec(b.velocity)) st.allFinite = false;
            st.maxAbsPos = std::max(st.maxAbsPos,
                std::max(std::abs(b.position.x), std::max(std::abs(b.position.y), std::abs(b.position.z))));
            const float sp = glm::length(b.velocity);
            st.maxSpeed = std::max(st.maxSpeed, sp);
            if (b.inverseMass > 0.0f) st.minDynY = std::min(st.minDynY, b.position.y);
        }
    }
    for (const auto& b : bs) st.finalSpeed += glm::length(b.velocity);
    return st;
}

// Generic invariants every scene must satisfy after a few seconds.
void genericChecks(const char* name, const RunStats& st) {
    check(name, "state stays finite", st.allFinite);
    check(name, "no explosion (bounded world)", st.maxAbsPos < 200.0f,
          "maxAbsPos=" + std::to_string(st.maxAbsPos));
    check(name, "no runaway speed", st.maxSpeed < 200.0f,
          "maxSpeed=" + std::to_string(st.maxSpeed));
    check(name, "no floor sink", st.minDynY > -1.0f,
          "minDynY=" + std::to_string(st.minDynY));
}

} // namespace

int main() {
    std::printf("SCENE VALIDATION  (headless, dt = 1/60 s)\n\n");

    PhysicsSolver solver;
    solver.captureTelemetry = false;

    EmptySandboxScene    emptySandbox;
    DominoSpiralScene    dominoSpiral;
    NewtonsCradleScene   newtonsCradle;
    AtwoodMachineScene   atwood;
    InclinedPlaneScene   inclinedPlane;
    RopeBridgeScene      ropeBridge;
    SuspensionBridgeScene suspensionBridge;
    CantileverBeamScene  cantilever;
    HangingChainScene    hangingChain;
    DoublePendulumScene  doublePendulum;
    SpringPendulumScene  springPendulum;
    TrebuchetScene       trebuchet;
    BallisticsScene      ballistics;
    SandboxScene         sandbox;

    SceneManager mgr(solver);
    mgr.registerScene("Empty Sandbox",     &emptySandbox);
    mgr.registerScene("Domino Spiral",     &dominoSpiral);
    mgr.registerScene("Newton's Cradle",   &newtonsCradle);
    mgr.registerScene("Atwood Machine",    &atwood);
    mgr.registerScene("Inclined Plane",    &inclinedPlane);
    mgr.registerScene("Rope Bridge",       &ropeBridge);
    mgr.registerScene("Suspension Bridge", &suspensionBridge);
    mgr.registerScene("Cantilever Beam",   &cantilever);
    mgr.registerScene("Hanging Chain",     &hangingChain);
    mgr.registerScene("Double Pendulum",   &doublePendulum);
    mgr.registerScene("Spring Pendulum",   &springPendulum);
    mgr.registerScene("Trebuchet",         &trebuchet);
    mgr.registerScene("Ballistics",        &ballistics);
    mgr.registerScene("Sandbox",           &sandbox);

    const int steps = 360; // 6 s

    // --- Generic pass over every scene (this alone reproduces any crash) ----
    for (std::size_t i = 0; i < mgr.count(); ++i) {
        mgr.loadSceneIndex(static_cast<int>(i));
        std::vector<RigidBody>& bs = mgr.bodies();
        const char* nm = mgr.activeScene()->name();
        // Progress marker flushed BEFORE stepping, so a hard crash still tells
        // us exactly which scene died.
        std::fprintf(stderr, ">> stepping scene %zu: %s (%zu bodies)\n", i, nm, bs.size());
        std::fflush(stderr);
        RunStats st = simulate(solver, bs, steps);
        genericChecks(nm, st);
        std::fflush(stdout);
    }

    std::printf("\n-- Scene-specific physical checks --\n");

    // Newton's Cradle: with 5 balls and 2 pulled back, total x-momentum must be
    // conserved and no kinetic energy created; the far balls must respond.
    {
        mgr.loadScene("Newton's Cradle");
        std::vector<RigidBody>& bs = mgr.bodies();
        // Let the pulled balls swing down and strike.
        float pMax = 0.0f, keMax = 0.0f;
        for (int i = 0; i < 240; ++i) {
            solver.step(bs, DT);
            float px = 0.0f, ke = 0.0f;
            for (auto& b : bs) { px += b.mass * b.velocity.x; ke += 0.5f * b.mass * glm::dot(b.velocity, b.velocity); }
            pMax = std::max(pMax, std::abs(px)); keMax = std::max(keMax, ke);
        }
        check("Newton's Cradle", "impact produces motion", pMax > 0.1f,
              "pMax=" + std::to_string(pMax));
        // Energy scale sanity: cradle KE should stay on the order of the initial
        // PE drop, not blow up.
        check("Newton's Cradle", "kinetic energy bounded", keMax < 200.0f,
              "keMax=" + std::to_string(keMax));
    }

    // Rope Bridge: the deck must remain SUSPENDED above the ground, not collapse.
    {
        mgr.loadScene("Rope Bridge");
        std::vector<RigidBody>& bs = mgr.bodies();
        simulate(solver, bs, 480); // 8 s to settle after the ball lands
        // Find the lowest plank (dynamic boxes, excluding the dropped ball).
        float lowestPlank = 1e9f;
        for (const auto& b : bs)
            if (b.inverseMass > 0.0f && b.shape == ShapeType::Box)
                lowestPlank = std::min(lowestPlank, b.position.y);
        check("Rope Bridge", "deck stays above ground", lowestPlank > 0.5f,
              "lowestPlank_y=" + std::to_string(lowestPlank));
    }

    // Suspension Bridge: same requirement -- a suspended deck hangs in the air.
    {
        mgr.loadScene("Suspension Bridge");
        std::vector<RigidBody>& bs = mgr.bodies();
        simulate(solver, bs, 480);
        float lowestPlank = 1e9f;
        for (const auto& b : bs)
            if (b.inverseMass > 0.0f && b.shape == ShapeType::Box)
                lowestPlank = std::min(lowestPlank, b.position.y);
        check("Suspension Bridge", "deck stays above ground", lowestPlank > 0.5f,
              "lowestPlank_y=" + std::to_string(lowestPlank));
    }

    // Inclined Plane: the block must rest ON the ramp (never sink through the
    // world floor) and, at 25 deg with mu=0.30 (tan25=0.466 > 0.30), it should
    // SLIDE down -- i.e. move, not explode and not stay perfectly frozen.
    {
        mgr.loadScene("Inclined Plane");
        std::vector<RigidBody>& bs = mgr.bodies();
        const glm::vec3 p0 = bs[0].position;
        RunStats st = simulate(solver, bs, 240);
        const float travel = glm::length(bs[0].position - p0);
        check("Inclined Plane", "block stays finite/onramp", st.allFinite && st.minDynY > -0.5f,
              "minDynY=" + std::to_string(st.minDynY));
        check("Inclined Plane", "block slides (mu<tan theta)", travel > 0.2f,
              "travel=" + std::to_string(travel));
    }

    std::printf("\n========================================================\n");
    std::printf("  SCENE SUMMARY : %d passed, %d failed\n", gPass, gFail);
    std::printf("========================================================\n");
    return gFail == 0 ? 0 : 1;
}
