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
#include <algorithm>

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
    HangingChainScene    hangingChain;
    DoublePendulumScene  doublePendulum;
    SpringPendulumScene  springPendulum;
    TrebuchetScene       trebuchet;
    BallisticsScene      ballistics;
    HangingChainWaveScene  hangingChainWave;
    ObjectVolumeScene      objectVolume;
    CableStayedBridgeScene cableStayedBridge;
    ExplosionScene         explosion;
    BoulderCastleScene     boulderCastle;
    SandboxScene         sandbox;

    SceneManager mgr(solver);
    mgr.registerScene("Empty Sandbox",     &emptySandbox);
    mgr.registerScene("Domino Spiral",     &dominoSpiral);
    mgr.registerScene("Newton's Cradle",   &newtonsCradle);
    mgr.registerScene("Atwood Machine",    &atwood);
    mgr.registerScene("Inclined Plane",    &inclinedPlane);
    mgr.registerScene("Rope Bridge",       &ropeBridge);
    mgr.registerScene("Suspension Bridge", &suspensionBridge);
    mgr.registerScene("Hanging Chain",     &hangingChain);
    mgr.registerScene("Double Pendulum",   &doublePendulum);
    mgr.registerScene("Spring Laboratory", &springPendulum);
    mgr.registerScene("Trebuchet",         &trebuchet);
    mgr.registerScene("Ballistics",        &ballistics);
    mgr.registerScene("Hanging Chain Wave", &hangingChainWave);
    mgr.registerScene("Object Volume",      &objectVolume);
    mgr.registerScene("Cable-Stayed Bridge",&cableStayedBridge);
    mgr.registerScene("Explosion",          &explosion);
    mgr.registerScene("Boulder vs Castle",  &boulderCastle);
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

    std::printf("\n-- Per-scene behavioural checks (real path vs intended) --\n");

    // Helpers for behavioural measurement. These read the live bodies before
    // and after a run and quantify the INTENDED behaviour, so a scene that only
    // "doesn't crash" is not enough -- it must actually do the physics.
    auto loadFresh = [&](const char* nm) -> std::vector<RigidBody>& {
        mgr.loadScene(nm);
        return mgr.bodies();
    };
    // Total signed x-momentum / kinetic energy of the current world.
    auto momentumX = [](const std::vector<RigidBody>& bs) {
        float p = 0.0f; for (auto& b : bs) if (b.inverseMass > 0.0f) p += b.mass * b.velocity.x; return p;
    };
    auto kinetic = [](const std::vector<RigidBody>& bs) {
        float k = 0.0f; for (auto& b : bs) if (b.inverseMass > 0.0f) k += 0.5f * b.mass * glm::dot(b.velocity, b.velocity); return k;
    };
    // How far a body's local +X axis has tilted from world +X (radians): a
    // measure of "did this plank stay level / did this domino topple".
    auto tiltFromX = [](const RigidBody& b) {
        const glm::vec3 ax = b.orientation * glm::vec3(1, 0, 0);
        return std::acos(std::clamp(ax.x / std::max(glm::length(ax), 1e-6f), -1.0f, 1.0f));
    };
    // Lowest dynamic BOX (deck plank) y, ignoring spheres (the load ball).
    auto lowestBoxY = [](const std::vector<RigidBody>& bs) {
        float y = 1e9f; for (auto& b : bs) if (b.inverseMass > 0.0f && b.shape == ShapeType::Box) y = std::min(y, b.position.y); return y;
    };
    // Largest deck-plank tilt in the scene (dynamic boxes only).
    auto maxBoxTilt = [&](const std::vector<RigidBody>& bs) {
        float t = 0.0f; for (auto& b : bs) if (b.inverseMass > 0.0f && b.shape == ShapeType::Box) t = std::max(t, tiltFromX(b)); return t;
    };

    // ---- 1. Domino Spiral: the first domino must topple and the cascade must
    //         propagate to distant dominoes (not just the first few). ----
    {
        auto& bs = loadFresh("Domino Spiral");
        const int n = static_cast<int>(bs.size());
        std::vector<glm::vec3> p0(n); for (int i = 0; i < n; ++i) p0[i] = bs[i].position;
        simulate(solver, bs, 600); // 10 s
        int toppled = 0; for (int i = 0; i < n; ++i) if (tiltFromX(bs[i]) > 0.5f) ++toppled;
        // The far quarter of the run should have been reached by the cascade.
        int farToppled = 0; for (int i = 3 * n / 4; i < n; ++i) if (tiltFromX(bs[i]) > 0.5f) ++farToppled;
        check("Domino Spiral", "cascade topples many", toppled > n / 3,
              "toppled=" + std::to_string(toppled) + "/" + std::to_string(n));
        check("Domino Spiral", "cascade reaches far end", farToppled > 0,
              "farToppled=" + std::to_string(farToppled));
    }

    // ---- 2. Atwood Machine: heavy (idx0) descends, light (idx1) rises by a
    //         similar amount, and motion is essentially VERTICAL (small |dx|),
    //         now that each mass rides in a vertical guide shaft. --
    {
        auto& bs = loadFresh("Atwood Machine");
        const glm::vec3 heavy0 = bs[0].position, light0 = bs[1].position;
        simulate(solver, bs, 180); // 3 s
        const float heavyDrop = heavy0.y - bs[0].position.y;   // + = descended
        const float lightRise = bs[1].position.y - light0.y;   // + = rose
        const float heavyDX = std::abs(bs[0].position.x - heavy0.x);
        const float lightDX = std::abs(bs[1].position.x - light0.x);
        check("Atwood Machine", "heavy descends", heavyDrop > 0.3f,
              "drop=" + std::to_string(heavyDrop));
        check("Atwood Machine", "light rises ~= heavy drop", lightRise > 0.3f && std::abs(lightRise - heavyDrop) < 0.6f,
              "rise=" + std::to_string(lightRise));
        // The guide shafts now hold each mass to its column, so lateral drift
        // must be SMALL in absolute terms (the walls physically prevent the
        // sideways slide that a bare point-pulley produced).
        check("Atwood Machine", "guide shafts keep motion vertical", heavyDX < 0.25f && lightDX < 0.25f,
              "dxH=" + std::to_string(heavyDX) + " dxL=" + std::to_string(lightDX));
    }

    // ---- 3. Newton's Cradle (museum-quality): the bifilar (dual-rope)
    //         suspension must keep every bead in ONE vertical swing plane, so
    //         there is essentially NO lateral (Z) drift and no axial spin. An
    //         impact must transmit motion down the line and eject the far bead,
    //         and energy must stay bounded (no pump). Only the beads (spheres)
    //         are measured; the steel frame is static. ----
    {
        auto& bs = loadFresh("Newton's Cradle");
        // Record the beads' rest Z (spheres) to measure lateral deviation.
        std::vector<float> z0;
        for (auto& b : bs) if (b.inverseMass > 0.0f && b.shape == ShapeType::Sphere) z0.push_back(b.position.z);
        float keMax = 0.0f, speedMax = 0.0f, maxLateral = 0.0f, maxSpin = 0.0f, farSpeedMax = 0.0f;
        // The far bead is the last sphere in the row; the struck (raised) bead
        // is the first sphere. For a clean 0->100->0 transfer we compare the
        // speed the far bead reaches on ejection with the speed the first bead
        // carried on its way in.
        int firstSphere = -1, lastSphere = -1;
        for (int i = 0; i < static_cast<int>(bs.size()); ++i)
            if (bs[i].inverseMass > 0.0f && bs[i].shape == ShapeType::Sphere) {
                if (firstSphere < 0) firstSphere = i;
                lastSphere = i;
            }
        float firstIncomingMax = 0.0f;  // peak speed of the raised bead before impact
        bool  impacted = false;
        for (int i = 0; i < 360; ++i) {
            solver.step(bs, DT);
            keMax = std::max(keMax, kinetic(bs));
            // Before the far bead has moved, track the incoming bead's top speed.
            if (!impacted && lastSphere >= 0 && glm::length(bs[lastSphere].velocity) > 0.3f)
                impacted = true;
            if (!impacted && firstSphere >= 0)
                firstIncomingMax = std::max(firstIncomingMax, glm::length(bs[firstSphere].velocity));
            int zi = 0;
            for (auto& b : bs) {
                if (b.inverseMass > 0.0f && b.shape == ShapeType::Sphere) {
                    speedMax = std::max(speedMax, glm::length(b.velocity));
                    maxLateral = std::max(maxLateral, std::abs(b.position.z - z0[zi]));
                    maxSpin = std::max(maxSpin, glm::length(b.angularVelocity));
                    ++zi;
                }
            }
            if (lastSphere >= 0) farSpeedMax = std::max(farSpeedMax, glm::length(bs[lastSphere].velocity));
        }
        check("Newton's Cradle", "impact ejects the far bead", farSpeedMax > 0.5f,
              "farBeadSpeedMax=" + std::to_string(farSpeedMax));
        // Momentum transfers through the line with little loss: the far bead
        // ejects at a large fraction of the speed the struck bead came in with
        // (equal masses => ideally 100%). Require >= 70% to allow for solver
        // damping while still proving a clean, efficient transfer.
        const float transferRatio = (firstIncomingMax > 0.1f) ? (farSpeedMax / firstIncomingMax) : 0.0f;
        check("Newton's Cradle", "momentum transfers cleanly through the line", transferRatio > 0.7f,
              "farSpeed=" + std::to_string(farSpeedMax) + " inSpeed=" + std::to_string(firstIncomingMax) +
              " ratio=" + std::to_string(transferRatio));
        check("Newton's Cradle", "no lateral drift (single plane)", maxLateral < 0.2f,
              "maxLateralZ=" + std::to_string(maxLateral));
        check("Newton's Cradle", "no axial spin", maxSpin < 3.0f,
              "maxSpin=" + std::to_string(maxSpin));
        check("Newton's Cradle", "kinetic energy bounded (no pump)", keMax < 200.0f,
              "keMax=" + std::to_string(keMax));
    }

    // ---- 4. Inclined Plane: low-friction block slides FURTHER than the
    //         high-friction block; the rolling ball actually rolls (spins). ----
    {
        auto& bs = loadFresh("Inclined Plane");
        // Bodies order (see scene): [0]=mu0.05 block, [1]=mu0.30, [2]=mu0.70, [3]=ball.
        const glm::vec3 p0lo = bs[0].position, p0hi = bs[2].position, p0ball = bs[3].position;
        RunStats st = simulate(solver, bs, 200);
        const float travelLo = glm::length(bs[0].position - p0lo);
        const float travelHi = glm::length(bs[2].position - p0hi);
        const float ballTravel = glm::length(bs[3].position - p0ball);
        const float ballSpin = glm::length(bs[3].angularVelocity);
        check("Inclined Plane", "stays on ramp (no sink)", st.minDynY > -0.5f,
              "minDynY=" + std::to_string(st.minDynY));
        check("Inclined Plane", "low-mu slides further than high-mu", travelLo > travelHi + 0.2f,
              "lo=" + std::to_string(travelLo) + " hi=" + std::to_string(travelHi));
        check("Inclined Plane", "ball rolls (spins) while descending", ballTravel > 0.3f && ballSpin > 0.5f,
              "ballTravel=" + std::to_string(ballTravel) + " spin=" + std::to_string(ballSpin));
    }

    // ---- 5. Rope Bridge: deck stays suspended, planks stay ~level (small
    //         tilt), the ends stay put, and the load ball rests on the deck. ----
    {
        auto& bs = loadFresh("Rope Bridge");
        simulate(solver, bs, 600); // 10 s to settle
        const float lowPlank = lowestBoxY(bs);
        const float tilt = maxBoxTilt(bs);
        const RigidBody& ball = bs.back();
        check("Rope Bridge", "deck stays suspended", lowPlank > 1.0f,
              "lowPlank_y=" + std::to_string(lowPlank));
        check("Rope Bridge", "planks stay ~level (no spin)", tilt < 0.9f,
              "maxTilt_rad=" + std::to_string(tilt));
        check("Rope Bridge", "load ball rests on deck", ball.position.y > lowPlank - 0.2f && std::abs(ball.position.x) < 2.0f,
              "ball_y=" + std::to_string(ball.position.y) + " ball_x=" + std::to_string(ball.position.x));
    }

    // ---- 6. Suspension Bridge: same suspended-deck + level + ball-on-deck. ----
    {
        auto& bs = loadFresh("Suspension Bridge");
        simulate(solver, bs, 600);
        const float lowPlank = lowestBoxY(bs);
        const float tilt = maxBoxTilt(bs);
        const RigidBody& ball = bs.back();
        check("Suspension Bridge", "deck stays suspended", lowPlank > 1.0f,
              "lowPlank_y=" + std::to_string(lowPlank));
        check("Suspension Bridge", "planks stay ~level (no spin)", tilt < 0.9f,
              "maxTilt_rad=" + std::to_string(tilt));
        check("Suspension Bridge", "load ball rests on deck", ball.position.y > lowPlank - 0.3f && std::abs(ball.position.x) < 2.5f,
              "ball_y=" + std::to_string(ball.position.y) + " ball_x=" + std::to_string(ball.position.x));
    }

    // ---- Cantilever Beam REMOVED (see Scenes.h): the solver has no bending-
    //      stiffness primitive, so an articulated beam could not be made to
    //      droop elegantly without collapsing to the floor. Scene deleted by
    //      request rather than shipped visibly broken. ----

    // ---- 8. Hanging Chain: settles into a catenary -- the middle link hangs
    //         BELOW the end links. ----
    {
        auto& bs = loadFresh("Hanging Chain");
        simulate(solver, bs, 480);
        const int n = static_cast<int>(bs.size());
        const float endY = 0.5f * (bs[0].position.y + bs[n - 1].position.y);
        const float midY = bs[n / 2].position.y;
        check("Hanging Chain", "middle sags below ends (catenary)", midY < endY - 0.3f,
              "midY=" + std::to_string(midY) + " endY=" + std::to_string(endY));
    }

    // ---- 9. Double Pendulum: it actually swings (the tip moves a lot) and
    //         energy stays bounded (no blow-up). ----
    {
        auto& bs = loadFresh("Double Pendulum");
        const glm::vec3 tip0 = bs[1].position;
        float tipTravel = 0.0f, keMax = 0.0f;
        for (int i = 0; i < 360; ++i) {
            solver.step(bs, DT);
            tipTravel = std::max(tipTravel, glm::length(bs[1].position - tip0));
            keMax = std::max(keMax, kinetic(bs));
        }
        check("Double Pendulum", "arm swings (tip moves)", tipTravel > 1.0f,
              "tipTravel=" + std::to_string(tipTravel));
        check("Double Pendulum", "energy bounded", keMax < 500.0f,
              "keMax=" + std::to_string(keMax));
    }

    // ---- 10. Spring Laboratory: the vertical oscillator (0), horizontal
    //          oscillator (1) and coupled pair (2,3) each oscillate about
    //          equilibrium with bounded amplitude (Hooke's-law SHM; no runaway,
    //          no collapse). We track the min/max of each mass over the run and
    //          require a real swing that stays bounded. ----
    {
        auto& bs = loadFresh("Spring Laboratory");
        glm::vec3 vMin(1e9f), vMax(-1e9f), hMin(1e9f), hMax(-1e9f);
        float cMin = 1e9f, cMax = -1e9f;   // coupled mass-0 x range
        RunStats st;
        for (int i = 0; i < 360; ++i) {
            solver.step(bs, DT);
            vMin = glm::min(vMin, bs[0].position); vMax = glm::max(vMax, bs[0].position);
            hMin = glm::min(hMin, bs[1].position); hMax = glm::max(hMax, bs[1].position);
            cMin = std::min(cMin, bs[2].position.x); cMax = std::max(cMax, bs[2].position.x);
            for (auto& b : bs) if (b.inverseMass > 0.0f) st.maxAbsPos = std::max(st.maxAbsPos, std::abs(b.position.y));
        }
        const float vAmpY = vMax.y - vMin.y;      // vertical oscillator swings in Y
        const float hAmpX = hMax.x - hMin.x;      // horizontal oscillator swings in X
        const float cAmpX = cMax - cMin;          // coupled mass swings in X
        check("Spring Laboratory", "vertical oscillator oscillates in Y", vAmpY > 0.4f,
              "ampY=" + std::to_string(vAmpY));
        check("Spring Laboratory", "horizontal oscillator oscillates in X", hAmpX > 0.4f,
              "ampX=" + std::to_string(hAmpX));
        check("Spring Laboratory", "coupled pair oscillates", cAmpX > 0.3f,
              "coupledAmpX=" + std::to_string(cAmpX));
        check("Spring Laboratory", "all stay bounded (no runaway)", st.maxAbsPos < 40.0f,
              "maxAbsY=" + std::to_string(st.maxAbsPos));
    }

    // ---- 11. Trebuchet: the projectile (last body) is flung -- it gains speed
    //          and moves away from its start. ----
    {
        auto& bs = loadFresh("Trebuchet");
        const glm::vec3 proj0 = bs.back().position;
        float projSpeedMax = 0.0f;
        for (int i = 0; i < 240; ++i) { solver.step(bs, DT); projSpeedMax = std::max(projSpeedMax, glm::length(bs.back().velocity)); }
        const float projTravel = glm::length(bs.back().position - proj0);
        check("Trebuchet", "projectile is launched (speed)", projSpeedMax > 2.0f,
              "projSpeedMax=" + std::to_string(projSpeedMax));
        check("Trebuchet", "projectile travels", projTravel > 1.0f,
              "projTravel=" + std::to_string(projTravel));
    }

    // ---- 12. Ballistics: all shots launched at equal speed; the 45-degree
    //          (middle) shot should reach the greatest BALLISTIC range. We
    //          measure the horizontal distance at the moment each projectile
    //          first falls back to its launch height -- the true range R =
    //          v^2 sin(2theta)/g -- NOT cumulative x (which would include
    //          post-landing skidding along the ground and favour flat shots). --
    {
        auto& bs = loadFresh("Ballistics");
        // Projectiles are the DYNAMIC SPHERES (barrels + markers are static
        // boxes). Collect them in registration order = ascending launch angle.
        std::vector<int> shot;
        for (int i = 0; i < static_cast<int>(bs.size()); ++i)
            if (bs[i].inverseMass > 0.0f && bs[i].shape == ShapeType::Sphere) shot.push_back(i);
        const int ns = static_cast<int>(shot.size());
        std::vector<float> x0(ns), y0(ns), range(ns, 0.0f); std::vector<bool> landed(ns, false);
        // Launched from the muzzle -> starts above the floor, not on it.
        bool muzzleLaunch = true;
        for (int k = 0; k < ns; ++k) {
            x0[k] = bs[shot[k]].position.x; y0[k] = bs[shot[k]].position.y;
            if (y0[k] < 0.5f) muzzleLaunch = false;
        }
        for (int i = 0; i < 300; ++i) {
            for (int k = 0; k < ns; ++k) {
                if (!landed[k] && bs[shot[k]].velocity.y < 0.0f && bs[shot[k]].position.y <= y0[k] + 0.05f) {
                    range[k] = bs[shot[k]].position.x - x0[k];
                    landed[k] = true;
                }
            }
            solver.step(bs, DT);
        }
        for (int k = 0; k < ns; ++k) if (!landed[k]) range[k] = bs[shot[k]].position.x - x0[k];
        int best = 0; for (int k = 1; k < ns; ++k) if (range[k] > range[best]) best = k;
        check("Ballistics", "projectiles launch from the muzzle (not the floor)", muzzleLaunch,
              "minStartY_ok=" + std::to_string(muzzleLaunch));
        check("Ballistics", "45deg (middle) shot has max range in vacuum", std::abs(best - ns / 2) <= 1,
              "best=" + std::to_string(best) + " mid=" + std::to_string(ns / 2));
    }

    // ---- Hanging Chain Wave: a flat-on-the-ground chain flicked sideways at
    //          one end -- the transverse pulse must reach the centre bead. ----
    {
        auto& bs = loadFresh("Hanging Chain Wave");
        const int n = static_cast<int>(bs.size());
        // The chain lies flat on the ground; the far (left) end is flicked
        // SIDEWAYS (+Z). The transverse pulse must travel along the chain and
        // laterally displace the CENTRE bead (which starts at rest, z=0).
        const int centreNode = n / 2;
        const float z0 = bs[centreNode].position.z;
        float centreMove = 0.0f;
        for (int i = 0; i < 300; ++i) {
            solver.step(bs, DT);
            centreMove = std::max(centreMove, std::abs(bs[centreNode].position.z - z0));
        }
        check("Hanging Chain Wave", "sideways wave reaches the centre", centreMove > 0.1f,
              "centreLateralMove=" + std::to_string(centreMove));
    }

    // ---- 14. Object Volume (many-body stress test): a dense pile of boxes and
    //          spheres dropped into a walled bin must SETTLE -- every body
    //          resolves its motion from real contacts, then the whole field
    //          comes to rest without exploding, sinking through the floor, or
    //          escaping the bin. ----
    {
        auto& bs = loadFresh("Object Volume");
        const int n = static_cast<int>(bs.size());
        int dynCount = 0; for (auto& b : bs) if (b.inverseMass > 0.0f) ++dynCount;
        simulate(solver, bs, 600); // 10 s to settle
        // After settling: bounded speed (came to rest), no floor sink, contained
        // in the bin (no lateral escape).
        float maxSpeed = 0.0f, minY = 1e9f, maxAbsXZ = 0.0f;
        for (auto& b : bs) {
            if (b.inverseMass <= 0.0f) continue;
            maxSpeed = std::max(maxSpeed, glm::length(b.velocity));
            minY = std::min(minY, b.position.y);
            maxAbsXZ = std::max(maxAbsXZ, std::max(std::abs(b.position.x), std::abs(b.position.z)));
        }
        check("Object Volume", "many dynamic bodies present", dynCount >= 20,
              "dynBodies=" + std::to_string(dynCount) + "/" + std::to_string(n));
        check("Object Volume", "pile settles (bounded speed)", maxSpeed < 2.0f,
              "maxSpeed=" + std::to_string(maxSpeed));
        check("Object Volume", "no body sinks through the floor", minY > -0.5f,
              "minY=" + std::to_string(minY));
        check("Object Volume", "stays contained in the bin", maxAbsXZ < 12.0f,
              "maxAbsXZ=" + std::to_string(maxAbsXZ));
    }

    // ---- 15. Explosion: a central cluster is blown RADIALLY outward. Each
    //          fragment must move away from the blast centre (the mean radius
    //          grows), the debris field expands, and it stays bounded. ----
    {
        auto& bs = loadFresh("Explosion");
        const glm::vec3 centre(0.0f, 4.0f, 0.0f);   // matches the scene's blast origin
        // Initial mean distance from the blast centre.
        auto meanRadius = [&](const std::vector<RigidBody>& v) {
            float s = 0.0f; int c = 0;
            for (auto& b : v) if (b.inverseMass > 0.0f) { s += glm::length(b.position - centre); ++c; }
            return (c > 0) ? s / c : 0.0f;
        };
        const float r0 = meanRadius(bs);
        // Every fragment should be moving outward at t=0 (radial launch).
        int outward = 0, dyn = 0;
        for (auto& b : bs) {
            if (b.inverseMass <= 0.0f) continue;
            ++dyn;
            const glm::vec3 radial = b.position - centre;
            if (glm::length(radial) > 1e-4f && glm::dot(glm::normalize(radial), b.velocity) > 0.0f) ++outward;
        }
        float maxAbs = 0.0f;
        for (int i = 0; i < 90; ++i) { // ~1.5 s: the expansion phase
            solver.step(bs, DT);
            for (auto& b : bs) if (b.inverseMass > 0.0f) maxAbs = std::max(maxAbs, glm::length(b.position));
        }
        const float r1 = meanRadius(bs);
        check("Explosion", "fragments launch radially outward", outward >= dyn - 1,
              "outward=" + std::to_string(outward) + "/" + std::to_string(dyn));
        check("Explosion", "debris field expands", r1 > r0 * 1.5f,
              "r0=" + std::to_string(r0) + " r1=" + std::to_string(r1));
        check("Explosion", "stays bounded (no runaway)", maxAbs < 60.0f,
              "maxAbsPos=" + std::to_string(maxAbs));
    }

    // ---- Boulder vs Castle: the boulder (last body, a big sphere) is hurled
    //          into the stacked-brick castle and must actually DEMOLISH it --
    //          a meaningful fraction of the bricks are knocked well away from
    //          their starting positions. The boulder must reach the wall (its
    //          X travels through the castle plane) rather than stopping short. ----
    {
        auto& bs = loadFresh("Boulder vs Castle");
        const int n = static_cast<int>(bs.size());
        const int boulderIdx = n - 1;               // boulder is the last body
        // Record every brick's start position (all dynamic bodies except the
        // boulder are bricks). The boulder is the only sphere.
        std::vector<glm::vec3> start(n);
        for (int i = 0; i < n; ++i) start[i] = bs[i].position;
        const float boulderX0 = bs[boulderIdx].position.x;

        simulate(solver, bs, 240); // 4 s: impact + scatter

        // Count bricks displaced meaningfully from where they started.
        int bricks = 0, knocked = 0;
        for (int i = 0; i < n; ++i) {
            if (i == boulderIdx) continue;
            ++bricks;
            if (glm::length(bs[i].position - start[i]) > 0.5f) ++knocked;
        }
        const float boulderTravelX = boulderX0 - bs[boulderIdx].position.x; // +ve = moved -X
        check("Boulder vs Castle", "boulder reaches the castle", boulderTravelX > 8.0f,
              "boulderTravelX=" + std::to_string(boulderTravelX));
        check("Boulder vs Castle", "castle is demolished (bricks scattered)", knocked >= bricks / 4,
              "knocked=" + std::to_string(knocked) + "/" + std::to_string(bricks));
    }

    // ---- Gyroscope REMOVED (see Scenes.h): the solver does not carry real
    //      gyroscopic coupling, so a genuine spin-stabilised gyroscope can't be
    //      done without faking it. Scene dropped rather than shipped misleading. ----

    // ---- Cable-Stayed Bridge: deck stays suspended and roughly level. ----
    {
        auto& bs = loadFresh("Cable-Stayed Bridge");
        simulate(solver, bs, 480);
        const float lowPlank = lowestBoxY(bs);
        const float tilt = maxBoxTilt(bs);
        check("Cable-Stayed Bridge", "deck stays suspended", lowPlank > 2.0f,
              "lowPlank_y=" + std::to_string(lowPlank));
        check("Cable-Stayed Bridge", "planks stay ~level (no spin)", tilt < 1.0f,
              "maxTilt_rad=" + std::to_string(tilt));
    }

    std::printf("\n========================================================\n");
    std::printf("  SCENE SUMMARY : %d passed, %d failed\n", gPass, gFail);
    std::printf("========================================================\n");
    return gFail == 0 ? 0 : 1;
}
