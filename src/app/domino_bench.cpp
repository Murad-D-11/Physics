// domino_bench — one-off headless timing of the Domino Spiral at scale.
//
// Builds the Domino Spiral through the real SceneManager + PhysicsSolver, sets
// the `dominoes` parameter to a chosen count, and steps it until the cascade
// completes (all bodies asleep) or a step cap is hit. Reports steps taken,
// total wall-clock compute time, ms/step, peak simultaneous contacts, and how
// many dominoes ended up toppled. No OpenGL.
//
// Build (from repo root):
//   g++ -O2 -std=c++17 -Iinclude src/app/domino_bench.cpp \
//       src/physics/physicssolver.cpp src/physics/obb.cpp src/physics/collision.cpp \
//       -o build/DominoBench.exe
// Run:
//   build/DominoBench.exe 3000

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <algorithm>

#include "../physics/physicssolver.h"
#include "../physics/rigidbody.h"
#include "Scene.h"
#include "SceneManager.h"
#include "Scenes.h"

static constexpr float DT = 1.0f / 60.0f;

int main(int argc, char** argv) {
    const int dominoes = (argc > 1) ? std::atoi(argv[1]) : 3000;
    const int stepCap  = (argc > 2) ? std::atoi(argv[2]) : 120000; // safety cap

    PhysicsSolver solver;
    solver.captureTelemetry = true;   // so we can read contact + sleep counts

    DominoSpiralScene domino;
    domino.setParam("dominoes", static_cast<float>(dominoes));

    SceneManager mgr(solver);
    mgr.registerScene("Domino Spiral", &domino);
    mgr.loadScene("Domino Spiral");
    std::vector<RigidBody>& bs = mgr.bodies();

    const int nBodies = static_cast<int>(bs.size());
    std::printf("Domino Spiral bench: %d dominoes (%d bodies), dt=1/60 s\n",
                dominoes, nBodies);

    int   peakContacts   = 0;
    int   peakAwake      = 0;
    long long contactStepSum = 0; // for average contacts/step
    int   steps = 0;

    const auto t0 = std::chrono::steady_clock::now();

    // Run until everything sleeps (cascade finished + settled) or the cap.
    // Give it a short warmup so the starter domino has actually begun moving
    // before we start checking for "all asleep".
    for (; steps < stepCap; ++steps) {
        solver.step(bs, DT);
        const TelemetryFrame& t = solver.lastTelemetry;
        peakContacts = std::max(peakContacts, t.contactCount);
        peakAwake    = std::max(peakAwake, t.awakeCount);
        contactStepSum += t.contactCount;
        // Stop once the whole pile is asleep (after a warmup window).
        if (steps > 120 && t.awakeCount == 0) { ++steps; break; }
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double msPerStep = (steps > 0) ? totalMs / steps : 0.0;
    const double avgContacts = (steps > 0) ? static_cast<double>(contactStepSum) / steps : 0.0;

    // Count toppled dominoes (local up-axis fallen away from world up).
    int toppled = 0;
    for (const auto& b : bs) {
        if (b.inverseMass <= 0.0f) continue;
        const glm::vec3 up = b.orientation * glm::vec3(0, 1, 0);
        if (up.y < 0.7f) ++toppled;
    }

    std::printf("---------------------------------------------\n");
    std::printf("steps to settle:        %d  (%.2f s simulated)\n", steps, steps * DT);
    std::printf("total compute wall time:%.1f ms  (%.2f s)\n", totalMs, totalMs / 1000.0);
    std::printf("ms / step:              %.3f\n", msPerStep);
    std::printf("real-time factor:       %.2fx  (>1 = faster than real time)\n",
                (msPerStep > 0.0) ? (1000.0 / 60.0) / msPerStep : 0.0);
    std::printf("peak simultaneous contacts: %d\n", peakContacts);
    std::printf("avg contacts / step:    %.1f\n", avgContacts);
    std::printf("peak awake bodies:      %d / %d\n", peakAwake, nBodies);
    std::printf("dominoes toppled:       %d / %d\n", toppled, nBodies);
    return 0;
}
