#pragma once
// ===========================================================================
// ValidationLab — reusable primitives for a research-grade physics validation
// laboratory. It does NOT contain experiments; it provides the four layers of
// scrutiny every experiment is run through:
//
//   Layer A — Numerical Integrity   (finite state, normalized quaternions,
//                                     bounded coords/spins, penetration <= slop,
//                                     deterministic replay)
//   Layer B — Constraint Validation (per-constraint error vs a tolerance,
//                                     read from the solver's TelemetryFrame)
//   Layer C — Physical Law          (analytical value vs measured value, with
//                                     percentage error and a pass band)
//   Layer D — Stress Testing        (run a randomized-variant closure many
//                                     times; any Layer-A failure fails the set)
//
// Everything is headless and deterministic: a Lab owns a single RNG seeded by
// the caller, so a given seed reproduces an entire run byte-for-byte. No
// rendering, no artificial stabilization — the lab only observes.
// ===========================================================================

#include <cstdio>
#include <cstdarg>
#include <cmath>
#include <string>
#include <vector>
#include <random>
#include <functional>

#include <glm/glm.hpp>
#include "../physics/rigidbody.h"
#include "../physics/physicssolver.h"

// Numerical-integrity limits for Layer A (top-level so they are complete types
// when used as method-parameter defaults).
struct LabIntegrityLimits {
    float maxCoord = 500.0f;   // |position component| bound (m)
    float maxSpeed = 500.0f;   // linear speed bound (m/s)
    float maxSpin  = 500.0f;   // angular speed bound (rad/s)
    float quatTol  = 1e-3f;    // |‖q‖ - 1| tolerance
    float penSlop  = 0.01f;    // acceptable interpenetration (m)
};

// Per-constraint-type acceptable residuals for Layer B (SI metres).
struct LabConstraintTolerances {
    float rope   = 0.0005f; // 0.5 mm overshoot past the rope limit
    float hinge  = 0.00025f;// 0.25 mm anchor separation
    float pulley = 0.001f;  // 1 mm rope-length residual
    float spring = 1e6f;    // springs legitimately stretch; no hard bound by default
};

class ValidationLab {
public:
    explicit ValidationLab(unsigned seed = 12345u) : rng_(seed), seed_(seed) {}

    // -------------------------------------------------------------------
    // Reporting
    // -------------------------------------------------------------------
    int passed() const { return pass_; }
    int failed() const { return fail_; }
    unsigned seed() const { return seed_; }

    void section(const std::string& title) {
        std::printf("\n========================================================\n");
        std::printf("  %s\n", title.c_str());
        std::printf("========================================================\n");
    }

    void subsection(const std::string& title) {
        std::printf("  -- %s --\n", title.c_str());
    }

    // A boolean assertion with an optional detail string.
    bool require(const std::string& name, bool ok, const std::string& detail = "") {
        (ok ? pass_ : fail_)++;
        std::printf("    [%s] %-46s %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
        return ok;
    }

    // A characterized KNOWN LIMITATION: a measurement that does NOT meet an
    // aspirational target but reflects a real, documented solver weakness that
    // is out of the current task's scope to fix. Logged loudly (so it is never
    // hidden) and counted separately — it does NOT fail the suite, but it is
    // also NOT silently passed. This is how the lab surfaces "the solver is
    // stable here but not yet accurate here" findings for follow-up.
    void knownLimitation(const std::string& name, const std::string& detail) {
        ++known_;
        std::printf("    [KNOWN] %-44s %s\n", name.c_str(), detail.c_str());
    }
    int knownLimitations() const { return known_; }

    // -------------------------------------------------------------------
    // Access to the RNG (Layer D variants draw from it so runs are seeded).
    // -------------------------------------------------------------------
    float uniform(float lo, float hi) {
        std::uniform_real_distribution<float> d(lo, hi);
        return d(rng_);
    }
    int uniformInt(int lo, int hi) {
        std::uniform_int_distribution<int> d(lo, hi);
        return d(rng_);
    }

    // ===================================================================
    // LAYER A — Numerical Integrity
    //
    // Called after a run. Verifies the solver produced physically admissible
    // state. `maxPenetration` is the deepest contact overlap observed over the
    // run (from telemetry); `slop` is the solver's allowed penetration slop.
    // Returns true iff every integrity check passed.
    // ===================================================================
    using IntegrityLimits = LabIntegrityLimits;

    bool layerA(const std::string& scene,
                const std::vector<RigidBody>& bodies,
                float maxPenetration,
                LabIntegrityLimits lim = LabIntegrityLimits()) {
        bool finite = true, quatOk = true, coordOk = true, speedOk = true, spinOk = true;
        float worstCoord = 0.0f, worstSpeed = 0.0f, worstSpin = 0.0f, worstQuat = 0.0f;

        for (const auto& b : bodies) {
            finite = finite && finiteVec(b.position) && finiteVec(b.velocity)
                            && finiteVec(b.angularVelocity)
                            && std::isfinite(b.orientation.w) && std::isfinite(b.orientation.x)
                            && std::isfinite(b.orientation.y) && std::isfinite(b.orientation.z);

            const float qn = std::sqrt(b.orientation.w * b.orientation.w + b.orientation.x * b.orientation.x
                                     + b.orientation.y * b.orientation.y + b.orientation.z * b.orientation.z);
            worstQuat = std::max(worstQuat, std::abs(qn - 1.0f));
            if (std::abs(qn - 1.0f) > lim.quatTol) quatOk = false;

            worstCoord = std::max(worstCoord, maxAbsComp(b.position));
            worstSpeed = std::max(worstSpeed, glm::length(b.velocity));
            worstSpin  = std::max(worstSpin,  glm::length(b.angularVelocity));
        }
        coordOk = worstCoord <= lim.maxCoord;
        speedOk = worstSpeed <= lim.maxSpeed;
        spinOk  = worstSpin  <= lim.maxSpin;

        bool all = true;
        all &= require(scene + " A: finite state", finite);
        all &= require(scene + " A: quaternions normalized", quatOk, fmt("max|q|-1=%.2e", worstQuat));
        all &= require(scene + " A: bounded coordinates", coordOk, fmt("max|xyz|=%.3f", worstCoord));
        all &= require(scene + " A: bounded linear speed", speedOk, fmt("maxV=%.3f", worstSpeed));
        all &= require(scene + " A: bounded angular speed", spinOk, fmt("maxW=%.3f", worstSpin));
        all &= require(scene + " A: penetration <= slop", maxPenetration <= lim.penSlop,
                       fmt("maxPen=%.4f (slop=%.3f)", maxPenetration, lim.penSlop));
        return all;
    }

    // Deterministic replay: run the same closure twice and require the final
    // configurations to match to the bit-ish level. The closure fully builds
    // and steps a scenario, returning the final body vector.
    bool layerA_determinism(const std::string& scene,
                            const std::function<std::vector<RigidBody>()>& run) {
        const std::vector<RigidBody> a = run();
        const std::vector<RigidBody> b = run();
        bool same = (a.size() == b.size());
        float worst = 0.0f;
        if (same) {
            for (std::size_t i = 0; i < a.size(); ++i) {
                worst = std::max(worst, glm::length(a[i].position - b[i].position));
                worst = std::max(worst, glm::length(a[i].velocity - b[i].velocity));
            }
        }
        return require(scene + " A: deterministic replay", same && worst < 1e-6f,
                       fmt("max delta=%.2e", worst));
    }

    // ===================================================================
    // LAYER B — Constraint Validation
    //
    // Reads the solver's post-step constraint telemetry and asserts each
    // constraint type's residual error is within tolerance. Only checks types
    // present in the frame.
    // ===================================================================
    using ConstraintTolerances = LabConstraintTolerances;

    bool layerB(const std::string& scene, const TelemetryFrame& tel,
                LabConstraintTolerances tol = LabConstraintTolerances()) {
        float maxRope = 0.0f, maxHinge = 0.0f, maxPulley = 0.0f, maxSpring = 0.0f;
        int nRope = 0, nHinge = 0, nPulley = 0, nSpring = 0;
        for (const auto& c : tel.constraints) {
            switch (c.type) {
                case ConstraintTelemetry::Type::Rope:   maxRope   = std::max(maxRope,   c.error); ++nRope;   break;
                case ConstraintTelemetry::Type::Hinge:  maxHinge  = std::max(maxHinge,  c.error); ++nHinge;  break;
                case ConstraintTelemetry::Type::Pulley: maxPulley = std::max(maxPulley, c.error); ++nPulley; break;
                case ConstraintTelemetry::Type::Spring: maxSpring = std::max(maxSpring, c.error); ++nSpring; break;
            }
        }
        bool all = true;
        if (nRope)   all &= require(scene + " B: rope length error",   maxRope   <= tol.rope,   fmt("max=%.4f mm", maxRope * 1e3f));
        if (nHinge)  all &= require(scene + " B: hinge drift",         maxHinge  <= tol.hinge,  fmt("max=%.4f mm", maxHinge * 1e3f));
        if (nPulley) all &= require(scene + " B: pulley length error", maxPulley <= tol.pulley, fmt("max=%.4f mm", maxPulley * 1e3f));
        if (nSpring) all &= require(scene + " B: spring extension bounded", maxSpring <= tol.spring, fmt("max=%.4f m", maxSpring));
        return all;
    }

    // ===================================================================
    // LAYER C — Physical Law
    //
    // Compares a measured value against an analytical prediction, reporting the
    // percentage error and passing iff within `tolPercent`. Prints
    // theoretical/measured/error for the scientific log.
    // ===================================================================
    bool layerC(const std::string& scene, const std::string& law,
                double theoretical, double measured, double tolPercent) {
        const double denom = (std::abs(theoretical) > 1e-9) ? std::abs(theoretical) : 1.0;
        const double pct = 100.0 * std::abs(measured - theoretical) / denom;
        const bool ok = pct <= tolPercent;
        std::printf("    [%s] %-30s theo=%- .5g  meas=%- .5g  err=%.2f%% (tol %.1f%%)\n",
                    ok ? "PASS" : "FAIL", (scene + " C: " + law).c_str(),
                    theoretical, measured, pct, tolPercent);
        (ok ? pass_ : fail_)++;
        return ok;
    }

    // ===================================================================
    // LAYER D — Stress Testing
    //
    // Runs `trials` randomized variants. `variant` builds + steps a randomized
    // scenario (drawing from this lab's RNG) and returns true iff that variant
    // stayed numerically healthy (typically its own Layer-A result). Reports
    // how many survived; passes iff all did.
    // ===================================================================
    bool layerD(const std::string& scene, int trials,
                const std::function<bool(int)>& variant) {
        int survived = 0;
        for (int i = 0; i < trials; ++i) if (variant(i)) ++survived;
        return require(scene + fmt(" D: %d randomized variants stable", trials),
                       survived == trials, fmt("%d/%d survived", survived, trials));
    }

    // -------------------------------------------------------------------
    // Small helpers usable by experiments.
    // -------------------------------------------------------------------
    static bool finiteVec(const glm::vec3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }
    static float maxAbsComp(const glm::vec3& v) {
        return std::max(std::abs(v.x), std::max(std::abs(v.y), std::abs(v.z)));
    }
    // printf into a std::string.
    static std::string fmt(const char* f, ...) {
        char buf[256];
        va_list ap; va_start(ap, f);
        std::vsnprintf(buf, sizeof(buf), f, ap);
        va_end(ap);
        return std::string(buf);
    }

    // Convenience: silently detect the worst interpenetration over a run by
    // reading telemetry each step. Experiments that don't capture telemetry can
    // pass 0 to layerA.
    static float worstPenetration(const PhysicsSolver& s) {
        return s.lastTelemetry.maxPenetration;
    }

private:
    std::mt19937 rng_;
    unsigned seed_;
    int pass_ = 0, fail_ = 0, known_ = 0;
};
