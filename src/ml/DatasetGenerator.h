#pragma once
// ===========================================================================
// DatasetGenerator — builds clean supervised-learning datasets from random
// rigid-body scenes, using the headless Environment. No neural networks, no
// rendering: it only simulates and writes CSV.
//
// Pipeline (per episode):
//   1. Build a random scene: N in [3, 20] bodies, random shapes (box/sphere),
//      random masses, random start positions, random initial impulses.
//   2. Simulate `frames` fixed steps, occasionally injecting random impulses,
//      recording every body's full state at every frame.
//   3. Emit supervised rows: each row is a body's observation at frame f paired
//      with the LABEL = that same body's position at frame f + horizon
//      (default 30 frames ahead). Rows without a valid future frame are
//      dropped, so every row has a complete label.
//
// Determinism: a single std::mt19937 seeded by the caller drives all randomness
// and the physics is fixed-step, so a given seed reproduces the dataset byte
// for byte.
//
// CSV layout (one row per (episode, body, frame) with a valid future):
//   episode, frame, object_id, shape, mass,
//   pos_x,pos_y,pos_z, vel_x,vel_y,vel_z,
//   ang_vel_x,ang_vel_y,ang_vel_z, quat_w,quat_x,quat_y,quat_z, sleeping,
//   future_pos_x, future_pos_y, future_pos_z   <-- supervised label
// ===========================================================================

#include <vector>
#include <string>
#include <fstream>
#include <random>
#include <cstdio>
#include <glm/glm.hpp>

#include "../physics/rigidbody.h"
#include "../physics/Material.h"
#include "Environment.h"

class DatasetGenerator {
public:
    struct Config {
        int   episodes      = 20;         // number of random scenes
        int   frames        = 120;        // simulated steps per episode
        int   horizon       = 30;         // label = position this many frames ahead
        float dt            = 1.0f / 60.0f;
        int   minBodies     = 3;
        int   maxBodies     = 20;
        float minMass       = 0.5f;
        float maxMass       = 5.0f;
        float impulseChance = 0.02f;      // per body, per frame chance of a random impulse
        float maxImpulse    = 6.0f;       // magnitude cap for random impulses (kg*m/s)
        unsigned seed       = 12345u;     // master RNG seed (reproducible)
    };

    explicit DatasetGenerator(const Config& cfg) : cfg_(cfg), rng_(cfg.seed) {}

    // Generate the full dataset and write it to `path`. Returns false on I/O
    // error. Writes one header row then all sample rows.
    bool generate(const std::string& path) {
        std::ofstream out(path, std::ios::out | std::ios::trunc);
        if (!out.is_open()) return false;

        out << "episode,frame,object_id,shape,material,mass,"
               "pos_x,pos_y,pos_z,"
               "vel_x,vel_y,vel_z,"
               "ang_vel_x,ang_vel_y,ang_vel_z,"
               "quat_w,quat_x,quat_y,quat_z,sleeping,"
               "future_pos_x,future_pos_y,future_pos_z\n";

        std::size_t rows = 0;
        for (int ep = 0; ep < cfg_.episodes; ++ep) {
            rows += runEpisode(ep, out);
        }
        lastRowCount_ = rows;
        return out.good();
    }

    std::size_t lastRowCount() const { return lastRowCount_; }

private:
    // Simulate one episode and stream its supervised rows into `out`.
    std::size_t runEpisode(int episode, std::ofstream& out) {
        Environment env;
        buildRandomScene(env);
        const int n = static_cast<int>(env.size());

        // Record per-frame observations for the whole episode first, so we can
        // look up each body's future position when emitting labels.
        // history[f] = observations of all bodies at frame f.
        std::vector<std::vector<Observation>> history;
        history.reserve(cfg_.frames + 1);

        history.push_back(env.getObservation()); // frame 0 (initial state)
        for (int f = 1; f <= cfg_.frames; ++f) {
            // Occasionally poke random bodies with random impulses.
            maybeApplyRandomImpulses(env, n);
            env.step(cfg_.dt);
            history.push_back(env.getObservation());
        }

        // Emit rows: for every frame f that has a valid frame f+horizon, write
        // each body's state at f with its future position as the label.
        std::size_t rows = 0;
        const int lastLabelled = static_cast<int>(history.size()) - 1 - cfg_.horizon;
        char line[640];
        for (int f = 0; f <= lastLabelled; ++f) {
            const std::vector<Observation>& now    = history[f];
            const std::vector<Observation>& future = history[f + cfg_.horizon];
            for (int i = 0; i < n; ++i) {
                const Observation& o = now[i];
                const glm::vec3&   fp = future[i].position; // same body id
                std::snprintf(line, sizeof(line),
                    "%d,%d,%d,%d,%d,%.6f,"
                    "%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,%.6f,%d,"
                    "%.6f,%.6f,%.6f\n",
                    episode, f, o.id, o.shape, o.material, o.mass,
                    o.position.x, o.position.y, o.position.z,
                    o.velocity.x, o.velocity.y, o.velocity.z,
                    o.angularVelocity.x, o.angularVelocity.y, o.angularVelocity.z,
                    o.orientation.w, o.orientation.x, o.orientation.y, o.orientation.z,
                    o.sleeping ? 1 : 0,
                    fp.x, fp.y, fp.z);
                out << line;
                ++rows;
            }
        }
        return rows;
    }

    // Build a random but physically sensible scene into `env`.
    void buildRandomScene(Environment& env) {
        std::uniform_int_distribution<int>  countDist(cfg_.minBodies, cfg_.maxBodies);
        std::uniform_int_distribution<int>  shapeDist(0, 1);
        std::uniform_int_distribution<int>  matDist(0, static_cast<int>(MaterialType::Count) - 1);
        std::uniform_real_distribution<float> massDist(cfg_.minMass, cfg_.maxMass);
        std::uniform_real_distribution<float> xzDist(-4.0f, 4.0f);
        std::uniform_real_distribution<float> yDist(1.0f, 8.0f);
        std::uniform_real_distribution<float> sizeDist(0.4f, 1.2f);
        std::uniform_real_distribution<float> spinDist(-2.0f, 2.0f);

        const int n = countDist(rng_);
        std::vector<RigidBody> bodies;
        bodies.reserve(n);

        for (int i = 0; i < n; ++i) {
            RigidBody b;
            const float mass = massDist(rng_);
            b.position = glm::vec3(xzDist(rng_), yDist(rng_), xzDist(rng_));
            b.angularVelocity = glm::vec3(spinDist(rng_), spinDist(rng_), spinDist(rng_));

            if (shapeDist(rng_) == 1) {
                // Sphere
                const float r = sizeDist(rng_) * 0.5f;
                b.shape = ShapeType::Sphere;
                b.radius = r;
                b.scale = glm::vec3(r * 2.0f);
            } else {
                // Box
                b.shape = ShapeType::Box;
                b.scale = glm::vec3(sizeDist(rng_), sizeDist(rng_), sizeDist(rng_));
            }
            b.mass = mass;
            b.inverseMass = 1.0f / mass;

            // Assign a random material: sets friction/restitution and derives a
            // physically consistent mass from density * volume (overriding the
            // placeholder mass above), and tags b.materialType for the dataset.
            const MaterialType mt = static_cast<MaterialType>(matDist(rng_));
            applyMaterial(b, mt);

            // Random initial impulse -> initial velocity (dv = J / m).
            b.velocity = randomImpulse() * b.inverseMass;

            bodies.push_back(b);
        }

        env.setBodies(bodies);
    }

    // With probability cfg_.impulseChance per body per frame, apply a random
    // impulse. This keeps trajectories varied so the dataset isn't just decay
    // to rest.
    void maybeApplyRandomImpulses(Environment& env, int n) {
        std::uniform_real_distribution<float> chance(0.0f, 1.0f);
        for (int i = 0; i < n; ++i) {
            if (chance(rng_) < cfg_.impulseChance) {
                env.applyAction(i, randomImpulse());
            }
        }
    }

    // A random impulse vector with magnitude in [0, maxImpulse], biased upward
    // slightly in Y so bodies occasionally hop rather than only sliding.
    glm::vec3 randomImpulse() {
        std::uniform_real_distribution<float> comp(-1.0f, 1.0f);
        std::uniform_real_distribution<float> mag(0.0f, cfg_.maxImpulse);
        glm::vec3 dir(comp(rng_), comp(rng_) * 0.5f + 0.25f, comp(rng_));
        const float len = glm::length(dir);
        if (len < 1e-6f) return glm::vec3(0.0f);
        return (dir / len) * mag(rng_);
    }

    Config cfg_;
    std::mt19937 rng_;
    std::size_t lastRowCount_ = 0;
};
