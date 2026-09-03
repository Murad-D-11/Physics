#pragma once
// ===========================================================================
// SimulationRecorder — in-memory, ML-friendly capture of a simulation run.
//
// After every physics timestep the app calls capture(time, bodies); the
// recorder appends one row PER BODY (time, object id, position, velocity,
// angular velocity, orientation quaternion). It only *reads* body state, so it
// can never affect the physics (no pointers into the solver, no mutation).
//
// The object id is the body's index within the recorded bodies vector. To keep
// ids unambiguous, the app clears() the recording whenever the set of bodies
// changes structurally (spawn / delete / scene reset). Within a single run the
// ids are therefore stable and the data is deterministic.
//
// export CSV: one flat table, one row per (frame, body). Columns are fixed and
// header-labelled so it loads directly into pandas / numpy with no reshaping.
// ===========================================================================

#include <vector>
#include <string>
#include <fstream>
#include <cstdint>
#include <cstdio>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../physics/rigidbody.h"

class SimulationRecorder {
public:
    // One recorded sample: a single body at a single timestep.
    struct Frame {
        std::uint64_t step;      // integer step index (0,1,2,...)
        float time;              // simulated time (s)
        int   objectId;          // body index within the scene
        int   shape;             // 0 = box, 1 = sphere
        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 angularVelocity;
        glm::quat orientation;   // (w, x, y, z)
    };

    void setEnabled(bool on) { enabled_ = on; }
    bool enabled() const { return enabled_; }
    bool empty()   const { return frames_.empty(); }
    std::size_t rowCount() const { return frames_.size(); }
    std::uint64_t stepCount() const { return stepIndex_; }

    // Discard all recorded data and reset the clock. Called on any structural
    // change to the world so object ids stay consistent.
    void clear() {
        frames_.clear();
        stepIndex_ = 0;
        time_ = 0.0f;
    }

    // Record one timestep. Read-only over `bodies`; advances the internal
    // step/time counters. `dt` is the fixed physics timestep.
    void capture(const std::vector<RigidBody>& bodies, float dt) {
        if (!enabled_) return;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const RigidBody& b = bodies[i];
            Frame f;
            f.step            = stepIndex_;
            f.time            = time_;
            f.objectId        = static_cast<int>(i);
            f.shape           = static_cast<int>(b.shape);
            f.position        = b.position;
            f.velocity        = b.velocity;
            f.angularVelocity = b.angularVelocity;
            f.orientation     = b.orientation;
            frames_.push_back(f);
        }
        ++stepIndex_;
        time_ += dt;
    }

    // Write every recorded frame to one CSV file. Returns false on I/O error.
    // Format: fixed columns, header row, '.'-decimal, no locale dependence.
    bool exportCSV(const std::string& path) const {
        std::ofstream out(path, std::ios::out | std::ios::trunc);
        if (!out.is_open()) return false;

        out << "step,time,object_id,shape,"
               "pos_x,pos_y,pos_z,"
               "vel_x,vel_y,vel_z,"
               "ang_vel_x,ang_vel_y,ang_vel_z,"
               "quat_w,quat_x,quat_y,quat_z\n";

        char line[512];
        for (const Frame& f : frames_) {
            std::snprintf(line, sizeof(line),
                "%llu,%.6f,%d,%d,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f\n",
                static_cast<unsigned long long>(f.step), f.time, f.objectId, f.shape,
                f.position.x, f.position.y, f.position.z,
                f.velocity.x, f.velocity.y, f.velocity.z,
                f.angularVelocity.x, f.angularVelocity.y, f.angularVelocity.z,
                f.orientation.w, f.orientation.x, f.orientation.y, f.orientation.z);
            out << line;
        }
        return out.good();
    }

private:
    std::vector<Frame> frames_;
    std::uint64_t stepIndex_ = 0;
    float time_ = 0.0f;
    bool enabled_ = true; // record by default; toggle from the app
};
