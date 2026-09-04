#pragma once
// ===========================================================================
// TelemetryExport — AI-readable, rendering-independent capture of the solver's
// per-step TelemetryFrame to a flat CSV.
//
// The physics engine already computes a complete, value-typed TelemetryFrame
// every step (see physics/Telemetry.h). This helper does NOT recompute any
// physics: it observes solver.lastTelemetry after each step and appends rows.
// The result is a long-format table (one row per body per frame) plus a
// parallel system-level column set, exactly the shape a reinforcement-learning
// pipeline wants (each row is a labelled observation with the frame's global
// quantities attached for context).
//
// Design goals mirror the rest of the ML layer:
//   * No OpenGL / rendering dependency.
//   * Deterministic: identical simulation -> identical CSV, byte for byte.
//   * Optimised for data extraction, not display.
//
// Usage:
//   TelemetryExport rec;
//   solver.captureTelemetry = true;           // solver must be filling frames
//   for (...) { solver.step(bodies, dt); rec.capture(solver.lastTelemetry); }
//   rec.writeCSV("run.csv");
// ===========================================================================

#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>

#include "../physics/Telemetry.h"

class TelemetryExport {
public:
    // Append every body of one frame as its own row, with the frame's global
    // aggregates attached to each row for context. Read-only over `f`.
    void capture(const TelemetryFrame& f) {
        // Per-constraint-type worst error this frame (0 if none of that type),
        // so an RL agent sees how well each constraint class is satisfied.
        float ropeErr = 0.0f, hingeErr = 0.0f, pulleyErr = 0.0f, springErr = 0.0f;
        for (const auto& c : f.constraints) {
            switch (c.type) {
                case ConstraintTelemetry::Type::Rope:   ropeErr   = std::max(ropeErr,   c.error); break;
                case ConstraintTelemetry::Type::Hinge:  hingeErr  = std::max(hingeErr,  c.error); break;
                case ConstraintTelemetry::Type::Pulley: pulleyErr = std::max(pulleyErr, c.error); break;
                case ConstraintTelemetry::Type::Spring: springErr = std::max(springErr, c.error); break;
            }
        }
        const float totalKE = f.kineticLinear + f.kineticRotational;

        for (const auto& b : f.bodies) {
            Row r;
            r.frame  = f.frameIndex;
            r.time   = f.simTime;
            r.id     = b.index;
            r.shape  = b.shape;
            r.mass   = b.mass;
            r.asleep = b.asleep ? 1 : 0;
            r.px = b.position.x; r.py = b.position.y; r.pz = b.position.z;
            r.vx = b.velocity.x; r.vy = b.velocity.y; r.vz = b.velocity.z;
            r.wx = b.angularVelocity.x; r.wy = b.angularVelocity.y; r.wz = b.angularVelocity.z;
            r.qw = b.orientation.w; r.qx = b.orientation.x; r.qy = b.orientation.y; r.qz = b.orientation.z;
            // System-level context (same for every row of this frame).
            r.contacts   = f.contactCount;
            r.keTotal    = totalKE;
            r.pe         = f.potential;
            r.mechEnergy = f.mechanicalEnergy;
            r.px_sys = f.linearMomentum.x; r.py_sys = f.linearMomentum.y; r.pz_sys = f.linearMomentum.z;
            r.Lx = f.angularMomentum.x; r.Ly = f.angularMomentum.y; r.Lz = f.angularMomentum.z;
            r.ropeErr = ropeErr; r.hingeErr = hingeErr; r.pulleyErr = pulleyErr; r.springErr = springErr;
            r.maxPen = f.maxPenetration;
            rows_.push_back(r);
        }
    }

    std::size_t rowCount() const { return rows_.size(); }
    void clear() { rows_.clear(); }

    // Write all captured rows to `path`. Returns false on I/O error. Fixed
    // column order, '.'-decimal, one header line — trivially loadable by
    // pandas/numpy without reshaping.
    bool writeCSV(const std::string& path) const {
        std::FILE* fp = std::fopen(path.c_str(), "wb");
        if (!fp) return false;

        std::fputs(
            "frame,time,object_id,shape,mass,sleeping,"
            "pos_x,pos_y,pos_z,"
            "vel_x,vel_y,vel_z,"
            "ang_vel_x,ang_vel_y,ang_vel_z,"
            "quat_w,quat_x,quat_y,quat_z,"
            "sys_contacts,sys_ke,sys_pe,sys_mech_energy,"
            "sys_px,sys_py,sys_pz,sys_Lx,sys_Ly,sys_Lz,"
            "rope_err,hinge_err,pulley_err,spring_err,max_penetration\n", fp);

        char line[768];
        for (const auto& r : rows_) {
            std::snprintf(line, sizeof(line),
                "%llu,%.6f,%d,%d,%.6f,%d,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,"
                "%d,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.8f,%.8f,%.8f,%.8f,%.8f\n",
                (unsigned long long)r.frame, r.time, r.id, r.shape, r.mass, r.asleep,
                r.px, r.py, r.pz,
                r.vx, r.vy, r.vz,
                r.wx, r.wy, r.wz,
                r.qw, r.qx, r.qy, r.qz,
                r.contacts, r.keTotal, r.pe, r.mechEnergy,
                r.px_sys, r.py_sys, r.pz_sys, r.Lx, r.Ly, r.Lz,
                r.ropeErr, r.hingeErr, r.pulleyErr, r.springErr, r.maxPen);
            std::fputs(line, fp);
        }
        std::fclose(fp);
        return true;
    }

private:
    struct Row {
        std::uint64_t frame;
        double time;
        int id, shape, asleep;
        float mass;
        float px, py, pz, vx, vy, vz, wx, wy, wz, qw, qx, qy, qz;
        int contacts;
        float keTotal, pe, mechEnergy;
        float px_sys, py_sys, pz_sys, Lx, Ly, Lz;
        float ropeErr, hingeErr, pulleyErr, springErr, maxPen;
    };
    std::vector<Row> rows_;
};
