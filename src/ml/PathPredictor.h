#pragma once
// ===========================================================================
// PathPredictor — a lightweight, swappable trajectory predictor.
//
//   bool loadModel(const std::string& path);
//   std::vector<glm::vec3> predict(const Observation& obs, int futureFrames);
//
// Two backends live behind this single interface:
//
//   * Physics-rollout stub (DEFAULT, always available). If no model is loaded
//     it predicts by simulating the body forward in an isolated, headless
//     Environment for `futureFrames` fixed steps and returning the sampled
//     positions. This is a real physical prediction, not a placeholder curve.
//
//   * ONNX model (OPTIONAL). Compiled in only when PHYSICS_ENABLE_ONNX is
//     defined AND the ONNX Runtime headers/libs are available. All ONNX code
//     is confined to loadModel()/predictWithModel() below, so the engine
//     builds and runs identically whether or not ONNX is installed.
//
// The interface, observation shape, and output (a list of future positions)
// are fixed now so a trained model can be dropped in later WITHOUT touching
// the renderer, the solver, or the call sites.
// ===========================================================================

#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "../physics/rigidbody.h"
#include "Environment.h"

#if defined(PHYSICS_ENABLE_ONNX)
    // The ONNX Runtime C++ header. Only pulled in for an explicit ONNX build;
    // the default build never sees it, so ONNX need not be installed.
    #include <onnxruntime_cxx_api.h>
    #include <array>
    #include <algorithm>
    #include <memory>
#endif

class PathPredictor {
public:
    // How many fixed steps the rollout advances per predicted frame, and the
    // timestep, kept in sync with the app's FIXED_DT (1/60 s).
    static constexpr float kDt = 1.0f / 60.0f;

    // Attempt to load a model from `path`. Returns true if a model backend is
    // now active; false means "no model" and predict() uses the physics
    // rollout. Model loading is fully isolated here.
    bool loadModel(const std::string& path) {
        modelPath_ = path;
        modelLoaded_ = false;

#if defined(PHYSICS_ENABLE_ONNX)
        try {
            env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "PathPredictor");
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(1);
            session_ = std::make_unique<Ort::Session>(env_, path.c_str(), opts);
            modelLoaded_ = (session_ != nullptr);
        } catch (const std::exception&) {
            modelLoaded_ = false; // fall back to the physics rollout
        }
#else
        (void)path; // no ONNX in this build: always fall back to the rollout
#endif
        return modelLoaded_;
    }

    bool hasModel() const { return modelLoaded_; }
    const std::string& modelPath() const { return modelPath_; }

    // Predict the future path of a single body as a list of `futureFrames`
    // world-space positions (one per predicted frame, ordered forward in time).
    std::vector<glm::vec3> predict(const Observation& obs, int futureFrames) {
        if (futureFrames <= 0) return {};
#if defined(PHYSICS_ENABLE_ONNX)
        if (modelLoaded_) return predictWithModel(obs, futureFrames);
#endif
        return rollout(obs, futureFrames);
    }

private:
    // ---- Physics-rollout stub (default) -----------------------------------
    // Reconstruct a single-body world from the observation and integrate it
    // forward. Runs in its own Environment/solver instance, so it never touches
    // the live simulation state.
    std::vector<glm::vec3> rollout(const Observation& obs, int futureFrames) {
        RigidBody b;
        b.shape           = (obs.shape == 1) ? ShapeType::Sphere : ShapeType::Box;
        b.position        = obs.position;
        b.velocity        = obs.velocity;
        b.angularVelocity = obs.angularVelocity;
        b.orientation     = obs.orientation;
        if (obs.mass > 0.0f) {
            b.mass = obs.mass;
            b.inverseMass = 1.0f / obs.mass;
        }
        if (b.shape == ShapeType::Sphere) {
            // Radius isn't in the Observation; use the visual scale if present,
            // else a sensible default. Only affects self-collision with the
            // floor during the rollout.
            b.radius = 0.5f;
            b.scale = glm::vec3(b.radius * 2.0f);
        }
        b.updateInertiaTensor();

        Environment env;
        env.setBodies({b});
        // Sleeping would freeze the preview once the body settles; disable it so
        // the predicted path reflects the full requested horizon.
        env.solver().sleepingEnabled = false;

        std::vector<glm::vec3> path;
        path.reserve(futureFrames);
        for (int f = 0; f < futureFrames; ++f) {
            env.step(kDt);
            path.push_back(env.getObservation(0).position);
        }
        return path;
    }

#if defined(PHYSICS_ENABLE_ONNX)
    // ---- ONNX backend (optional, isolated) --------------------------------
    // Build the input tensor from the observation, run the session, and unpack
    // the output into a list of positions. The exact tensor layout is defined
    // by the trained model; this is the single place that changes when a real
    // model is wired in.
    std::vector<glm::vec3> predictWithModel(const Observation& obs, int futureFrames) {
        // Intentionally minimal + defensive: if anything about the model shape
        // is unexpected, fall back to the physics rollout rather than crash.
        try {
            // Example input feature vector (kept explicit for the future model):
            // [mass, px,py,pz, vx,vy,vz, wx,wy,wz, qw,qx,qy,qz]
            std::vector<float> input = {
                obs.mass,
                obs.position.x, obs.position.y, obs.position.z,
                obs.velocity.x, obs.velocity.y, obs.velocity.z,
                obs.angularVelocity.x, obs.angularVelocity.y, obs.angularVelocity.z,
                obs.orientation.w, obs.orientation.x, obs.orientation.y, obs.orientation.z
            };
            const std::array<int64_t, 2> shape{1, static_cast<int64_t>(input.size())};

            Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
                mem, input.data(), input.size(), shape.data(), shape.size());

            // NOTE: input/output node names are model-specific; wire them when a
            // real .onnx is available.
            const char* inputNames[]  = {"input"};
            const char* outputNames[] = {"output"};

            auto outputs = session_->Run(Ort::RunOptions{nullptr},
                                         inputNames, &inputTensor, 1,
                                         outputNames, 1);
            const float* out = outputs.front().GetTensorData<float>();
            const size_t count = outputs.front().GetTensorTypeAndShapeInfo().GetElementCount();

            std::vector<glm::vec3> path;
            const int frames = std::min<int>(futureFrames, static_cast<int>(count / 3));
            path.reserve(frames);
            for (int f = 0; f < frames; ++f) {
                path.emplace_back(out[f * 3 + 0], out[f * 3 + 1], out[f * 3 + 2]);
            }
            return path;
        } catch (const std::exception&) {
            return rollout(obs, futureFrames);
        }
    }

    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "PathPredictor"};
    std::unique_ptr<Ort::Session> session_;
#endif

    std::string modelPath_;
    bool modelLoaded_ = false;
};
