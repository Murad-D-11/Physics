#include <iostream>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../renderer/render.h"
#include "../renderer/camera.h"
#include "../renderer/cube.h"
#include "../renderer/sphere.h"
#include "../renderer/ground.h"
#include "../physics/rigidbody.h"
#include "../physics/physicssolver.h"
#include "../physics/Material.h"

#include "Scene.h"
#include "SceneManager.h"
#include "Scenes.h"
#include "SimulationRecorder.h"
#include "../ml/PathPredictor.h"

using namespace std;

Camera* activeCamera = nullptr;
SceneManager* activeSceneManager = nullptr; // set in main(); used by key input
PhysicsSolver* activeSolver = nullptr;      // set in main(); env controls
SimulationRecorder* activeRecorder = nullptr; // set in main(); records each step
SandboxScene* activeSandbox = nullptr;        // set when the sandbox is loaded
PathPredictor* activePredictor = nullptr;     // set in main(); trajectory preview
bool simulationPaused = true; // starts paused; press Space to begin
bool isDragging = false;      // left-drag orbiting the camera
double lastMouseX = 0.0;
double lastMouseY = 0.0;

// --- Feature toggles (shown in the HUD) ---
bool predictionEnabled = false; // P: draw predicted trajectory of selection
bool recordingEnabled  = true;  // R: capture each timestep into the recorder
static constexpr int PREDICTION_FRAMES = 90; // ~1.5 s preview at 60 Hz

// --- Physics inspector overlay toggles (Part 4) ---
bool ovContactNormals = false;  // F1: contact normals (from telemetry)
bool ovAngularAxis    = false;  // F2: angular-velocity axis per body
bool ovCenterOfMass   = false;  // F3: center-of-mass cross
bool ovBoundingVolume = false;  // F4: axis-aligned bounding box
bool ovSleeping       = true;   // F5: tint sleeping bodies (on by default)

// --- Single-frame step request (Simulation panel) ---
bool stepOnce = false;          // set by the '.' key; advances exactly one step

// --- Interaction state ------------------------------------------------------
int   selectedBody   = -1;    // index into the active scene's bodies (-1 = none)
bool  draggingBody   = false; // left-drag repositioning the selected body (paused only)
bool  applyingImpulse = false;// right-drag aiming an impulse at the selected body
double impulseStartX = 0.0, impulseStartY = 0.0;
glm::vec3 dragPlanePoint(0.0f); // point on the drag plane under the cursor at grab

// Screen -> world picking ray (origin at the camera, direction through the
// pixel under the cursor). Built from the inverse of proj*view. Read-only.
static void screenRay(double mouseX, double mouseY, int width, int height,
                      const Camera& cam, glm::vec3& outOrigin, glm::vec3& outDir) {
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
    // Normalised device coordinates in [-1, 1], y flipped (screen y grows down).
    const float ndcX = (2.0f * static_cast<float>(mouseX)) / static_cast<float>(width) - 1.0f;
    const float ndcY = 1.0f - (2.0f * static_cast<float>(mouseY)) / static_cast<float>(height);

    const glm::mat4 invVP = glm::inverse(cam.getProjectionMatrix(aspect) * cam.getViewMatrix());
    glm::vec4 nearP = invVP * glm::vec4(ndcX, ndcY, -1.0f, 1.0f); // near plane
    glm::vec4 farP  = invVP * glm::vec4(ndcX, ndcY,  1.0f, 1.0f); // far plane
    nearP /= nearP.w;
    farP  /= farP.w;

    outOrigin = glm::vec3(nearP);
    outDir    = glm::normalize(glm::vec3(farP - nearP));
}

// Ray vs sphere. Returns nearest positive hit distance in `t`, or false.
static bool raySphere(const glm::vec3& o, const glm::vec3& d,
                      const glm::vec3& center, float radius, float& t) {
    const glm::vec3 oc = o - center;
    const float b = glm::dot(oc, d);
    const float c = glm::dot(oc, oc) - radius * radius;
    const float disc = b * b - c;
    if (disc < 0.0f) return false;
    const float sq = std::sqrt(disc);
    float t0 = -b - sq;
    float t1 = -b + sq;
    if (t0 < 0.0f) t0 = t1;
    if (t0 < 0.0f) return false;
    t = t0;
    return true;
}

// Ray vs oriented box (OBB). Slab test in the body's local frame. Returns the
// nearest positive entry distance in `t`, or false.
static bool rayOBB(const glm::vec3& o, const glm::vec3& d,
                   const glm::vec3& center, const glm::quat& orient,
                   const glm::vec3& halfExtents, float& t) {
    // Transform the ray into the box's local space.
    const glm::quat inv = glm::conjugate(orient);
    const glm::vec3 lo = inv * (o - center);
    const glm::vec3 ld = inv * d;

    float tmin = -1e30f, tmax = 1e30f;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(ld[i]) < 1e-8f) {
            if (lo[i] < -halfExtents[i] || lo[i] > halfExtents[i]) return false; // parallel & outside
        } else {
            float inv1 = 1.0f / ld[i];
            float ta = (-halfExtents[i] - lo[i]) * inv1;
            float tb = ( halfExtents[i] - lo[i]) * inv1;
            if (ta > tb) std::swap(ta, tb);
            tmin = std::max(tmin, ta);
            tmax = std::min(tmax, tb);
            if (tmin > tmax) return false;
        }
    }
    float hit = (tmin >= 0.0f) ? tmin : tmax;
    if (hit < 0.0f) return false;
    t = hit;
    return true;
}

// Pick the nearest body under the cursor. Returns its index or -1.
static int pickBody(double mouseX, double mouseY, int width, int height,
                    const Camera& cam, const std::vector<RigidBody>& bodies) {
    glm::vec3 o, d;
    screenRay(mouseX, mouseY, width, height, cam, o, d);

    int best = -1;
    float bestT = 1e30f;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const RigidBody& b = bodies[i];
        float t;
        bool hit = (b.shape == ShapeType::Sphere)
            ? raySphere(o, d, b.position, b.radius, t)
            : rayOBB(o, d, b.position, b.orientation, b.scale * 0.5f, t);
        if (hit && t < bestT) { bestT = t; best = static_cast<int>(i); }
    }
    return best;
}

// Intersect a picking ray with the horizontal plane y = planeY. Returns false
// if the ray is parallel to the plane. Used for drag-repositioning.
static bool rayPlaneY(const glm::vec3& o, const glm::vec3& d, float planeY, glm::vec3& out) {
    if (std::abs(d.y) < 1e-8f) return false;
    const float t = (planeY - o.y) / d.y;
    if (t < 0.0f) return false;
    out = o + d * t;
    return true;
}

// Clear the recorder on any structural change so object ids stay consistent.
static void resetRecordingForStructuralChange() {
    if (activeRecorder) activeRecorder->clear();
}

void framebufferSizeChanged(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (activeCamera == nullptr || activeSceneManager == nullptr) return;

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    double mx, my;
    glfwGetCursorPos(window, &mx, &my);
    std::vector<RigidBody>& bodies = activeSceneManager->bodies();

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            // First, try to pick a body under the cursor.
            const int hit = pickBody(mx, my, width, height, *activeCamera, bodies);
            if (hit >= 0) {
                selectedBody = hit;
                // While paused, a left-drag on a picked body repositions it on
                // the horizontal plane through the body's current height.
                if (simulationPaused) {
                    glm::vec3 o, d;
                    screenRay(mx, my, width, height, *activeCamera, o, d);
                    glm::vec3 hitPt;
                    if (rayPlaneY(o, d, bodies[hit].position.y, hitPt)) {
                        draggingBody = true;
                        dragPlanePoint = hitPt;
                    }
                }
                // If not repositioning, fall through to orbit the camera too.
                isDragging = true;
                lastMouseX = mx; lastMouseY = my;
            } else {
                // Empty space: clear selection and orbit the camera.
                selectedBody = -1;
                isDragging = true;
                lastMouseX = mx; lastMouseY = my;
            }
        } else if (action == GLFW_RELEASE) {
            isDragging = false;
            draggingBody = false;
        }
        return;
    }

    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            // Right-press picks (if nothing selected yet) then begins aiming an
            // impulse; the impulse is applied on release based on drag vector.
            const int hit = pickBody(mx, my, width, height, *activeCamera, bodies);
            if (hit >= 0) selectedBody = hit;
            if (selectedBody >= 0) {
                applyingImpulse = true;
                impulseStartX = mx; impulseStartY = my;
            }
        } else if (action == GLFW_RELEASE && applyingImpulse) {
            applyingImpulse = false;
            if (selectedBody >= 0 && selectedBody < static_cast<int>(bodies.size())) {
                // Convert the screen drag into a world-space velocity along the
                // camera's right/up axes. Dragging further = stronger impulse.
                const float dx = static_cast<float>(mx - impulseStartX);
                const float dy = static_cast<float>(my - impulseStartY);
                const glm::mat4 view = activeCamera->getViewMatrix();
                // Camera basis rows of the view matrix give world axes.
                const glm::vec3 right(view[0][0], view[1][0], view[2][0]);
                const glm::vec3 up   (view[0][1], view[1][1], view[2][1]);
                const float scale = 0.05f; // pixels -> m/s
                glm::vec3 impulse = right * (dx * scale) + up * (-dy * scale);
                RigidBody& b = bodies[selectedBody];
                if (b.inverseMass > 0.0f) {
                    b.velocity += impulse;   // apply as a velocity change (impulse / mass)
                    b.asleep = false;        // wake it so the impulse takes effect
                    b.sleepTimer = 0.0f;
                }
            }
        }
        return;
    }
}

void cursorPosCallback(GLFWwindow* window, double xPos, double yPos) {
    if (activeCamera == nullptr) return;

    // Repositioning the selected body takes priority over camera orbit.
    if (draggingBody && simulationPaused && activeSceneManager &&
        selectedBody >= 0) {
        std::vector<RigidBody>& bodies = activeSceneManager->bodies();
        if (selectedBody < static_cast<int>(bodies.size())) {
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);
            glm::vec3 o, d;
            screenRay(xPos, yPos, width, height, *activeCamera, o, d);
            glm::vec3 hitPt;
            RigidBody& b = bodies[selectedBody];
            if (rayPlaneY(o, d, b.position.y, hitPt)) {
                const glm::vec3 delta = hitPt - dragPlanePoint;
                b.position += glm::vec3(delta.x, 0.0f, delta.z); // slide on the plane
                b.velocity = glm::vec3(0.0f);                    // no residual velocity
                dragPlanePoint = hitPt;
                // Moving a body invalidates any prior recording's geometry.
                resetRecordingForStructuralChange();
            }
        }
        lastMouseX = xPos; lastMouseY = yPos;
        return;
    }

    if (!isDragging) return;
    const float deltaX = static_cast<float>(xPos - lastMouseX);
    const float deltaY = static_cast<float>(yPos - lastMouseY);
    lastMouseX = xPos;
    lastMouseY = yPos;
    activeCamera->processMouseDrag(deltaX, -deltaY);
}

void scrollCallback(GLFWwindow* window, double xOffset, double yOffset) {
    if (activeCamera == nullptr) return;
    activeCamera->processScroll(static_cast<float>(yOffset));
}

// Print the current UI status to the console (scene + recording + prediction).
// The on-screen HUD shows colored bars; this gives the full text readout since
// there is no font renderer.
static void logStatus() {
    Scene* s = activeSceneManager ? activeSceneManager->activeScene() : nullptr;
    const char* scene = activeSceneManager ? activeSceneManager->activeName().c_str() : "(none)";
    const char* model = (activePredictor && activePredictor->hasModel())
        ? activePredictor->modelPath().c_str() : "(none: physics rollout)";
    std::cout << "[Status] Scene: " << scene
              << "  |  Recording: " << (recordingEnabled ? "ON" : "OFF")
              << "  |  Prediction: " << (predictionEnabled ? "ON" : "OFF")
              << "  |  Model: " << model
              << (simulationPaused ? "  |  (paused)" : "") << "\n";
    if (s && s->description()[0]) {
        std::cout << "         " << s->description();
        if (s->principle()[0]) std::cout << "  [" << s->principle() << "]";
        std::cout << "\n";
    }
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action != GLFW_PRESS) return;

    // --- Primary UI toggles (per task spec) --------------------------------
    // P = prediction on/off, R = recording on/off. Space pauses/resumes.
    if (key == GLFW_KEY_P) {
        predictionEnabled = !predictionEnabled;
        logStatus();
        return;
    }
    if (key == GLFW_KEY_R) {
        recordingEnabled = !recordingEnabled;
        if (activeRecorder) activeRecorder->setEnabled(recordingEnabled);
        logStatus();
        return;
    }
    if (key == GLFW_KEY_SPACE) {
        simulationPaused = !simulationPaused;
        logStatus();
        return;
    }

    // Single-frame step (advance exactly one fixed step while paused).
    if (key == GLFW_KEY_PERIOD) {
        stepOnce = true;
        return;
    }

    // --- Physics inspector overlay toggles (F1-F5) -------------------------
    if (key == GLFW_KEY_F1) { ovContactNormals = !ovContactNormals; std::cout << "[Overlay] contact normals "  << (ovContactNormals?"ON":"OFF") << "\n"; return; }
    if (key == GLFW_KEY_F2) { ovAngularAxis    = !ovAngularAxis;    std::cout << "[Overlay] angular axis "     << (ovAngularAxis?"ON":"OFF")    << "\n"; return; }
    if (key == GLFW_KEY_F3) { ovCenterOfMass   = !ovCenterOfMass;   std::cout << "[Overlay] center of mass "   << (ovCenterOfMass?"ON":"OFF")   << "\n"; return; }
    if (key == GLFW_KEY_F4) { ovBoundingVolume = !ovBoundingVolume; std::cout << "[Overlay] bounding volume " << (ovBoundingVolume?"ON":"OFF") << "\n"; return; }
    if (key == GLFW_KEY_F5) { ovSleeping       = !ovSleeping;       std::cout << "[Overlay] sleeping markers " << (ovSleeping?"ON":"OFF")       << "\n"; return; }

    if (activeSceneManager == nullptr) return;

    // --- Environment controls (Part 3): affect the running simulation ------
    // G toggles aerodynamics, Shift+G toggles gravity. [ / ] adjust air
    // density. Left / Right arrows adjust wind along X.
    if (activeSolver && key == GLFW_KEY_G) {
        if (mods & GLFW_MOD_SHIFT) {
            activeSolver->gravityEnabled = !activeSolver->gravityEnabled;
            std::cout << "[Env] gravity " << (activeSolver->gravityEnabled?"ON":"OFF") << "\n";
        } else {
            activeSolver->aerodynamicsEnabled = !activeSolver->aerodynamicsEnabled;
            std::cout << "[Env] aerodynamics " << (activeSolver->aerodynamicsEnabled?"ON":"OFF") << "\n";
        }
        return;
    }
    if (activeSolver && key == GLFW_KEY_LEFT_BRACKET)  { activeSolver->airDensity = std::max(0.0f, activeSolver->airDensity - 0.2f); std::cout << "[Env] air density " << activeSolver->airDensity << " kg/m^3\n"; return; }
    if (activeSolver && key == GLFW_KEY_RIGHT_BRACKET) { activeSolver->airDensity += 0.2f; std::cout << "[Env] air density " << activeSolver->airDensity << " kg/m^3\n"; return; }
    if (activeSolver && key == GLFW_KEY_LEFT)  { activeSolver->windVelocity.x -= 1.0f; std::cout << "[Env] wind (" << activeSolver->windVelocity.x << ",0,0) m/s\n"; return; }
    if (activeSolver && key == GLFW_KEY_RIGHT) { activeSolver->windVelocity.x += 1.0f; std::cout << "[Env] wind (" << activeSolver->windVelocity.x << ",0,0) m/s\n"; return; }

    // --- Material assignment to the selected body (Part 3) -----------------
    // Keys 6..0 would collide with scene switching; use F6-F10 for materials.
    auto assignMaterial = [&](MaterialType mt) {
        std::vector<RigidBody>& bs = activeSceneManager->bodies();
        if (selectedBody >= 0 && selectedBody < static_cast<int>(bs.size())) {
            applyMaterial(bs[selectedBody], mt);
            std::cout << "[Material] body " << selectedBody << " -> " << materialName(mt)
                      << "  (mass=" << bs[selectedBody].mass << " kg)\n";
        } else {
            std::cout << "[Material] no body selected\n";
        }
    };
    if (key == GLFW_KEY_F6)  { assignMaterial(MaterialType::Steel);    return; }
    if (key == GLFW_KEY_F7)  { assignMaterial(MaterialType::Aluminum); return; }
    if (key == GLFW_KEY_F8)  { assignMaterial(MaterialType::Wood);     return; }
    if (key == GLFW_KEY_F9)  { assignMaterial(MaterialType::Rubber);   return; }
    if (key == GLFW_KEY_F10) { assignMaterial(MaterialType::Ice);      return; }

    // Refresh interaction/recorder state after any scene (re)load: the body
    // set changed, so clear the selection and the recording, and re-resolve
    // the sandbox pointer.
    auto onSceneChanged = [&]() {
        simulationPaused = true; // start each freshly loaded scene paused
        selectedBody = -1;
        draggingBody = false;
        applyingImpulse = false;
        activeSandbox = dynamic_cast<SandboxScene*>(activeSceneManager->activeScene());
        resetRecordingForStructuralChange();
        logStatus();
    };

    // Number keys 1-9: instantly switch to the corresponding registered scene.
    if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9) {
        if (activeSceneManager->loadSceneIndex(key - GLFW_KEY_1)) onSceneChanged();
        return;
    }

    // Scene navigation: Backspace restarts the current scene, N / B cycle.
    // (R now toggles recording and Space pauses, per the UI spec.)
    if (key == GLFW_KEY_BACKSPACE) { activeSceneManager->restartCurrentScene(); onSceneChanged(); return; }
    if (key == GLFW_KEY_N) { activeSceneManager->nextScene();     onSceneChanged(); return; }
    if (key == GLFW_KEY_B) { activeSceneManager->previousScene(); onSceneChanged(); return; }

    // -----------------------------------------------------------------------
    // Sandbox editing + recording export.
    //   C = spawn cube, V = spawn sphere (near the origin)
    //   X / Delete = delete the selected body
    //   E = export the recording to CSV
    // Structural edits clear the recording so object ids stay consistent.
    // -----------------------------------------------------------------------
    if (key == GLFW_KEY_E) {
        if (activeRecorder) {
            const char* path = "recording.csv";
            if (activeRecorder->exportCSV(path)) {
                std::cout << "[Recorder] exported " << activeRecorder->rowCount()
                          << " rows (" << activeRecorder->stepCount() << " steps) to "
                          << path << "\n";
            } else {
                std::cerr << "[Recorder] failed to write " << path << "\n";
            }
        }
        return;
    }

    // The remaining edits require the sandbox scene.
    if (activeSandbox == nullptr) return;

    if (key == GLFW_KEY_C) {
        // Spawn a cube slightly above the ground near the origin.
        activeSandbox->spawnCube(glm::vec3(0.0f, 3.0f, 0.0f));
        resetRecordingForStructuralChange();
        return;
    }
    if (key == GLFW_KEY_V) {
        activeSandbox->spawnSphere(glm::vec3(0.0f, 3.0f, 0.0f));
        resetRecordingForStructuralChange();
        return;
    }
    if (key == GLFW_KEY_X || key == GLFW_KEY_DELETE) {
        if (selectedBody >= 0 && activeSandbox->deleteBody(selectedBody)) {
            selectedBody = -1;
            draggingBody = false;
            resetRecordingForStructuralChange();
        }
        return;
    }
}

// ===========================================================================
// Physics scenes now live in Scene.h / SceneManager.h / Scenes.h. The old
// hardcoded spawn* functions were replaced by the modular scene system; this
// file only wires the SceneManager into the render/step/input loop.
// ===========================================================================

#if 0 // --- legacy hardcoded demos (superseded by the Scene Manager) ---
void setCubeMassProperties(RigidBody& body, float mass) {
    body.mass = mass;
    body.inverseMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    body.updateInertiaTensor();
}

std::vector<RigidBody> spawnExplosion() {
    // A cluster of cubes sitting together, then an "explosion" gives each one
    // an outward radial velocity from the center — like a bomb going off.
    // No magic forces needed: we just set initial velocities pointing away
    // from the origin, scaled by a blast strength. Physics handles the rest
    // (collisions on the way out, gravity arcing them back down, pile settling).
    std::vector<RigidBody> bodies;

    const float blastStrength = 24.0f;
    const float g = 1.1f; // spacing

    // 3x3x3 cube cluster centered at the origin, elevated so they arc nicely
    for (int x = -1; x <= 1; ++x) {
        for (int y = 0; y <= 2; ++y) {
            for (int z = -1; z <= 1; ++z) {
                RigidBody cube;
                cube.position = glm::vec3(
                    static_cast<float>(x) * g,
                    1.5f + static_cast<float>(y) * g,
                    static_cast<float>(z) * g
                );

                // Radial velocity away from the cluster center
                glm::vec3 dir = cube.position - glm::vec3(0.0f, 2.5f, 0.0f);
                const float dist = glm::length(dir);
                if (dist > 0.01f) {
                    dir /= dist;
                } else {
                    dir = glm::vec3(0.0f, 1.0f, 0.0f);
                }
                // Strength falls off slightly with distance for a natural look
                cube.velocity = dir * blastStrength * (1.0f - 0.2f * dist);

                // Add some random-ish spin for drama
                cube.angularVelocity = glm::vec3(
                    static_cast<float>(x) * 3.0f,
                    static_cast<float>(z) * 2.0f,
                    static_cast<float>(y) * -2.5f
                );

                setCubeMassProperties(cube, 1.0f);
                cube.restitution = 0.3f;
                cube.friction = 0.4f;
                bodies.push_back(cube);
            }
        }
    }

    return bodies;
}

std::vector<RigidBody> spawnStableTower() {
    // 6 cubes stacked precisely. Tests resting contact stability,
    // accumulated impulse convergence, and warm starting.
    // Expected: tower settles and remains perfectly stationary indefinitely.
    std::vector<RigidBody> bodies;

    for (int i = 0; i < 6; ++i) {
        RigidBody cube;
        cube.position = glm::vec3(0.0f, 0.5f + static_cast<float>(i) * 1.05f, 0.0f);
        cube.velocity = glm::vec3(0.0f);
        cube.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.1f;
        cube.friction = 0.6f;
        bodies.push_back(cube);
    }

    return bodies;
}

std::vector<RigidBody> spawnDominoSpiral() {
    // Hundreds of dominoes on an Archimedean spiral, stepped by constant arc
    // length so spacing stays reliable at every radius. The innermost domino
    // is pre-tilted past its tipping point; the cascade travels outward.
    std::vector<RigidBody> bodies;

    const int count = 150;                           // tune freely
    const glm::vec3 dominoScale(0.15f, 0.9f, 0.45f); // thin, tall, wide
    const float halfHeight = dominoScale.y * 0.5f;
    const float halfThick  = dominoScale.x * 0.5f;

    const float spacing = 0.45f; // center-to-center along the spiral
    const float r0 = 1.5f;       // inner radius
    const float b  = 0.18f;      // radial growth per radian (loop gap = 2*pi*b)
    const glm::vec3 up(0.0f, 1.0f, 0.0f);

    // Walk the spiral, placing a domino every `spacing` units of arc length
    std::vector<glm::vec3> pos;
    pos.reserve(count);
    float theta = 0.0f;
    for (int i = 0; i < count; ++i) {
        const float r = r0 + b * theta;
        pos.push_back(glm::vec3(r * std::cos(theta), halfHeight, r * std::sin(theta)));
        theta += spacing / std::sqrt(r * r + b * b); // constant arc-length step
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
        setCubeMassProperties(domino, 0.5f);
        domino.restitution = 0.03f;
        domino.friction = 0.5f;
        bodies.push_back(domino);
    }

    // Start the chain with a decisive push, not a metastable near-balance release.
    // A domino 4-5 deg past its ~9.5 deg balance angle tips so slowly that its
    // angular velocity stays under the sleep threshold and it falls asleep
    // mid-tip. An initial angular velocity well above SLEEP_ANGULAR_THRESHOLD
    // (0.20 rad/s) keeps it "awake" and carries it past the balance point, after
    // which gravity accelerates the topple on its own.
    glm::vec3 tangent0 = glm::normalize(pos[1] - pos[0]);
    glm::vec3 tiltAxis = glm::normalize(glm::cross(up, tangent0));
    const float tiltAngle = glm::radians(14.0f);
    bodies[0].orientation = glm::angleAxis(tiltAngle, tiltAxis) * bodies[0].orientation;
    bodies[0].position.y = halfHeight * std::cos(tiltAngle) + halfThick * std::sin(tiltAngle);
    bodies[0].angularVelocity = tiltAxis * 3.0f; // the "flick" that starts the cascade


    return bodies;
}

std::vector<RigidBody> spawnNewtonsCradle() {
    // Newton's cradle: 5 cubes between two immovable walls.
    // Walls are spaced further out so you can see the end cubes separate
    // before bouncing back. Perfectly elastic, zero friction.
    std::vector<RigidBody> bodies;

    // Left wall (infinite mass)
    RigidBody leftWall;
    leftWall.position = glm::vec3(-5.0f, 0.5f, 0.0f);
    leftWall.velocity = glm::vec3(0.0f);
    leftWall.scale = glm::vec3(0.3f, 1.0f, 1.5f);
    setCubeMassProperties(leftWall, 1.0f);
    leftWall.inverseMass = 0.0f;
    leftWall.inverseInertiaLocal = glm::mat3(0.0f);
    leftWall.inverseInertiaWorld = glm::mat3(0.0f);
    leftWall.restitution = 0.9f;
    leftWall.friction = 0.1f;
    bodies.push_back(leftWall);

    // 5 dynamic cubes, spaced with small gaps so you see separation
    for (int i = 0; i < 5; ++i) {
        RigidBody cube;
        cube.position = glm::vec3(-2.0f + static_cast<float>(i) * 1.05f, 0.5f, 0.0f);
        cube.velocity = glm::vec3(0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 1.0f;
        cube.friction = 0.0f;
        bodies.push_back(cube);
    }

    // Right wall (infinite mass)
    RigidBody rightWall;
    rightWall.position = glm::vec3(5.0f, 0.5f, 0.0f);
    rightWall.velocity = glm::vec3(0.0f);
    rightWall.scale = glm::vec3(0.3f, 1.0f, 1.5f);
    setCubeMassProperties(rightWall, 1.0f);
    rightWall.inverseMass = 0.0f;
    rightWall.inverseInertiaLocal = glm::mat3(0.0f);
    rightWall.inverseInertiaWorld = glm::mat3(0.0f);
    rightWall.restitution = 1.0f;
    rightWall.friction = 0.0f;
    bodies.push_back(rightWall);

    // Give the leftmost dynamic cube a push
    bodies[1].velocity = glm::vec3(4.0f, 0.0f, 0.0f);

    return bodies;
}

std::vector<RigidBody> spawnAvalanche() {
    // 10 cubes dropped in a staggered cluster from various heights.
    // Tests many simultaneous OBB contacts, pile formation, friction settling.
    // Expected: cubes collide mid-air and on the ground, settling into a stable pile.
    std::vector<RigidBody> bodies;

    const glm::vec3 positions[] = {
        {-0.5f, 3.0f,  0.3f}, { 0.6f, 3.5f, -0.2f}, {-0.2f, 4.0f,  0.5f},
        { 0.8f, 4.5f,  0.1f}, {-0.7f, 5.0f, -0.4f}, { 0.3f, 5.5f,  0.6f},
        {-0.4f, 6.0f, -0.1f}, { 0.5f, 6.5f,  0.3f}, {-0.1f, 7.0f, -0.5f},
        { 0.2f, 7.5f,  0.0f}
    };

    const float angles[] = {0.0f, 15.0f, -10.0f, 30.0f, -20.0f, 5.0f, -35.0f, 25.0f, 10.0f, -5.0f};

    for (int i = 0; i < 10; ++i) {
        RigidBody cube;
        cube.position = positions[i];
        cube.velocity = glm::vec3(0.0f);
        cube.orientation = glm::angleAxis(glm::radians(angles[i]), glm::normalize(glm::vec3(0.3f, 0.0f, 1.0f)));
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.2f;
        cube.friction = 0.5f;
        bodies.push_back(cube);
    }

    return bodies;
}

std::vector<RigidBody> spawnBilliards() {
    // A triangle rack of cubes struck by a cue cube, enclosed by four
    // immovable rails like a pool table so the cubes bounce off the walls.
    std::vector<RigidBody> bodies;

    const float gap = 1.05f;
    const glm::vec3 triBase(1.5f, 0.5f, 0.0f);

    // --- Rack: triangle of 6 cubes ---
    // Row 1 (3 cubes)
    for (int i = 0; i < 3; ++i) {
        RigidBody cube;
        cube.position = triBase + glm::vec3(0.0f, 0.0f, (static_cast<float>(i) - 1.0f) * gap);
        cube.velocity = glm::vec3(0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.6f; // a bit bouncier so wall rebounds stay lively
        cube.friction = 0.001f;
        bodies.push_back(cube);
    }
    // Row 2 (2 cubes)
    for (int i = 0; i < 2; ++i) {
        RigidBody cube;
        cube.position = triBase + glm::vec3(gap, 0.0f, (static_cast<float>(i) - 0.5f) * gap);
        cube.velocity = glm::vec3(0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.6f;
        cube.friction = 0.001f;
        bodies.push_back(cube);
    }
    // Row 3 (1 cube, the apex)
    {
        RigidBody cube;
        cube.position = triBase + glm::vec3(2.0f * gap, 0.0f, 0.0f);
        cube.velocity = glm::vec3(0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.6f;
        cube.friction = 0.001f;
        bodies.push_back(cube);
    }

    // --- Pool table rails (four immovable walls forming a rectangle) ---
    // Table interior spans roughly x in [-8, 8], z in [-5, 5].
    const float halfX = 8.0f;
    const float halfZ = 5.0f;
    const float railThickness = 0.5f;
    const float railHeight = 1.0f;

    auto rail = [&](glm::vec3 pos, glm::vec3 scale) {
        RigidBody wall;
        wall.position = pos;
        wall.scale = scale;
        wall.velocity = glm::vec3(0.0f);
        setCubeMassProperties(wall, 1.0f);
        wall.inverseMass = 0.0f; // immovable
        wall.inverseInertiaLocal = glm::mat3(0.0f);
        wall.inverseInertiaWorld = glm::mat3(0.0f);
        wall.restitution = 0.7f; // rails bounce the cubes back
        wall.friction = 0.2f;
        bodies.push_back(wall);
    };

    // Long rails (top and bottom in z), spanning the full x width
    rail(glm::vec3(0.0f, 0.5f,  halfZ + railThickness * 0.5f), glm::vec3(halfX * 2.0f + railThickness * 2.0f, railHeight, railThickness));
    rail(glm::vec3(0.0f, 0.5f, -halfZ - railThickness * 0.5f), glm::vec3(halfX * 2.0f + railThickness * 2.0f, railHeight, railThickness));
    // End rails (left and right in x), spanning the full z depth
    rail(glm::vec3( halfX + railThickness * 0.5f, 0.5f, 0.0f), glm::vec3(railThickness, railHeight, halfZ * 2.0f));
    rail(glm::vec3(-halfX - railThickness * 0.5f, 0.5f, 0.0f), glm::vec3(railThickness, railHeight, halfZ * 2.0f));

    // --- Cue cube — launched from the right toward the rack ---
    RigidBody cue;
    cue.position = glm::vec3(6.0f, 0.5f, 0.1f); // slight z-offset for asymmetric break
    cue.velocity = glm::vec3(-2400.0f, 0.0f, 0.0f);
    setCubeMassProperties(cue, 1.0f);
    cue.restitution = 0.6f;
    cue.friction = 0.2f;
    bodies.push_back(cue);

    return bodies;
}

std::vector<RigidBody> spawnElasticVsInelastic() {
    // Two identical collisions side by side: one elastic (e=1), one inelastic (e=0.1).
    // Demonstrates: how restitution controls energy retention.
    // Expected: elastic pair keeps bouncing; inelastic pair sticks together and stops.
    std::vector<RigidBody> bodies;

    // --- Elastic pair (z = -1.5) ---
    RigidBody eA;
    eA.position = glm::vec3(-3.0f, 0.5f, -1.5f);
    eA.velocity = glm::vec3(3.0f, 0.0f, 0.0f);
    setCubeMassProperties(eA, 1.0f);
    eA.restitution = 1.0f;
    eA.friction = 0.0f;
    bodies.push_back(eA);

    RigidBody eB;
    eB.position = glm::vec3(0.0f, 0.5f, -1.5f);
    eB.velocity = glm::vec3(0.0f);
    setCubeMassProperties(eB, 1.0f);
    eB.restitution = 1.0f;
    eB.friction = 0.0f;
    bodies.push_back(eB);

    // --- Inelastic pair (z = +1.5) ---
    RigidBody iA;
    iA.position = glm::vec3(-3.0f, 0.5f, 1.5f);
    iA.velocity = glm::vec3(3.0f, 0.0f, 0.0f);
    setCubeMassProperties(iA, 1.0f);
    iA.restitution = 0.1f;
    iA.friction = 0.0f;
    bodies.push_back(iA);

    RigidBody iB;
    iB.position = glm::vec3(0.0f, 0.5f, 1.5f);
    iB.velocity = glm::vec3(0.0f);
    setCubeMassProperties(iB, 1.0f);
    iB.restitution = 0.1f;
    iB.friction = 0.0f;
    bodies.push_back(iB);

    return bodies;
}

std::vector<RigidBody> spawnWreckingBall() {
    std::vector<RigidBody> bodies;

    const float g = 1.1f; // 10% gap between bricks — prevents self-collapse

    auto brick = [&](float x, float y, float z) {
        RigidBody b;
        b.position = glm::vec3(x, y, z);
        b.velocity = glm::vec3(0.0f);
        setCubeMassProperties(b, 1.0f);
        b.restitution = 0.15f;
        b.friction = 0.6f;
        bodies.push_back(b);
    };

    // Left tower (2x2 base, 4 tall) at z = -2.5
    for (int row = 0; row < 4; ++row) {
        brick(-0.5f * g, 0.5f + static_cast<float>(row) * g, -2.5f - 0.5f * g);
        brick( 0.5f * g, 0.5f + static_cast<float>(row) * g, -2.5f - 0.5f * g);
        brick(-0.5f * g, 0.5f + static_cast<float>(row) * g, -2.5f + 0.5f * g);
        brick( 0.5f * g, 0.5f + static_cast<float>(row) * g, -2.5f + 0.5f * g);
    }

    // Right tower (2x2 base, 4 tall) at z = +2.5
    for (int row = 0; row < 4; ++row) {
        brick(-0.5f * g, 0.5f + static_cast<float>(row) * g, 2.5f - 0.5f * g);
        brick( 0.5f * g, 0.5f + static_cast<float>(row) * g, 2.5f - 0.5f * g);
        brick(-0.5f * g, 0.5f + static_cast<float>(row) * g, 2.5f + 0.5f * g);
        brick( 0.5f * g, 0.5f + static_cast<float>(row) * g, 2.5f + 0.5f * g);
    }

    // Connecting wall (single-thick, 2 tall, between towers)
    for (int row = 0; row < 2; ++row) {
        brick(0.0f, 0.5f + static_cast<float>(row) * g, -1.1f);
        brick(0.0f, 0.5f + static_cast<float>(row) * g,  0.0f);
        brick(0.0f, 0.5f + static_cast<float>(row) * g,  1.1f);
    }

    // Boulder
    RigidBody boulder;
    boulder.scale = glm::vec3(1.9f, 1.9f, 1.9f);
    boulder.position = glm::vec3(-10.0f, 3.0f, 1.0f);
    boulder.velocity = glm::vec3(24.0f, -2.0f, 0.0f);
    setCubeMassProperties(boulder, 8.0f);
    boulder.restitution = 0.25f;
    boulder.friction = 0.4f;
    bodies.push_back(boulder);

    return bodies;
}

std::vector<RigidBody> spawnInertiaDemo() {
    // Demonstrates Newton's first law (inertia): two cubes stacked on top of
    // each other move together to the right. The bottom cube slams into a
    // heavy immovable wall and stops dead. The top cube, with nothing acting
    // on it horizontally, keeps sailing forward at its original speed.
    //
    // This is the classic "tablecloth trick" or "brick on a cart hitting a wall."
    // The top cube has low friction so the bottom stopping doesn't drag it along.
    std::vector<RigidBody> bodies;

    // Bottom cube (the "cart") — moving right, will hit the wall
    RigidBody bottom;
    bottom.position = glm::vec3(-4.0f, 0.5f, 0.0f);
    bottom.velocity = glm::vec3(5.0f, 0.0f, 0.0f);
    setCubeMassProperties(bottom, 1.0f);
    bottom.restitution = 0.05f; // nearly inelastic so it stops dead on impact
    bottom.friction = 0.00f;    // low friction with the top cube
    bodies.push_back(bottom);

    // Top cube (the "passenger") — same velocity, riding on top
    RigidBody top;
    top.position = glm::vec3(-4.0f, 1.5f, 0.0f); // sitting on the bottom cube
    top.velocity = glm::vec3(5.0f, 0.0f, 0.0f);  // same initial speed
    setCubeMassProperties(top, 1.0f);
    top.restitution = 0.1f;
    top.friction = 0.05f; // low friction so it slides off cleanly when bottom stops
    bodies.push_back(top);

    // Wall (infinite mass) — the obstacle the bottom cube hits
    RigidBody wall;
    wall.position = glm::vec3(0.0f, 0.4f, 0.0f);
    wall.scale = glm::vec3(0.4f, 0.8f, 2.0f); // thin, tall, wide
    wall.velocity = glm::vec3(0.0f);
    setCubeMassProperties(wall, 1.0f);
    wall.inverseMass = 0.0f;
    wall.inverseInertiaLocal = glm::mat3(0.0f);
    wall.inverseInertiaWorld = glm::mat3(0.0f);
    wall.restitution = 0.05f;
    wall.friction = 0.8f;
    bodies.push_back(wall);

    return bodies;
}

// ===========================================================================
// Sphere helper
// ===========================================================================

void setSphereMassProperties(RigidBody& body, float mass, float radius) {
    body.shape = ShapeType::Sphere;
    body.radius = radius;
    body.mass = mass;
    body.inverseMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    body.scale = glm::vec3(radius * 2.0f); // visual scale matches collision radius
    body.updateInertiaTensor();
}

// ===========================================================================
// Sphere Demo Scene
// ===========================================================================

std::vector<RigidBody> spawnSphereDemo() {
    std::vector<RigidBody> bodies;

    // (a) Sphere falling from height — tests sphere-floor contact
    RigidBody s1;
    s1.position = glm::vec3(0.0f, 3.0f, 0.0f);
    s1.velocity = glm::vec3(0.0f);
    setSphereMassProperties(s1, 1.0f, 0.5f);
    s1.restitution = 0.6f;
    s1.friction = 0.4f;
    bodies.push_back(s1);

    // (b) Sphere rolling along the floor — tests friction + angular coupling
    RigidBody s2;
    s2.position = glm::vec3(-4.0f, 0.5f, 0.0f);
    s2.velocity = glm::vec3(3.0f, 0.0f, 0.0f);
    setSphereMassProperties(s2, 1.0f, 0.5f);
    s2.restitution = 0.1f;
    s2.friction = 0.6f;
    bodies.push_back(s2);

    // (c) Two spheres colliding head-on — tests sphere-sphere
    RigidBody s3;
    s3.position = glm::vec3(3.0f, 0.5f, 2.0f);
    s3.velocity = glm::vec3(2.0f, 0.0f, 0.0f);
    setSphereMassProperties(s3, 1.0f, 0.4f);
    s3.restitution = 0.8f;
    s3.friction = 0.3f;
    bodies.push_back(s3);

    RigidBody s4;
    s4.position = glm::vec3(6.0f, 0.4f, 2.0f);
    s4.velocity = glm::vec3(-2.0f, 0.0f, 0.0f);
    setSphereMassProperties(s4, 1.0f, 0.4f);
    s4.restitution = 0.8f;
    s4.friction = 0.3f;
    bodies.push_back(s4);

    // (d) Sphere hitting a stationary cube — tests sphere-box
    RigidBody cube;
    cube.position = glm::vec3(0.0f, 0.5f, -3.0f);
    cube.velocity = glm::vec3(0.0f);
    setCubeMassProperties(cube, 2.0f);
    cube.restitution = 0.3f;
    cube.friction = 0.5f;
    bodies.push_back(cube);

    RigidBody s5;
    s5.position = glm::vec3(-3.0f, 0.5f, -3.0f);
    s5.velocity = glm::vec3(4.0f, 0.0f, 0.0f);
    setSphereMassProperties(s5, 1.0f, 0.5f);
    s5.restitution = 0.5f;
    s5.friction = 0.4f;
    bodies.push_back(s5);

    // (e) Multiple spheres dropped in a cluster — tests pile settling
    for (int i = 0; i < 5; ++i) {
        RigidBody s;
        s.position = glm::vec3(-1.0f + i * 0.6f, 2.0f + i * 0.8f, 4.0f);
        s.velocity = glm::vec3(0.0f);
        setSphereMassProperties(s, 0.8f, 0.3f);
        s.restitution = 0.2f;
        s.friction = 0.5f;
        bodies.push_back(s);
    }

    return bodies;
}

// ===========================================================================
// Aerodynamics Demo: Terminal Velocity
//
// Enables the physically based drag model and drops several bodies from a
// great height so their terminal velocities (where drag balances weight) can
// be observed emerging naturally. Nothing about the terminal speed is imposed;
// it falls out of  m g = 1/2 rho Cd A v^2.
//
//   left  : small sphere vs large sphere (larger area -> slower)
//   middle: light sphere vs heavy sphere (more weight -> faster)
//   right : sphere vs cube (higher Cd*A on the cube -> slower)
// ===========================================================================
std::vector<RigidBody> spawnAeroTerminalVelocityDemo(PhysicsSolver& solver) {
    std::vector<RigidBody> bodies;
    bodies.reserve(8);

    // Turn on the aerodynamic environment. Sea-level air, still (no wind).
    solver.aerodynamicsEnabled = true;
    solver.airDensity = 1.225f;              // kg/m^3
    solver.windVelocity = glm::vec3(0.0f);   // still air

    const float dropY = 30.0f; // start high so terminal velocity is reached

    // --- Size comparison: small vs large sphere (same mass, same Cd) ---
    RigidBody smallSphere;
    smallSphere.position = glm::vec3(-4.0f, dropY, 0.0f);
    setSphereMassProperties(smallSphere, 2.0f, 0.25f);
    smallSphere.dragCoefficient = 0.47f; // smooth sphere
    smallSphere.restitution = 0.2f; smallSphere.friction = 0.4f;
    bodies.push_back(smallSphere);

    RigidBody largeSphere;
    largeSphere.position = glm::vec3(-2.0f, dropY, 0.0f);
    setSphereMassProperties(largeSphere, 2.0f, 0.5f); // 2x radius -> ~1/2 v_terminal
    largeSphere.dragCoefficient = 0.47f;
    largeSphere.restitution = 0.2f; largeSphere.friction = 0.4f;
    bodies.push_back(largeSphere);

    // --- Mass comparison: light vs heavy sphere (same size, same Cd) ---
    RigidBody lightSphere;
    lightSphere.position = glm::vec3(0.0f, dropY, 0.0f);
    setSphereMassProperties(lightSphere, 1.0f, 0.4f);
    lightSphere.dragCoefficient = 0.47f;
    lightSphere.restitution = 0.2f; lightSphere.friction = 0.4f;
    bodies.push_back(lightSphere);

    RigidBody heavySphere;
    heavySphere.position = glm::vec3(2.0f, dropY, 0.0f);
    setSphereMassProperties(heavySphere, 4.0f, 0.4f); // 4x mass -> ~2x v_terminal
    heavySphere.dragCoefficient = 0.47f;
    heavySphere.restitution = 0.2f; heavySphere.friction = 0.4f;
    bodies.push_back(heavySphere);

    // --- Shape comparison: sphere vs cube (same mass) ---
    RigidBody shapeSphere;
    shapeSphere.position = glm::vec3(4.0f, dropY, 0.0f);
    setSphereMassProperties(shapeSphere, 2.0f, 0.5f);
    shapeSphere.dragCoefficient = 0.47f;
    shapeSphere.restitution = 0.2f; shapeSphere.friction = 0.4f;
    bodies.push_back(shapeSphere);

    RigidBody shapeCube;
    shapeCube.scale = glm::vec3(1.0f);
    shapeCube.position = glm::vec3(6.0f, dropY, 0.0f);
    setCubeMassProperties(shapeCube, 2.0f);
    shapeCube.dragCoefficient = 1.05f; // bluff cube face -> higher Cd, slower fall
    shapeCube.restitution = 0.1f; shapeCube.friction = 0.5f;
    bodies.push_back(shapeCube);

    return bodies;
}

// ===========================================================================
// Aerodynamics Demo: Wind & Weather-Vane
//
// A steady horizontal wind blows across the scene. Bodies feel drag from the
// RELATIVE airflow (wind - velocity), so light objects are carried downwind
// while heavy ones resist. One box is given a center-of-pressure offset (a
// tail); the off-COM aerodynamic force produces a torque that turns it to
// face into the wind, exactly like a weather-vane.
// ===========================================================================
std::vector<RigidBody> spawnAeroWindDemo(PhysicsSolver& solver) {
    std::vector<RigidBody> bodies;
    bodies.reserve(8);

    solver.aerodynamicsEnabled = true;
    solver.airDensity = 1.225f;
    solver.windVelocity = glm::vec3(12.0f, 0.0f, 0.0f); // 12 m/s wind along +x

    // Light beach-ball: strongly carried by the wind.
    RigidBody beachBall;
    beachBall.position = glm::vec3(-6.0f, 6.0f, 0.0f);
    setSphereMassProperties(beachBall, 0.3f, 0.5f); // large area, tiny mass
    beachBall.dragCoefficient = 0.6f;
    beachBall.restitution = 0.5f; beachBall.friction = 0.4f;
    bodies.push_back(beachBall);

    // Heavy ball-bearing: same size, barely nudged by the wind.
    RigidBody heavyBall;
    heavyBall.position = glm::vec3(-6.0f, 3.0f, 2.0f);
    setSphereMassProperties(heavyBall, 8.0f, 0.5f);
    heavyBall.dragCoefficient = 0.47f;
    heavyBall.restitution = 0.3f; heavyBall.friction = 0.5f;
    bodies.push_back(heavyBall);

    // Weather-vane: a light box with its center of pressure behind the COM
    // (a tail along local -x). Started 50 deg off the wind; the aero torque
    // rotates it to point into the airflow.
    RigidBody vane;
    vane.scale = glm::vec3(1.2f, 0.3f, 0.3f); // long, thin (arrow-like)
    vane.position = glm::vec3(0.0f, 5.0f, 0.0f);
    setCubeMassProperties(vane, 1.0f);
    vane.dragCoefficient = 1.0f;
    vane.aeroCenterOffset = glm::vec3(-0.5f, 0.0f, 0.0f); // tail behind COM
    vane.orientation = glm::angleAxis(glm::radians(50.0f), glm::vec3(0, 0, 1));
    vane.restitution = 0.1f; vane.friction = 0.5f;
    bodies.push_back(vane);

    // A tumbling paper-like plate: no offset, so it does not self-align --
    // it simply gets pushed downwind and drifts (contrast with the vane).
    RigidBody plate;
    plate.scale = glm::vec3(1.5f, 0.1f, 1.5f);
    plate.position = glm::vec3(0.0f, 8.0f, -2.5f);
    setCubeMassProperties(plate, 0.5f);
    plate.dragCoefficient = 1.2f;
    plate.orientation = glm::angleAxis(glm::radians(35.0f), glm::vec3(0, 0, 1));
    plate.restitution = 0.1f; plate.friction = 0.5f;
    bodies.push_back(plate);

    return bodies;
}

std::vector<RigidBody> spawnSlopeDemo(PhysicsSolver& solver) {
    std::vector<RigidBody> bodies;

    // --- Physical slope: 20-degree incline ---
    const float slopeAngle = 20.0f;
    const float rad = glm::radians(slopeAngle);
    PhysicsSolver::StaticPlane slope;
    slope.point = glm::vec3(0.0f, 2.0f, 0.0f); // plane passes through this point
    slope.normal = glm::vec3(-std::sin(rad), std::cos(rad), 0.0f); // tilted normal
    slope.friction = 0.4f;
    slope.restitution = 0.2f;
    // Finite bounds matching the visual ramp (8m long × 3m wide)
    slope.halfExtent = glm::vec2(4.0f, 1.5f);
    // Tangent axes: t1 = down-slope direction, t2 = across-slope (Z axis)
    slope.tangent1 = glm::normalize(glm::vec3(std::cos(rad), std::sin(rad), 0.0f));
    slope.tangent2 = glm::vec3(0.0f, 0.0f, 1.0f);
    solver.planes.push_back(slope);

    // Visual ramp (a long thin static box tilted to match the slope)
    RigidBody ramp;
    ramp.scale = glm::vec3(8.0f, 0.2f, 3.0f); // long, thin, wide
    ramp.position = glm::vec3(0.0f, 2.0f, 0.0f);
    ramp.orientation = glm::angleAxis(-rad, glm::vec3(0, 0, 1));
    ramp.velocity = glm::vec3(0.0f);
    ramp.angularVelocity = glm::vec3(0.0f);
    ramp.inverseMass = 0.0f;
    ramp.inverseInertiaLocal = glm::mat3(0.0f);
    ramp.inverseInertiaWorld = glm::mat3(0.0f);
    ramp.restitution = 0.2f;
    ramp.friction = 0.4f;
    bodies.push_back(ramp);

    // (a) Sphere at the top of the slope — should roll/slide down
    RigidBody s1;
    s1.position = slope.point + slope.normal * 0.4f + glm::vec3(-2.5f, 1.0f, 0.0f);
    s1.velocity = glm::vec3(0.0f);
    setSphereMassProperties(s1, 1.0f, 0.4f);
    s1.restitution = 0.2f;
    s1.friction = 0.5f;
    bodies.push_back(s1);

    // (b) Another sphere, different size
    RigidBody s2;
    s2.position = slope.point + slope.normal * 0.3f + glm::vec3(-2.0f, 0.8f, 0.8f);
    s2.velocity = glm::vec3(0.0f);
    setSphereMassProperties(s2, 0.5f, 0.25f);
    s2.restitution = 0.3f;
    s2.friction = 0.6f;
    bodies.push_back(s2);

    // (c) Cube sliding down the slope
    RigidBody cube1;
    cube1.scale = glm::vec3(0.6f);
    cube1.position = slope.point + slope.normal * 0.35f + glm::vec3(-1.5f, 0.6f, -0.5f);
    cube1.velocity = glm::vec3(0.0f);
    cube1.orientation = glm::angleAxis(-rad, glm::vec3(0, 0, 1)); // aligned with slope
    setCubeMassProperties(cube1, 1.0f);
    cube1.restitution = 0.1f;
    cube1.friction = 0.3f; // low friction -> slides
    bodies.push_back(cube1);

    // (d) Cube with high friction — should stay put on the slope
    RigidBody cube2;
    cube2.scale = glm::vec3(0.5f);
    cube2.position = slope.point + slope.normal * 0.3f + glm::vec3(-0.5f, 0.3f, 0.5f);
    cube2.velocity = glm::vec3(0.0f);
    cube2.orientation = glm::angleAxis(-rad, glm::vec3(0, 0, 1));
    setCubeMassProperties(cube2, 1.0f);
    cube2.restitution = 0.1f;
    cube2.friction = 0.9f; // high friction -> should hold (tan20°=0.36 < 0.9)
    bodies.push_back(cube2);

    // (e) Sphere dropped from above onto the slope — bounces then rolls
    RigidBody s3;
    s3.position = glm::vec3(0.0f, 5.0f, 0.0f);
    s3.velocity = glm::vec3(0.0f);
    setSphereMassProperties(s3, 1.5f, 0.5f);
    s3.restitution = 0.5f;
    s3.friction = 0.4f;
    bodies.push_back(s3);

    return bodies;
}
#endif // --- end legacy hardcoded demos ---

// ===========================================================================
// Main
// ===========================================================================

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    const GLFWvidmode* VideoMode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    const auto ScreenHeight = VideoMode->height;
    GLFWwindow* window = glfwCreateWindow(ScreenHeight / 2, ScreenHeight / 2, "Physics Engine — OBB Demos", NULL, NULL);

    if (window == NULL) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glfwSetFramebufferSizeCallback(window, framebufferSizeChanged);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);

    Render renderer;
    Cube cube(1.0f);
    Sphere sphereMesh; // unit-radius sphere mesh for rendering
    Ground ground(20.0f);

    Camera camera(11.0f, -55.0f, 18.0f);
    activeCamera = &camera;

    PhysicsSolver physicsSolver;
    physicsSolver.captureTelemetry = true; // fill lastTelemetry for stats/overlays
    activeSolver = &physicsSolver;         // env controls reach it from callbacks

    // -----------------------------------------------------------------
    // Modular scenes. Press number keys 1-9 to switch instantly;
    // Backspace = restart, N / B = next / previous scene, Space = play/pause.
    // -----------------------------------------------------------------
    // Experiment library. Grouped Mechanics / Structures / Dynamics + the
    // interactive sandbox. Number keys 1-9 load the first nine; the rest are
    // reachable with N / B (next / previous).
    EmptySandboxScene    emptySandbox;   // Mechanics
    DominoSpiralScene    dominoSpiral;
    NewtonsCradleScene   newtonsCradle;
    AtwoodMachineScene   atwood;
    InclinedPlaneScene   inclinedPlane;
    RopeBridgeScene      ropeBridge;      // Structures
    SuspensionBridgeScene suspensionBridge;
    CantileverBeamScene  cantilever;
    HangingChainScene    hangingChain;
    DoublePendulumScene  doublePendulum;  // Dynamics
    SpringPendulumScene  springPendulum;
    TrebuchetScene       trebuchet;
    SandboxScene         sandbox;         // Interactive

    SceneManager sceneManager(physicsSolver);
    sceneManager.registerScene("Empty Sandbox",     &emptySandbox);     // 1
    sceneManager.registerScene("Domino Spiral",     &dominoSpiral);     // 2
    sceneManager.registerScene("Newton's Cradle",   &newtonsCradle);    // 3
    sceneManager.registerScene("Atwood Machine",    &atwood);           // 4
    sceneManager.registerScene("Inclined Plane",    &inclinedPlane);    // 5
    sceneManager.registerScene("Rope Bridge",       &ropeBridge);       // 6
    sceneManager.registerScene("Suspension Bridge", &suspensionBridge); // 7
    sceneManager.registerScene("Cantilever Beam",   &cantilever);       // 8
    sceneManager.registerScene("Hanging Chain",     &hangingChain);     // 9
    sceneManager.registerScene("Double Pendulum",   &doublePendulum);   // N/B
    sceneManager.registerScene("Spring Pendulum",   &springPendulum);   // N/B
    sceneManager.registerScene("Trebuchet",         &trebuchet);        // N/B
    sceneManager.registerScene("Sandbox",           &sandbox);          // N/B
    activeSceneManager = &sceneManager;

    // In-memory simulation recorder (captures every timestep; never mutates
    // physics). Exported to CSV on demand with the E key.
    SimulationRecorder recorder;
    activeRecorder = &recorder;
    recorder.setEnabled(recordingEnabled);

    // Trajectory predictor. Try to load a model from disk; if none exists (the
    // default), predict() falls back to a physics rollout. Model loading is
    // isolated inside PathPredictor and needs no ONNX to be installed.
    PathPredictor predictor;
    activePredictor = &predictor;
    if (predictor.loadModel("model.onnx")) {
        std::cout << "[Predictor] loaded model: " << predictor.modelPath() << "\n";
    } else {
        std::cout << "[Predictor] no model loaded; using physics rollout.\n";
    }

    sceneManager.loadScene("Sandbox"); // start in the interactive sandbox
    activeSandbox = &sandbox;
    recorder.clear();
    logStatus();

    // Fixed timestep
    static constexpr float FIXED_DT = 1.0f / 60.0f;
    float accumulator = 0.0f;
    float lastTime = static_cast<float>(glfwGetTime());
    float lastDebugTime = static_cast<float>(glfwGetTime());

    while (!glfwWindowShouldClose(window)) {

        // Frame time
        const float currentTime = static_cast<float>(glfwGetTime());
        float frameTime = currentTime - lastTime;
        frameTime = std::min(frameTime, 0.25f);
        lastTime = currentTime;
        accumulator += frameTime;

        // The live world is owned by the active scene.
        std::vector<RigidBody>& bodies = sceneManager.bodies();

        // Physics loop. Runs while unpaused; while paused, a single '.' press
        // advances exactly one fixed step (Simulation panel single-frame step).
        if (!simulationPaused) {
            while (accumulator >= FIXED_DT) {
                sceneManager.update(FIXED_DT);        // scripted-scene hook (no-op by default)
                physicsSolver.step(bodies, FIXED_DT); // CCD-aware advance (integrate + TOI + resolve)
                recorder.capture(bodies, FIXED_DT);   // read-only snapshot AFTER the step
                accumulator -= FIXED_DT;
            }
        } else {
            accumulator = 0.0f; // don't let time pile up while paused
            if (stepOnce) {
                sceneManager.update(FIXED_DT);
                physicsSolver.step(bodies, FIXED_DT);
                recorder.capture(bodies, FIXED_DT);
                stepOnce = false;
            }
        }

        // Statistics panel (telemetry-driven; throttled console readout).
        // FPS, timestep, active/sleeping bodies, contacts, constraints, energy.
        if (currentTime - lastDebugTime >= 0.25f) {
            const TelemetryFrame& t = physicsSolver.lastTelemetry;
            const float fps = frameTime > 1e-6f ? 1.0f / frameTime : 0.0f;
            std::cout << std::fixed << std::setprecision(2)
                      << "[Stats] FPS: " << fps
                      << "  | dt: " << (FIXED_DT * 1000.0f) << " ms"
                      << "  | bodies: " << bodies.size()
                      << " (awake " << t.awakeCount << ", asleep " << t.sleepingCount << ")"
                      << "  | contacts: " << t.contactCount
                      << "  | constraints: " << t.constraints.size()
                      << "  | KE: " << (t.kineticLinear + t.kineticRotational)
                      << " J  | PE: " << t.potential << " J\n";
            lastDebugTime = currentTime;
        }


        // Render
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        const float aspectRatio = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;

        renderer.beginFrame();

        // Draw the ground grid first (depth test handles cube-ground overlap)
        renderer.drawGround(ground, camera, aspectRatio);

        // Draw all physics bodies
        for (const auto& body : bodies) {
            if (body.shape == ShapeType::Sphere) {
                renderer.drawSphere(sphereMesh, camera, aspectRatio, body.position, body.orientation, body.radius, body.isColliding);
            } else {
                renderer.drawBody(cube, camera, aspectRatio, body.position, body.orientation, body.scale, body.isColliding);
            }
        }

        // Highlight the selected body with a wireframe bounding box (orange).
        if (selectedBody >= 0 && selectedBody < static_cast<int>(bodies.size())) {
            const RigidBody& b = bodies[selectedBody];
            // Half extents: box uses scale*0.5; sphere uses its radius.
            const glm::vec3 he = (b.shape == ShapeType::Sphere)
                ? glm::vec3(b.radius) : b.scale * 0.5f;
            // Spheres have no meaningful orientation for a box outline; use
            // identity so the highlight stays axis-aligned and readable.
            const glm::quat q = (b.shape == ShapeType::Sphere)
                ? glm::quat(1.0f, 0.0f, 0.0f, 0.0f) : b.orientation;
            const glm::vec3 col(1.0f, 0.6f, 0.1f);

            // 8 corners in local space, transformed to world.
            glm::vec3 c[8];
            int n = 0;
            for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                    for (int sz = -1; sz <= 1; sz += 2)
                        c[n++] = b.position + q * glm::vec3(sx * he.x, sy * he.y, sz * he.z);

            // 12 edges of the box (indices into the corner array above).
            static const int edges[12][2] = {
                {0,1},{0,2},{0,4},{1,3},{1,5},{2,3},
                {2,6},{3,7},{4,5},{4,6},{5,7},{6,7}
            };
            for (auto& e : edges) {
                renderer.drawLine(camera, aspectRatio, c[e[0]], c[e[1]], col);
            }
        }

        // Draw constraint lines (springs = yellow, hinges/ropes = white)
        for (const auto& sp : physicsSolver.springs) {
            glm::vec3 wA = sp.localAnchorA; // world anchor if bodyA is null
            glm::vec3 wB = sp.localAnchorB;
            if (sp.bodyA) wA = sp.bodyA->position + sp.bodyA->orientation * sp.localAnchorA;
            if (sp.bodyB) wB = sp.bodyB->position + sp.bodyB->orientation * sp.localAnchorB;
            renderer.drawLine(camera, aspectRatio, wA, wB, glm::vec3(1.0f, 0.9f, 0.1f));
        }
        for (const auto& h : physicsSolver.hinges) {
            glm::vec3 wA = h.localAnchorA;
            glm::vec3 wB = h.localAnchorB;
            if (h.bodyA) wA = h.bodyA->position + h.bodyA->orientation * h.localAnchorA;
            if (h.bodyB) wB = h.bodyB->position + h.bodyB->orientation * h.localAnchorB;
            renderer.drawLine(camera, aspectRatio, wA, wB, glm::vec3(0.9f, 0.9f, 0.9f));
        }
        for (const auto& r : physicsSolver.ropes) {
            glm::vec3 wA = r.bodyA ? r.bodyA->position + r.bodyA->orientation * r.localAnchorA : r.localAnchorA;
            glm::vec3 wB = r.bodyB ? r.bodyB->position + r.bodyB->orientation * r.localAnchorB : r.localAnchorB;
            // Green when taut, dark green when slack
            glm::vec3 col = r.taut ? glm::vec3(0.2f, 1.0f, 0.3f) : glm::vec3(0.1f, 0.5f, 0.15f);
            renderer.drawLine(camera, aspectRatio, wA, wB, col);
        }
        for (const auto& p : physicsSolver.pulleys) {
            glm::vec3 wA = p.bodyA ? p.bodyA->position + p.bodyA->orientation * p.localAnchorA : p.localAnchorA;
            glm::vec3 wB = p.bodyB ? p.bodyB->position + p.bodyB->orientation * p.localAnchorB : p.localAnchorB;

            const float wheelR = p.pulleyRadius; // render-only radius
            // The pulley sits in the XY plane; the wheel axis is +Z so the two
            // rope sides wrap around opposite edges of the wheel.

            // Tangent point where a rope from world anchor `anchor` leaves a
            // wheel of radius r centered at `center`. `side` (+1/-1) selects
            // which of the two tangents to use so each segment departs from an
            // opposite edge of the wheel. Falls back to the center if the anchor
            // is inside the wheel (degenerate).
            auto tangentPoint = [&](const glm::vec3& center, const glm::vec3& anchor, float side) -> glm::vec3 {
                const glm::vec3 toAnchor = anchor - center;
                const float d = glm::length(toAnchor);
                if (d <= wheelR + 1e-4f) return center; // degenerate: anchor inside wheel
                const glm::vec3 u = toAnchor / d;                 // center -> anchor (unit)
                float c = wheelR / d;
                if (c < -1.0f) c = -1.0f; else if (c > 1.0f) c = 1.0f;
                const float alpha = std::acos(c);
                // Rotate u by ±alpha about the wheel axis (Rodrigues; axis = +Z).
                const float s = std::sin(side * alpha);
                const float cs = std::cos(side * alpha);
                const glm::vec3 rotated(
                    u.x * cs - u.y * s,
                    u.x * s + u.y * cs,
                    u.z);
                return center + rotated * wheelR;
            };

            const glm::vec3 tangA = tangentPoint(p.pulleyPos, wA, +1.0f);
            const glm::vec3 tangB = tangentPoint(p.pulleyPos, wB, -1.0f);

            // Rope segments: body anchor -> tangent point on the wheel (cyan).
            renderer.drawLine(camera, aspectRatio, wA, tangA, glm::vec3(0.2f, 0.8f, 1.0f));
            renderer.drawLine(camera, aspectRatio, wB, tangB, glm::vec3(0.2f, 0.8f, 1.0f));

            // Draw pulley wheel as a circle of line segments (white)
            const int segs = 24;
            for (int seg = 0; seg < segs; ++seg) {
                const float a1 = 2.0f * 3.14159265f * seg / segs;
                const float a2 = 2.0f * 3.14159265f * (seg + 1) / segs;
                glm::vec3 p1 = p.pulleyPos + glm::vec3(wheelR * std::cos(a1), wheelR * std::sin(a1), 0.0f);
                glm::vec3 p2 = p.pulleyPos + glm::vec3(wheelR * std::cos(a2), wheelR * std::sin(a2), 0.0f);
                renderer.drawLine(camera, aspectRatio, p1, p2, glm::vec3(1.0f, 1.0f, 1.0f));
            }
            // Draw axle cross (scaled to the wheel radius)
            const float ax = wheelR * 0.5f;
            renderer.drawLine(camera, aspectRatio,
                p.pulleyPos + glm::vec3(-ax, 0, 0), p.pulleyPos + glm::vec3(ax, 0, 0), glm::vec3(0.7f, 0.7f, 0.7f));
            renderer.drawLine(camera, aspectRatio,
                p.pulleyPos + glm::vec3(0, -ax, 0), p.pulleyPos + glm::vec3(0, ax, 0), glm::vec3(0.7f, 0.7f, 0.7f));
        }

        // -------------------------------------------------------------------
        // Physics inspector overlays (Part 4). All read-only, driven by the
        // live bodies and the solver's telemetry snapshot. Toggle with F1-F5.
        // -------------------------------------------------------------------
        {
            const TelemetryFrame& tel = physicsSolver.lastTelemetry;

            // Contact normals (yellow): from the solved contact set.
            if (ovContactNormals) {
                for (const auto& c : tel.contacts) {
                    renderer.drawLine(camera, aspectRatio, c.point,
                                      c.point + c.normal * 0.6f, glm::vec3(1.0f, 0.95f, 0.2f));
                }
            }

            for (const auto& b : bodies) {
                // Center of mass (magenta cross).
                if (ovCenterOfMass) {
                    const float s = 0.15f;
                    renderer.drawLine(camera, aspectRatio, b.position - glm::vec3(s,0,0), b.position + glm::vec3(s,0,0), glm::vec3(1.0f, 0.2f, 1.0f));
                    renderer.drawLine(camera, aspectRatio, b.position - glm::vec3(0,s,0), b.position + glm::vec3(0,s,0), glm::vec3(1.0f, 0.2f, 1.0f));
                    renderer.drawLine(camera, aspectRatio, b.position - glm::vec3(0,0,s), b.position + glm::vec3(0,0,s), glm::vec3(1.0f, 0.2f, 1.0f));
                }
                // Angular-velocity axis (orange): direction + magnitude.
                if (ovAngularAxis) {
                    const float w = glm::length(b.angularVelocity);
                    if (w > 1e-3f) {
                        const glm::vec3 axis = b.angularVelocity / w;
                        renderer.drawLine(camera, aspectRatio, b.position,
                                          b.position + axis * std::min(w * 0.2f, 1.5f),
                                          glm::vec3(1.0f, 0.5f, 0.0f));
                    }
                }
                // Axis-aligned bounding volume (grey wireframe box).
                if (ovBoundingVolume) {
                    const glm::vec3 he = (b.shape == ShapeType::Sphere) ? glm::vec3(b.radius) : b.scale * 0.5f;
                    glm::vec3 c[8]; int n = 0;
                    for (int sx=-1; sx<=1; sx+=2) for (int sy=-1; sy<=1; sy+=2) for (int sz=-1; sz<=1; sz+=2)
                        c[n++] = b.position + glm::vec3(sx*he.x, sy*he.y, sz*he.z);
                    static const int E[12][2] = {{0,1},{0,2},{0,4},{1,3},{1,5},{2,3},{2,6},{3,7},{4,5},{4,6},{5,7},{6,7}};
                    for (auto& e : E) renderer.drawLine(camera, aspectRatio, c[e[0]], c[e[1]], glm::vec3(0.5f, 0.5f, 0.55f));
                }
                // Sleeping bodies: small blue marker above them.
                if (ovSleeping && b.asleep) {
                    renderer.drawLine(camera, aspectRatio, b.position + glm::vec3(0, 0.3f, 0),
                                      b.position + glm::vec3(0, 0.7f, 0), glm::vec3(0.3f, 0.5f, 1.0f));
                }
            }
        }

        // -------------------------------------------------------------------
        // Dual trajectory (Part 8): for the selected body when prediction is on.
        //   Ground truth  = solid green   (an isolated physics rollout)
        //   AI prediction = dotted purple (PathPredictor; rollout until a model
        //                    is loaded, in which case the two diverge)
        // Both use isolated Environments, so they never touch the live world.
        // -------------------------------------------------------------------
        if (predictionEnabled && activePredictor &&
            selectedBody >= 0 && selectedBody < static_cast<int>(bodies.size())) {
            const RigidBody& b = bodies[selectedBody];
            Observation obs;
            obs.id              = selectedBody;
            obs.mass            = b.mass;
            obs.shape           = static_cast<int>(b.shape);
            obs.position        = b.position;
            obs.velocity        = b.velocity;
            obs.angularVelocity = b.angularVelocity;
            obs.orientation     = b.orientation;
            obs.sleeping        = b.asleep;

            // Ground truth: roll the single body forward in its own Environment.
            {
                RigidBody gt = b;
                gt.asleep = false; gt.sleepTimer = 0.0f;
                Environment truthEnv;
                truthEnv.setBodies({gt});
                truthEnv.solver().sleepingEnabled = false;
                truthEnv.solver().gravityEnabled = physicsSolver.gravityEnabled;
                truthEnv.solver().aerodynamicsEnabled = physicsSolver.aerodynamicsEnabled;
                truthEnv.solver().airDensity = physicsSolver.airDensity;
                truthEnv.solver().windVelocity = physicsSolver.windVelocity;
                std::vector<glm::vec3> truth;
                truth.reserve(PREDICTION_FRAMES + 1);
                truth.push_back(b.position);
                for (int f = 0; f < PREDICTION_FRAMES; ++f) {
                    truthEnv.step(FIXED_DT);
                    truth.push_back(truthEnv.getObservation(0).position);
                }
                renderer.drawPath(camera, aspectRatio, truth, glm::vec3(0.1f, 0.9f, 0.2f)); // green
            }

            // AI prediction (purple dotted), seeded from the current position.
            const std::vector<glm::vec3> path = activePredictor->predict(obs, PREDICTION_FRAMES);
            std::vector<glm::vec3> full;
            full.reserve(path.size() + 1);
            full.push_back(b.position);
            for (const auto& p : path) full.push_back(p);
            renderer.drawDottedPath(camera, aspectRatio, full, glm::vec3(0.7f, 0.2f, 0.9f)); // purple
        }

        // Pause indicator
        if (simulationPaused) {
            renderer.drawPauseIcon();
        }

        // HUD status bars (font-free): slot 0 = recording, slot 1 = prediction,
        // slot 2 = simulation running. Green = ON/active, red = OFF/paused. The
        // full text status (scene, recording, prediction, model) is logged to
        // the console on every toggle.
        renderer.drawStatusBar(0, recordingEnabled);
        renderer.drawStatusBar(1, predictionEnabled);
        renderer.drawStatusBar(2, !simulationPaused);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}