#include <iostream>
#include <algorithm>
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
#include "../renderer/ground.h"
#include "../physics/rigidbody.h"
#include "../physics/physicssolver.h"

using namespace std;

Camera* activeCamera = nullptr;
bool isDragging = false;
double lastMouseX = 0.0;
double lastMouseY = 0.0;

void framebufferSizeChanged(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;
    if (action == GLFW_PRESS) {
        isDragging = true;
        glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
    } else if (action == GLFW_RELEASE) {
        isDragging = false;
    }
}

void cursorPosCallback(GLFWwindow* window, double xPos, double yPos) {
    if (!isDragging || activeCamera == nullptr) return;
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

// ===========================================================================
// Mass property helper
// ===========================================================================
void setCubeMassProperties(RigidBody& body, float mass) {
    body.mass = mass;
    body.inverseMass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    body.updateInertiaTensor();
}

// ===========================================================================
// Physics Demo Scenarios
// ===========================================================================
// Uncomment exactly ONE line in main() to choose which demo runs.
// Each function returns a self-contained scene centered near the origin.
// ===========================================================================

std::vector<RigidBody> spawnStableTower() {
    // 6 cubes stacked precisely. Tests resting contact stability,
    // accumulated impulse convergence, and warm starting.
    // Expected: tower settles and remains perfectly stationary indefinitely.
    std::vector<RigidBody> bodies;

    for (int i = 0; i < 6; ++i) {
        RigidBody cube;
        cube.position = glm::vec3(0.0f, 0.5f + static_cast<float>(i) * 1.01f, 0.0f);
        cube.velocity = glm::vec3(0.0f);
        cube.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.1f;
        cube.friction = 0.6f;
        bodies.push_back(cube);
    }

    return bodies;
}

std::vector<RigidBody> spawnDominoChain() {
    // 8 tall thin dominoes spaced in a line. The first is given an angular
    // push to topple it. Tests OBB edge-face contacts, angular momentum
    // transfer, and friction-driven chain reaction.
    // Expected: first domino falls into second, chain reaction topples all.
    std::vector<RigidBody> bodies;

    const float spacing = 0.8f;
    const glm::vec3 dominoScale(0.2f, 1.0f, 0.5f);

    for (int i = 0; i < 8; ++i) {
        RigidBody domino;
        domino.scale = dominoScale;
        domino.position = glm::vec3(static_cast<float>(i) * spacing - 2.8f, 0.5f, 0.0f);
        domino.velocity = glm::vec3(0.0f);
        domino.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        setCubeMassProperties(domino, 1.0f);
        domino.restitution = 0.1f;
        domino.friction = 0.5f;
        bodies.push_back(domino);
    }

    // Give the first domino a small angular push to start the chain
    bodies[0].angularVelocity = glm::vec3(0.0f, 0.0f, -2.0f);

    return bodies;
}

std::vector<RigidBody> spawnRotatedDrop() {
    // A cube rotated 45 degrees about Z dropped onto a flat cube below.
    // Tests OBB edge-to-face contact generation and angular settling.
    // Expected: rotated cube lands on its edge, tips to one side, settles flat.
    std::vector<RigidBody> bodies;

    // Base cube, resting on floor
    RigidBody base;
    base.position = glm::vec3(0.0f, 0.5f, 0.0f);
    base.velocity = glm::vec3(0.0f);
    setCubeMassProperties(base, 2.0f);
    base.restitution = 0.2f;
    base.friction = 0.6f;
    bodies.push_back(base);

    // Rotated cube falling onto it
    RigidBody rotated;
    rotated.position = glm::vec3(0.0f, 4.0f, 0.0f);
    rotated.velocity = glm::vec3(0.0f);
    rotated.orientation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    setCubeMassProperties(rotated, 1.0f);
    rotated.restitution = 0.2f;
    rotated.friction = 0.5f;
    bodies.push_back(rotated);

    return bodies;
}

std::vector<RigidBody> spawnNewtonsCradle() {
    // 5 cubes in a row, touching. One cube launched into the end.
    // Tests impulse propagation through a chain of resting contacts.
    // Expected: the far cube pops off while the rest stay mostly still.
    std::vector<RigidBody> bodies;

    for (int i = 0; i < 5; ++i) {
        RigidBody cube;
        cube.position = glm::vec3(static_cast<float>(i) * 1.0f - 2.0f, 0.5f, 0.0f);
        cube.velocity = glm::vec3(0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.95f;
        cube.friction = 0.2f;
        bodies.push_back(cube);
    }

    // Launcher cube
    RigidBody launcher;
    launcher.position = glm::vec3(-5.0f, 0.5f, 0.0f);
    launcher.velocity = glm::vec3(5.0f, 0.0f, 0.0f);
    setCubeMassProperties(launcher, 1.0f);
    launcher.restitution = 0.95f;
    launcher.friction = 0.2f;
    bodies.push_back(launcher);

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
    // A triangle formation of 6 cubes on the floor, struck by one launched cube.
    // Tests off-center OBB impacts, angular + linear momentum scattering.
    // Expected: formation breaks apart with realistic deflections and spins.
    std::vector<RigidBody> bodies;

    const float gap = 1.05f;
    const glm::vec3 triBase(1.5f, 0.5f, 0.0f);

    // Row 1 (3 cubes)
    for (int i = 0; i < 3; ++i) {
        RigidBody cube;
        cube.position = triBase + glm::vec3(0.0f, 0.0f, (static_cast<float>(i) - 1.0f) * gap);
        cube.velocity = glm::vec3(0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.4f;
        cube.friction = 0.3f;
        bodies.push_back(cube);
    }

    // Row 2 (2 cubes)
    for (int i = 0; i < 2; ++i) {
        RigidBody cube;
        cube.position = triBase + glm::vec3(gap, 0.0f, (static_cast<float>(i) - 0.5f) * gap);
        cube.velocity = glm::vec3(0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.4f;
        cube.friction = 0.3f;
        bodies.push_back(cube);
    }

    // Row 3 (1 cube, the apex)
    {
        RigidBody cube;
        cube.position = triBase + glm::vec3(2.0f * gap, 0.0f, 0.0f);
        cube.velocity = glm::vec3(0.0f);
        setCubeMassProperties(cube, 1.0f);
        cube.restitution = 0.4f;
        cube.friction = 0.3f;
        bodies.push_back(cube);
    }

    // Cue cube — launched from the left with slight z-offset for asymmetric scatter
    RigidBody cue;
    cue.position = glm::vec3(-4.0f, 0.5f, 0.1f);
    cue.velocity = glm::vec3(6.0f, 0.0f, 0.0f);
    setCubeMassProperties(cue, 1.0f);
    cue.restitution = 0.4f;
    cue.friction = 0.3f;
    bodies.push_back(cue);

    return bodies;
}

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

    Render renderer;
    Cube cube(1.0f);
    Ground ground(20.0f);

    Camera camera(11.0f, -55.0f, 18.0f);
    activeCamera = &camera;

    PhysicsSolver physicsSolver;

    // -----------------------------------------------------------------
    // Uncomment exactly ONE line below to choose which demo runs.
    // -----------------------------------------------------------------
    std::vector<RigidBody> bodies = spawnStableTower();
    // std::vector<RigidBody> bodies = spawnDominoChain();
    // std::vector<RigidBody> bodies = spawnRotatedDrop();
    // std::vector<RigidBody> bodies = spawnNewtonsCradle();
    // std::vector<RigidBody> bodies = spawnAvalanche();
    // std::vector<RigidBody> bodies = spawnBilliards();

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

        // Physics loop
        const float physicsStart = static_cast<float>(glfwGetTime());

        while (accumulator >= FIXED_DT) {
            for (auto& body : bodies) {
                physicsSolver.integrate(body, FIXED_DT);
            }
            physicsSolver.detectAndResolve(bodies);
            accumulator -= FIXED_DT;
        }

        const float physicsTimeMs = (static_cast<float>(glfwGetTime()) - physicsStart) * 1000.0f;

        // Debug statistics
        if (currentTime - lastDebugTime >= 0.1f) {
            const int pairCount = static_cast<int>(bodies.size() * (bodies.size() - 1) / 2);
            std::cout << "[Physics] Bodies: " << bodies.size()
                      << "  |  Pairs: " << pairCount
                      << "  |  Contacts: " << physicsSolver.lastContactCount
                      << "  |  Time: " << std::fixed << std::setprecision(2) << physicsTimeMs << " ms\n";
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
            renderer.drawBody(cube, camera, aspectRatio, body.position, body.orientation, body.isColliding);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
