#include <iostream>
#include <algorithm>
#include <iomanip>
#include <random>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../renderer/render.h"
#include "../renderer/camera.h"
#include "../renderer/cube.h"
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
// Test scenarios
// ===========================================================================
// Each function returns a self-contained scene, and every scenario is
// centered near the world origin (x and z close to 0) so the same static
// camera frames whichever one is active -- no per-test camera tuning needed.
//
// To demonstrate one at a time: in main(), uncomment exactly ONE of the
// `bodies = spawnXxx();` lines below and leave the rest commented out.
// Uncommenting more than one at once will not compile (both would try to
// initialize the same 'bodies' variable) -- that's intentional, it forces
// exactly one active scenario at a time.
//
// Friction tests, specifically:
//   spawnTestSlidingCube()        - Test 1: cube-floor friction alone
//   spawnTestCubeOnCube()         - Test 2: cube-cube friction, both affected
//   spawnTestCubeStack()          - Test 3a: stack stability at rest
//   spawnTestPushStack()          - Test 3b: stack stability when pushed
//   spawnTestFrictionComparison() - Test 4: low vs. high friction side by side
// ===========================================================================
 
std::vector<RigidBody> spawnTestHeadOn() {
    // Test A: head-on collision, pure X-axis, equal masses.
    // Two cubes on a direct collision course, dropped from the same height
    // so gravity is identical for both -- isolates the horizontal response.
    std::vector<RigidBody> bodies;
 
    RigidBody left;
    left.position  = glm::vec3(-2.5f, 4.0f, 0.0f);
    left.velocity  = glm::vec3(3.0f, 0.0f, 0.0f);
    left.mass = 1.0f; left.inverseMass = 1.0f; left.restitution = 0.5f;
    bodies.push_back(left);
 
    RigidBody right;
    right.position = glm::vec3(2.5f, 4.0f, 0.0f);
    right.velocity = glm::vec3(-3.0f, 0.0f, 0.0f);
    right.mass = 1.0f; right.inverseMass = 1.0f; right.restitution = 0.5f;
    bodies.push_back(right);
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestVertical() {
    // Test B: vertical collision, body-body (not the floor).
    // One cube rests near the floor; a second drops straight down onto it.
    std::vector<RigidBody> bodies;
 
    RigidBody bottom;
    bottom.position = glm::vec3(0.0f, 0.5f, 0.0f);
    bottom.velocity = glm::vec3(0.0f);
    bottom.mass = 1.0f; bottom.inverseMass = 1.0f; bottom.restitution = 0.3f;
    bodies.push_back(bottom);
 
    RigidBody dropper;
    dropper.position = glm::vec3(0.0f, 6.0f, 0.0f);
    dropper.velocity = glm::vec3(0.0f, -4.0f, 0.0f);
    dropper.mass = 1.0f; dropper.inverseMass = 1.0f; dropper.restitution = 0.3f;
    bodies.push_back(dropper);
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestDiagonal() {
    // Test C: diagonal collision, X, Y, and Z velocity components at once.
    // Note the AABB collision normal is still always axis-aligned (SAT picks whichever single axis has the smallest overlap), that's correct
    // behaviour for box-vs-box collision, but the approach here is diagonal, which exercises the general impulse formula well.
    std::vector<RigidBody> bodies;
 
    RigidBody diagA;
    diagA.position = glm::vec3(-2.0f, 5.0f, -2.0f);
    diagA.velocity = glm::vec3(2.0f, -1.0f, 2.0f);
    diagA.mass = 1.0f; diagA.inverseMass = 1.0f; diagA.restitution = 0.5f;
    bodies.push_back(diagA);
 
    RigidBody diagB;
    diagB.position = glm::vec3(2.0f, 4.0f, 2.0f);
    diagB.velocity = glm::vec3(-2.0f, 0.5f, -2.0f);
    diagB.mass = 1.0f; diagB.inverseMass = 1.0f; diagB.restitution = 0.5f;
    bodies.push_back(diagB);
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestDifferentMasses() {
    // Test D: momentum transfer between different masses.
    // A heavy cube (mass 8) barrels into a light, stationary cube (mass 0.5).
    // Because the impulse each body receives is scaled by its own
    // inverseMass, the light cube should fly off much faster than the
    // heavy cube slows down.
    std::vector<RigidBody> bodies;
 
    RigidBody heavy;
    heavy.position = glm::vec3(-2.5f, 6.0f, 0.0f);
    heavy.velocity = glm::vec3(3.0f, 0.0f, 0.0f);
    heavy.mass = 8.0f; heavy.inverseMass = 1.0f / 8.0f; heavy.restitution = 0.4f;
    bodies.push_back(heavy);
 
    RigidBody light;
    light.position = glm::vec3(1.5f, 6.0f, 0.0f);
    light.velocity = glm::vec3(0.0f);
    light.mass = 0.5f; light.inverseMass = 1.0f / 0.5f; light.restitution = 0.4f;
    bodies.push_back(light);
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestSlidingCube() {
    // Test 1: Sliding cube.
    // A single cube is given horizontal velocity right at floor height (no fall, no bounce -- isolates friction's deceleration effect). 
    // Expected: the cube slides, gradually slows, and eventually comes to a full stop.
    std::vector<RigidBody> bodies;
 
    RigidBody cube;
    cube.position = glm::vec3(-3.5f, 0.5f, 0.0f);
    cube.velocity = glm::vec3(4.0f, 0.0f, 0.0f);
    cube.mass = 1.0f;
    cube.inverseMass = 1.0f;
    cube.restitution = 0.1f;
    // Low restitution so it doesn't hop on the small initial floor contact.
    cube.friction = 0.4f;
    // Moderate friction --> should visibly decelerate over roughly a second.
    bodies.push_back(cube);
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestCubeOnCube() {
    // Test 2: Cube on cube.
    // Cube A slides along the floor and hits stationary Cube B. Expected:
    // friction affects their relative motion both while sliding beforehand (floor friction slows A down before impact) and at the contact
    // between A and B (the two share momentum, then both decelerate from floor friction afterward). Both cubes are visibly affected.
    std::vector<RigidBody> bodies;
 
    RigidBody a;
    a.position = glm::vec3(-3.5f, 0.5f, 0.0f);
    a.velocity = glm::vec3(10.0f, 0.0f, 0.0f);
    a.mass = 1.0f;
    a.inverseMass = 1.0f;
    a.restitution = 0.2f;
    a.friction = 0.4f;
    bodies.push_back(a);
 
    RigidBody b;
    b.position = glm::vec3(0.0f, 0.5f, 0.0f);
    b.velocity = glm::vec3(0.0f);
    b.mass = 1.0f;
    b.inverseMass = 1.0f;
    b.restitution = 0.2f;
    b.friction = 0.4f;
    bodies.push_back(b);
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestCubeStack() {
    // Test 3 (part 1): Cube stack stability.
    // Four cubes stacked exactly on top of each other and dropped a short distance onto the floor. Expected: the stack remains standing with
    // minimal jitter once SOLVER_ITERATIONS propagates the contact forces all the way down to the floor.
    std::vector<RigidBody> bodies;
 
    for (int i = 0; i < 4; ++i) {
        RigidBody cube;
        cube.position = glm::vec3(0.0f, 0.55f + static_cast<float>(i) * 1.02f, 0.0f);
        // Tiny 0.02 gap between cubes so they start just barely separated (avoids beginning the simulation already overlapping, which would
        // otherwise trigger one large correction on the very first step).
        cube.velocity = glm::vec3(0.0f);
        cube.mass = 1.0f;
        cube.inverseMass = 1.0f;
        cube.restitution = 0.1f;
        // Low restitution --> stacked cubes shouldn't bounce off each other.
        cube.friction = 0.6f;
        // Higher friction helps the stack resist toppling sideways.
        bodies.push_back(cube);
    }
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestPushStack() {
    // Test 3 (part 2): Push the bottom cube of a stack.
    // Same 4-cube stack as spawnTestCubeStack(), but the bottom cube starts with horizontal velocity, as if it had just been shoved. Expected:
    // friction between the cubes should make the stack resist sliding coherently -- the bottom cube shouldn't shoot out from under the
    // others; the whole stack should drag together and settle, rather than the top cubes staying behind while the bottom one escapes.
    std::vector<RigidBody> bodies;
 
    for (int i = 0; i < 4; ++i) {
        RigidBody cube;
        cube.position = glm::vec3(0.0f, 0.55f + static_cast<float>(i) * 1.02f, 0.0f);
        cube.velocity = (i == 0) ? glm::vec3(3.0f, 0.0f, 0.0f) : glm::vec3(0.0f);
        // Only the bottom cube (i == 0) gets the initial push.
        cube.mass = 1.0f;
        cube.inverseMass = 1.0f;
        cube.restitution = 0.1f;
        cube.friction = 0.6f;
        bodies.push_back(cube);
    }
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestFallingCubes() {
    // Test 3: Falling cubes.
    // Several cubes spawn above the floor in a loose scattered cluster (not perfectly aligned, so they don't all land in a single neat column) and
    // fall together. Expected: they collide with each other and the floor, then settle into a stable pile without exploding apart.
    std::vector<RigidBody> bodies;
 
    const glm::vec3 offsets[] = {
        { -1.0f, 3.0f,  0.6f}, { 0.8f, 3.6f, -0.5f}, {-0.4f, 4.4f,  0.9f},
        {  1.1f, 5.0f,  0.2f}, {-1.2f, 5.6f, -0.8f}, { 0.3f, 6.4f,  0.5f},
    };
 
    for (const auto& offset : offsets) {
        RigidBody cube;
        cube.position = offset;
        cube.velocity = glm::vec3(0.0f);
        cube.mass = 1.0f;
        cube.inverseMass = 1.0f;
        cube.restitution = 0.2f;
        cube.friction = 0.5f;
        bodies.push_back(cube);
    }
 
    return bodies;
}
 
std::vector<RigidBody> spawnTestFrictionComparison() {
    // Test 4: Different friction values, side by side.
    // Two identical cubes given the same starting horizontal velocity, but one has very low friction and the other very high friction. They're
    // offset in Z so they don't collide with each other; both are visible in the same shot. 
    // Expected: the low-friction cube slides much farther than the high-friction cube, which stops almost immediately.
    std::vector<RigidBody> bodies;
 
    RigidBody lowFriction;
    lowFriction.position = glm::vec3(-3.0f, 0.5f, -1.5f);
    lowFriction.velocity = glm::vec3(4.0f, 0.0f, 0.0f);
    lowFriction.mass = 1.0f;
    lowFriction.inverseMass = 1.0f;
    lowFriction.restitution = 0.1f;
    lowFriction.friction = 0.03f;
    // Nearly frictionless --> should slide most of the way across the scene.
    bodies.push_back(lowFriction);
 
    RigidBody highFriction;
    highFriction.position = glm::vec3(-3.0f, 0.5f, 1.5f);
    highFriction.velocity = glm::vec3(4.0f, 0.0f, 0.0f);
    highFriction.mass = 1.0f;
    highFriction.inverseMass = 1.0f;
    highFriction.restitution = 0.1f;
    highFriction.friction = 1.2f;
    // High friction --> should stop within a fraction of a second.
    bodies.push_back(highFriction);
 
    return bodies;
}


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
    GLFWwindow* window = glfwCreateWindow(ScreenHeight / 2, ScreenHeight / 2, "Hello Physics Engine!", NULL, NULL);

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
    // One shared cube mesh. Every body uses this same VAO, just drawn at a
    // different world position via a different model matrix each draw call.

    Camera camera(11.0f, -55.0f, 18.0f);
    // A single static framing that works for every test above: all
    // scenarios keep their action within roughly x∈[-4,4], z∈[-2,2],
    // y∈[0,9], centered near the origin the camera already orbits.
    activeCamera = &camera;
 
    PhysicsSolver physicsSolver;
 
    // -----------------------------------------------------------------
    // Uncomment exactly ONE line below to choose which test scenario runs.
    // -----------------------------------------------------------------
    // std::vector<RigidBody> bodies = spawnTestHeadOn();
    // std::vector<RigidBody> bodies = spawnTestVertical();
    // std::vector<RigidBody> bodies = spawnTestDiagonal();
    // std::vector<RigidBody> bodies = spawnTestDifferentMasses();
    // std::vector<RigidBody> bodies = spawnTestSlidingCube();
    // std::vector<RigidBody> bodies = spawnTestCubeOnCube();
    // std::vector<RigidBody> bodies = spawnTestCubeStack();
    // std::vector<RigidBody> bodies = spawnTestPushStack();
    // std::vector<RigidBody> bodies = spawnTestFallingCubes();
    std::vector<RigidBody> bodies = spawnTestFrictionComparison();
 
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
                // Pass 1: gravity + Euler integration, per body.
            }

            for (auto& body : bodies) {
                physicsSolver.floorCollision(body);
                // Pass 2: floor response per body (uses per-body restitution).
            }

            physicsSolver.detectAndResolve(bodies);
            // Pass 3: pairwise detection + penetration correction + impulse iterations (all inside detectAndResolve).
            accumulator -= FIXED_DT;
        }
 
        const float physicsTimeMs = (static_cast<float>(glfwGetTime()) - physicsStart) * 1000.0f;
 
        // Debug statistics (printed once per 1/10th of a second)
        if (currentTime - lastDebugTime >= 0.1f) {
            const int pairCount = static_cast<int>(bodies.size() * (bodies.size() - 1) / 2);
            std::cout << "[Physics] " << "Bodies: "        << bodies.size() << "  |  Pairs: "    << pairCount << "  |  Contacts: " << physicsSolver.lastContactCount << "  |  Time: "     << std::fixed << std::setprecision(2) << physicsTimeMs     << " ms\n";
            lastDebugTime = currentTime;
        }

        // Render all bodies into the same frame

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        const float aspectRatio = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;

        // Clears the color and depth buffers ONCE for this frame.
        // Must happen before any drawBody() calls; the clear resets the
        // depth buffer so all 10 cubes depth-test against each other correctly.
        renderer.beginFrame();

        /** 
         * Draws this body at its current physics position with its collisiondebug color. 
         * Each call sets its own model matrix uniform, so the same cube mesh gets "stamped" at each body's position in turn.
         * The depth buffer accumulates all cubes: a face of body[7] can correctly occlude a face of body[3] that's behind it.
         */
        for (const auto& body : bodies) {
            renderer.drawBody(cube, camera, aspectRatio, body.position, body.isColliding);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}