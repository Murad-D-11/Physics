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
#include "../renderer/ground.h"
#include "../physics/rigidbody.h"
#include "../physics/physicssolver.h"

using namespace std;

Camera* activeCamera = nullptr;
bool simulationPaused = true; // starts paused; press P to begin
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

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_P && action == GLFW_PRESS) {
        simulationPaused = !simulationPaused;
    }
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
    Ground ground(20.0f);

    Camera camera(11.0f, -55.0f, 18.0f);
    activeCamera = &camera;

    PhysicsSolver physicsSolver;

    // -----------------------------------------------------------------
    // Uncomment exactly ONE line below to choose which demo runs.
    // Press P to start/pause the simulation.
    // -----------------------------------------------------------------
    // std::vector<RigidBody> bodies = spawnStableTower();
    // std::vector<RigidBody> bodies = spawnExplosion();
    // std::vector<RigidBody> bodies = spawnWreckingBall();
    // std::vector<RigidBody> bodies = spawnBilliards();
    // std::vector<RigidBody> bodies = spawnInertiaDemo();
    // std::vector<RigidBody> bodies = spawnElasticVsInelastic();
    // std::vector<RigidBody> bodies = spawnNewtonsCradle();
    std::vector<RigidBody> bodies = spawnDominoSpiral();

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

        // Physics loop (only when unpaused)
        if (!simulationPaused) {
            const float physicsStart = static_cast<float>(glfwGetTime());

            while (accumulator >= FIXED_DT) {
                physicsSolver.step(bodies, FIXED_DT); // CCD-aware advance (integrate + TOI + resolve)
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
        } else {
            accumulator = 0.0f; // don't let time pile up while paused
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
            renderer.drawBody(cube, camera, aspectRatio, body.position, body.orientation, body.scale, body.isColliding);
        }

        // Pause indicator
        if (simulationPaused) {
            renderer.drawPauseIcon();
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}