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

std::vector<RigidBody> spawnConstraintDemo(PhysicsSolver& solver) {
    std::vector<RigidBody> bodies;
    bodies.reserve(16); // prevent reallocation during setup

    // =====================================================================
    // (A) PENDULUM: sphere on a hinge anchored to the ceiling
    // =====================================================================
    RigidBody pendulumBob;
    pendulumBob.position = glm::vec3(-4.0f, 3.0f, 0.0f);
    pendulumBob.velocity = glm::vec3(0.0f);
    setSphereMassProperties(pendulumBob, 2.0f, 0.4f);
    pendulumBob.restitution = 0.3f;
    pendulumBob.friction = 0.5f;
    bodies.push_back(pendulumBob); // index 0

    // =====================================================================
    // (B) SPRING-MASS OSCILLATOR: cube on a vertical spring
    // =====================================================================
    RigidBody springMass;
    springMass.scale = glm::vec3(0.6f);
    springMass.position = glm::vec3(0.0f, 2.0f, 0.0f);
    springMass.velocity = glm::vec3(0.0f);
    setCubeMassProperties(springMass, 1.0f);
    springMass.restitution = 0.2f;
    springMass.friction = 0.5f;
    bodies.push_back(springMass); // index 1

    // =====================================================================
    // (C) HINGED DOOR: cube hinged at its left edge
    // =====================================================================
    RigidBody door;
    door.scale = glm::vec3(0.1f, 1.5f, 1.0f);
    door.position = glm::vec3(3.5f, 2.0f, 0.0f);
    door.velocity = glm::vec3(0.0f);
    door.angularVelocity = glm::vec3(0.0f, 1.0f, 0.0f);
    setCubeMassProperties(door, 3.0f);
    door.restitution = 0.1f;
    door.friction = 0.5f;
    bodies.push_back(door); // index 2

    // =====================================================================
    // (D) SPRING BETWEEN TWO CUBES
    // =====================================================================
    RigidBody cubeA;
    cubeA.scale = glm::vec3(0.5f);
    cubeA.position = glm::vec3(-2.0f, 0.5f, 3.0f);
    cubeA.velocity = glm::vec3(0.0f);
    setCubeMassProperties(cubeA, 1.0f);
    cubeA.restitution = 0.2f;
    cubeA.friction = 0.4f;
    bodies.push_back(cubeA); // index 3

    RigidBody cubeB;
    cubeB.scale = glm::vec3(0.5f);
    cubeB.position = glm::vec3(0.0f, 0.5f, 3.0f);
    cubeB.velocity = glm::vec3(2.0f, 0.0f, 0.0f);
    setCubeMassProperties(cubeB, 1.0f);
    cubeB.restitution = 0.2f;
    cubeB.friction = 0.4f;
    bodies.push_back(cubeB); // index 4

    // =====================================================================
    // (E) DOUBLE PENDULUM
    // =====================================================================
    RigidBody bob1;
    bob1.position = glm::vec3(5.0f, 3.5f, 0.0f);
    bob1.velocity = glm::vec3(0.0f);
    setSphereMassProperties(bob1, 1.0f, 0.3f);
    bob1.restitution = 0.2f;
    bob1.friction = 0.3f;
    bodies.push_back(bob1); // index 5

    RigidBody bob2;
    bob2.position = glm::vec3(6.0f, 2.5f, 0.0f);
    bob2.velocity = glm::vec3(0.0f);
    setSphereMassProperties(bob2, 1.0f, 0.3f);
    bob2.restitution = 0.2f;
    bob2.friction = 0.3f;
    bodies.push_back(bob2); // index 6

    // NOTE: Constraints store pointers into the bodies vector. Since we
    // reserved enough capacity above, addresses are stable. The solver also
    // receives bodies by reference and uses pointers from this same vector.
    // Constraints are set up in main() AFTER bodies is fully built — see below.

    return bodies;
}

// Called after spawnConstraintDemo returns, with stable body addresses.
void setupConstraints(PhysicsSolver& solver, std::vector<RigidBody>& bodies) {
    // (A) Pendulum hinge: pivot at (-3, 5, 0) in world.
    // Body 0 (sphere) starts at (-4, 3, 0). For the hinge to create a pendulum,
    // localAnchorB must be the point on the body that connects to the pivot.
    // The rod from pivot to body center has vector: body.pos - pivot = (-4,3,0)-(-3,5,0) = (-1,-2,0).
    // In body-local space (identity orientation), localAnchorB should be the
    // NEGATIVE of this (from body center TO the pivot): (1, 2, 0).
    {
        HingeConstraint h;
        h.bodyA = nullptr;
        h.bodyB = &bodies[0];
        h.localAnchorA = glm::vec3(-3.0f, 5.0f, 0.0f); // world pivot
        h.localAnchorB = glm::vec3(1.0f, 2.0f, 0.0f);  // from body center to pivot (local)
        h.localAxisA = glm::vec3(0.0f, 0.0f, 1.0f);
        h.localAxisB = glm::vec3(0.0f, 0.0f, 1.0f);
        solver.hinges.push_back(h);
    }

    // (B) Vertical spring (body 1 -> world ceiling)
    {
        SpringConstraint sp;
        sp.bodyA = nullptr;
        sp.bodyB = &bodies[1];
        sp.localAnchorA = glm::vec3(0.0f, 5.0f, 0.0f);
        sp.localAnchorB = glm::vec3(0.0f, 0.3f, 0.0f);
        sp.restLength = 1.5f;
        sp.stiffness = 40.0f;
        sp.damping = 1.0f;
        solver.springs.push_back(sp);
    }

    // (C) Door hinge: pivot at left edge of door, vertical axis.
    // Door (body 2) is at (3.5, 2, 0), left edge at local (-0.05, 0, -0.5).
    // The world pivot is at (3.0, 2, -0.5). From door center to pivot:
    // (3.0-3.5, 2-2, -0.5-0) = (-0.5, 0, -0.5) in world = local (since orient=identity).
    {
        HingeConstraint h;
        h.bodyA = nullptr;
        h.bodyB = &bodies[2];
        h.localAnchorA = glm::vec3(3.0f, 2.0f, -0.5f);
        h.localAnchorB = glm::vec3(-0.5f, 0.0f, -0.5f); // from center to pivot
        h.localAxisA = glm::vec3(0.0f, 1.0f, 0.0f);
        h.localAxisB = glm::vec3(0.0f, 1.0f, 0.0f);
        solver.hinges.push_back(h);
    }

    // (D) Spring between two cubes (body 3 <-> body 4)
    {
        SpringConstraint sp;
        sp.bodyA = &bodies[3];
        sp.bodyB = &bodies[4];
        sp.localAnchorA = glm::vec3(0.25f, 0.0f, 0.0f);
        sp.localAnchorB = glm::vec3(-0.25f, 0.0f, 0.0f);
        sp.restLength = 1.0f;
        sp.stiffness = 30.0f;
        sp.damping = 2.0f;
        solver.springs.push_back(sp);
    }

    // (E) Double pendulum: body 5 hinged to world, body 6 hinged to body 5.
    // Body 5 at (5, 3.5, 0), pivot at (5, 5, 0) → localAnchorB = (0, 1.5, 0)
    // Body 6 at (6, 2.5, 0), attached to body 5 center → localAnchorA = (0,0,0) on body5,
    //   and localAnchorB = (offset from body6 center to where body5 center is).
    //   body5.pos - body6.pos = (5-6, 3.5-2.5, 0) = (-1, 1, 0) → localAnchorB = (-1,1,0)
    {
        HingeConstraint h1;
        h1.bodyA = nullptr;
        h1.bodyB = &bodies[5];
        h1.localAnchorA = glm::vec3(5.0f, 5.0f, 0.0f);
        h1.localAnchorB = glm::vec3(0.0f, 1.5f, 0.0f);
        h1.localAxisA = glm::vec3(0.0f, 0.0f, 1.0f);
        h1.localAxisB = glm::vec3(0.0f, 0.0f, 1.0f);
        solver.hinges.push_back(h1);

        HingeConstraint h2;
        h2.bodyA = &bodies[5];
        h2.bodyB = &bodies[6];
        h2.localAnchorA = glm::vec3(0.0f, 0.0f, 0.0f); // bottom of bob1
        h2.localAnchorB = glm::vec3(-1.0f, 1.0f, 0.0f); // from bob2 center to bob1 center
        h2.localAxisA = glm::vec3(0.0f, 0.0f, 1.0f);
        h2.localAxisB = glm::vec3(0.0f, 0.0f, 1.0f);
        solver.hinges.push_back(h2);
    }
}

// ===========================================================================
// Rope & Pulley Demo Scene
// ===========================================================================

std::vector<RigidBody> spawnRopePulleyDemo(PhysicsSolver& solver) {
    std::vector<RigidBody> bodies;
    bodies.reserve(16);

    // =====================================================================
    // ATWOOD MACHINE (Day 30)
    //
    // Two masses connected by an inextensible rope over a fixed pulley.
    // Constraint: dist(pulley,A) + dist(pulley,B) = L (constant)
    //
    //        [PULLEY] at (0, 8, 0)
    //         |     |
    //     [2kg]   [1kg]
    //   x=-0.8    x=0.8   (both start at y=5, 3m below pulley)
    //
    // Expected: heavy (2kg) descends, light (1kg) ascends
    // Acceleration: a = (2-1)/(2+1) * g = g/3 ≈ 3.27 m/s²
    // =====================================================================

    const glm::vec3 pulleyPos(0.0f, 8.0f, 0.0f);

    // Heavy mass A (left, 2kg)
    RigidBody heavy;
    heavy.position = glm::vec3(-0.8f, 5.0f, 0.0f);
    heavy.velocity = glm::vec3(0.0f);
    setSphereMassProperties(heavy, 2.0f, 0.35f);
    heavy.restitution = 0.1f;
    heavy.friction = 0.3f;
    bodies.push_back(heavy); // index 0

    // Light mass B (right, 1kg)
    RigidBody light;
    light.position = glm::vec3(0.8f, 5.0f, 0.0f);
    light.velocity = glm::vec3(0.0f);
    setSphereMassProperties(light, 1.0f, 0.3f);
    light.restitution = 0.1f;
    light.friction = 0.3f;
    bodies.push_back(light); // index 1

    // Pulley constraint: total rope length = dist(pulley,A) + dist(pulley,B)
    // dist(pulley, A) = sqrt(0.8² + 3²) = sqrt(9.64) ≈ 3.105
    // dist(pulley, B) = sqrt(0.8² + 3²) = sqrt(9.64) ≈ 3.105
    // Total L ≈ 6.21
    PulleyConstraint atwood;
    atwood.bodyA = &bodies[0];
    atwood.bodyB = &bodies[1];
    atwood.localAnchorA = glm::vec3(0.0f);
    atwood.localAnchorB = glm::vec3(0.0f);
    atwood.pulleyPos = pulleyPos;
    atwood.totalRopeLength = std::sqrt(0.8f * 0.8f + 3.0f * 3.0f) * 2.0f;
    atwood.pulleyRadius = 0.25f; // render-only wheel radius
    solver.pulleys.push_back(atwood);

    // =====================================================================
    // ROPE PENDULUM: A sphere swinging on a stiff "rope" from a fixed point.
    // =====================================================================

    RigidBody pendulumBob;
    pendulumBob.position = glm::vec3(-5.0f, 5.0f, 0.0f); // displaced to swing
    pendulumBob.velocity = glm::vec3(0.0f);
    setSphereMassProperties(pendulumBob, 1.5f, 0.4f);
    pendulumBob.restitution = 0.3f;
    pendulumBob.friction = 0.5f;
    bodies.push_back(pendulumBob); // index 2

    // Rope as a stiff spring (pendulum)
    SpringConstraint ropePend;
    ropePend.bodyA = nullptr;
    ropePend.bodyB = &bodies[2];
    ropePend.localAnchorA = glm::vec3(-4.0f, 8.0f, 0.0f); // ceiling anchor
    ropePend.localAnchorB = glm::vec3(0.0f);
    ropePend.restLength = 3.0f;
    ropePend.stiffness = 400.0f;  // stiff (rope-like)
    ropePend.damping = 3.0f;      // small damping
    solver.springs.push_back(ropePend);

    // =====================================================================
    // CHAIN: Four small spheres connected by stiff springs (like a rope chain)
    // =====================================================================

    const glm::vec3 chainTop(5.0f, 8.0f, 0.0f);
    for (int i = 0; i < 4; ++i) {
        RigidBody link;
        link.position = glm::vec3(5.0f, 7.0f - i * 0.8f, 0.0f);
        link.velocity = glm::vec3(0.3f, 0.0f, 0.0f); // slight sideways motion
        setSphereMassProperties(link, 0.4f, 0.15f);
        link.restitution = 0.1f;
        link.friction = 0.4f;
        bodies.push_back(link); // indices 3, 4, 5, 6
    }

    // Chain segments as stiff springs
    // Ceiling -> first link
    SpringConstraint chain0;
    chain0.bodyA = nullptr;
    chain0.bodyB = &bodies[3];
    chain0.localAnchorA = chainTop;
    chain0.localAnchorB = glm::vec3(0.0f);
    chain0.restLength = 0.8f;
    chain0.stiffness = 300.0f;
    chain0.damping = 5.0f;
    solver.springs.push_back(chain0);

    // Link-to-link
    for (int i = 0; i < 3; ++i) {
        SpringConstraint seg;
        seg.bodyA = &bodies[3 + i];
        seg.bodyB = &bodies[4 + i];
        seg.localAnchorA = glm::vec3(0.0f);
        seg.localAnchorB = glm::vec3(0.0f);
        seg.restLength = 0.8f;
        seg.stiffness = 300.0f;
        seg.damping = 5.0f;
        solver.springs.push_back(seg);
    }

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

    // -----------------------------------------------------------------
    // Uncomment exactly ONE line below to choose which demo runs.
    // Press P to start/pause the simulation.
    // -----------------------------------------------------------------
    // std::vector<RigidBody> bodies = spawnStableTower();
    // std::vector<RigidBody> bodies = spawnSphereDemo();
    std::vector<RigidBody> bodies = spawnRopePulleyDemo(physicsSolver);
    // std::vector<RigidBody> bodies = spawnConstraintDemo(physicsSolver);
    // setupConstraints(physicsSolver, bodies);
    // std::vector<RigidBody> bodies = spawnSlopeDemo(physicsSolver);
    // std::vector<RigidBody> bodies = spawnExplosion();
    // std::vector<RigidBody> bodies = spawnBilliards();
    // std::vector<RigidBody> bodies = spawnInertiaDemo();
    // std::vector<RigidBody> bodies = spawnElasticVsInelastic();
    // std::vector<RigidBody> bodies = spawnNewtonsCradle();
    // std::vector<RigidBody> bodies = spawnDominoSpiral();

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
            if (body.shape == ShapeType::Sphere) {
                renderer.drawSphere(sphereMesh, camera, aspectRatio, body.position, body.orientation, body.radius, body.isColliding);
            } else {
                renderer.drawBody(cube, camera, aspectRatio, body.position, body.orientation, body.scale, body.isColliding);
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