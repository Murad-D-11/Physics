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

    Camera camera(14.0f, -45.0f, 25.0f);
    // Pulled back to 14 units and angled so all 10 falling cubes are visible.
    activeCamera = &camera;

    PhysicsSolver physicsSolver;

    /**
     * Spawn 20 bodies: 4 columns x 5 rows (stress test)
     * 
     * Each column is a different material to demonstrate different restitution properties
     * To stress-test with more bodies, increase BODY_ROWS
     * 
     * Col 0 - heavy steel: mass=5, resititution=0.10 (barely bounces)
     * Col 1 - standard: mass=1, restitution=0.40 (normal)
     * Col 2 - light rubber: mass=0.3, restitution=0.85 (very bouncy)
     * Col 3 - heavy rubber: mass=4, restitution=0.60 (bouncy)
     */

    // const int BODY_COLS = 4;
    // const int BODY_ROWS = 5;

    // struct MaterialDef { float mass, inverseMass, restitution; };
    // const MaterialDef materials[BODY_COLS] = {
    //     { 5.0f, 1.0f / 5.0f, 0.10f },
    //     { 1.0f, 1.0f, 0.40f },
    //     { 0.3f, 1.0f / 0.3f, 0.85f },
    //     { 4.0f, 1.0f / 4.0f, 0.60f },
    // };

    // std::vector<RigidBody> bodies(BODY_COLS * BODY_ROWS);
    // for (int row = 0; row < BODY_ROWS; ++row) {
    //     for (int col = 0; col < BODY_COLS; ++col) {
    //         const int idx = row * BODY_COLS + col;
    //         bodies[idx].position = glm::vec3((col - (BODY_COLS - 1) * 0.5f) * 2.5f, 1.5f + row * 2.8f, 0.0f); // centers columns around x=0, with 2.5 unit spacing + first body sits just above the floor --> each row is 2.8 units higher
            
    //         bodies[idx].mass = materials[col].mass;
    //         bodies[idx].inverseMass = materials[col].inverseMass;
    //         bodies[idx].restitution = materials[col].restitution;
    //     }
    // }

    const int BODY_COUNT = 20;

    struct MaterialDef {
        float mass;
        float inverseMass;
        float restitution;
    };

    const MaterialDef materials[4] = {
        {5.0f, 1.0f / 5.0f, 0.10f},
        {1.0f, 1.0f,        0.40f},
        {0.3f, 1.0f / 0.3f, 0.85f},
        {4.0f, 1.0f / 4.0f, 0.60f},
    };

    std::vector<RigidBody> bodies(BODY_COUNT);

    // Random number generator
    std::random_device rd;
    std::mt19937 rng(rd());

    // Spawn region
    std::uniform_real_distribution<float> xDist(-4.0f, 4.0f);
    std::uniform_real_distribution<float> zDist(-4.0f, 4.0f);
    std::uniform_real_distribution<float> yOffset(0.0f, 0.5f);

    for (int i = 0; i < BODY_COUNT; ++i)
    {
        const MaterialDef& mat = materials[i % 4];

        bodies[i].position = glm::vec3(
            xDist(rng),
            8.0f + i * 2.2f + yOffset(rng),
            zDist(rng)
        );

        bodies[i].mass = mat.mass;
        bodies[i].inverseMass = mat.inverseMass;
        bodies[i].restitution = mat.restitution;
    }

    // Fixed timestep 
    static constexpr float FIXED_DT = 1.0f / 60.0f;
    float accumulator = 0.0f;
    float lastTime = static_cast<float>(glfwGetTime());
    float lastDebugTime = static_cast<float>(glfwGetTime());
 
    while (!glfwWindowShouldClose(window)) {
 
        // Frame time 
        const float currentTime = static_cast<float>(glfwGetTime());
        float frameTime         = currentTime - lastTime;
        frameTime               = std::min(frameTime, 0.25f);
        lastTime                = currentTime;
        accumulator            += frameTime;
 
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
            std::cout << "[Physics] "
                      << "Bodies: "        << bodies.size()
                      << "  |  Pairs: "    << pairCount
                      << "  |  Contacts: " << physicsSolver.lastContactCount
                      << "  |  Time: "     << std::fixed << std::setprecision(2)
                      << physicsTimeMs     << " ms\n";
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