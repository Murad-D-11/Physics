#include <iostream>
#include <algorithm>
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
bool isDragging      = false;
double lastMouseX    = 0.0;
double lastMouseY    = 0.0;

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
    const auto ScreenHeight      = VideoMode->height;
    GLFWwindow* window           = glfwCreateWindow(ScreenHeight / 2, ScreenHeight / 2, "Hello Physics Engine!", NULL, NULL);

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

    // Spawn 10 bodies at varied positions (stress test)

    // Default-constructs 10 RigidBodies. All fields get their default values (position=(0,5,0), velocity=0, scale=1, mass=1, isColliding=false).
    // We then overwrite each body's position individually since RigidBody has no constructor (it's plain data, no functions).
    std::vector<RigidBody> bodies(10);

    // Left cluster: three cubes in a vertical column, will stack on the floor
    bodies[0].position = glm::vec3(-3.0f, 10.0f, 0.0f);
    bodies[1].position = glm::vec3(-3.0f,  7.0f, 0.0f);
    bodies[2].position = glm::vec3(-3.0f,  4.0f, 0.0f);

    // Center cluster: two cubes placed only 0.6 units apart on X even though each cube is 1 unit wide they already overlap and will turn red
    bodies[3].position = glm::vec3( 0.0f, 8.0f, 0.0f);
    bodies[4].position = glm::vec3( 0.6f, 6.5f, 0.0f);
    bodies[5].position = glm::vec3( 0.0f, 3.5f, 0.0f);

    // Right cluster: three cubes at staggered depths (z offsets) so they spread visually and don't all collide at once
    bodies[6].position = glm::vec3( 3.0f, 9.0f,  1.0f);
    bodies[7].position = glm::vec3( 3.0f, 6.0f, -1.0f);
    bodies[8].position = glm::vec3( 3.0f, 3.0f,  0.0f);

    // Lone outlier far to the left should stay its normal colors until it eventually hits the floor. Useful to verify non-colliding bodies are not incorrectly flagged red.
    bodies[9].position = glm::vec3(-6.5f, 5.0f, 0.0f);

    // Fixed timestep setup

    // All physics steps are exactly this long, regardless of frame rate.
    // This makes the simulation deterministic: same inputs always produce the same trajectory.
    static constexpr float FIXED_DT = 1.0f / 60.0f;

    float accumulator = 0.0f;
    // Pool of real time waiting to be consumed by physics steps.

    float lastTime = static_cast<float>(glfwGetTime());
    // Timestamp of the previous frame, for computing frameTime.

    while (!glfwWindowShouldClose(window)) {
        // Fixed timestep accumulator

        const float currentTime = static_cast<float>(glfwGetTime());
        float frameTime         = currentTime - lastTime; // How many real seconds elapsed since the last frame.
        frameTime               = std::min(frameTime, 0.25f);
        // Cap at 0.25s if the window is dragged, a breakpoint fires, or the machine stutters badly, the accumulator could overflow and the physics
        // loop would run hundreds of steps in one frame, causing the simulation to effectively teleport. The cap limits "catch-up" to 15 steps max.
        lastTime    = currentTime;
        accumulator += frameTime;

        while (accumulator >= FIXED_DT) {
            // Physics pipeline: three ordered passes

            // Pass 1: apply gravity and integrate velocity into position.
            // All bodies integrate before any collision handling, so no body gets an unfair advantage from being processed first.
            for (auto& body : bodies) {
                physicsSolver.integrate(body, FIXED_DT);
            }

            // Pass 2: floor collision -> done as a separate pass so integration is complete for all bodies before any floor responses fire.
            for (auto& body : bodies) {
                physicsSolver.floorCollision(body);
            }

            // Pass 3: test all N*(N-1)/2 unique body pairs, set isColliding flags, and correct positions. Order matters: this runs
            // after integration and floor so AABBs are computed from fully-updated positions.
            physicsSolver.detectAndResolve(bodies);

            accumulator -= FIXED_DT;
        }

        // Render all 10 bodies into the same frame

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        const float aspectRatio = height > 0
            ? static_cast<float>(width) / static_cast<float>(height)
            : 1.0f;

        // Clears the color and depth buffers ONCE for this frame.
        // Must happen before any drawBody() calls; the clear resets the
        // depth buffer so all 10 cubes depth-test against each other correctly.
        renderer.beginFrame();

        /** 
         * Draws this body at its current physics position with its collisiondebug color. 
         * Each call sets its own model matrix uniform, so the same cube mesh gets "stamped" at each body's position in turn.
         * The depth buffer accumulates all 10 cubes: a face of body[7] can correctly occlude a face of body[3] that's behind it.
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