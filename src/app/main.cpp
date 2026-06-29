#include <iostream>
#include <glad/glad.h> // must come before glfw
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

Camera* activeCamera = nullptr; // global raw pointer to the one Camera the program uses (functions can't capture C++ context the way lambdas can, so a global is the standard way to reach application objects from inside them)
bool isDragging = false; // tracks whether the lmb is currently held down, so mouse movement only orbits the camera while actively dragging

// Tracks the coordinates of the cursor, 0 and 0 by default
double lastMouseX = 0.0;
double lastMouseY = 0.0;

/** 
 * Supports the frame of the window to be changed in real time
 * @param window The GLFW window in the current context
 * @param width The updated width
 * @param height The updated height
*/
void framebufferSizeChanged(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

/**
 * Tracks whether the left mouse button is currently held down for orbiting
 * @param window The GLFW window in the current context
 * @param button The mouse button pressed
 * @param action Tracked event
 * @param mods Which modifier keys were held (i.e. shift, alt, ctrl, etc.)
 */
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button != GLFW_MOUSE_BUTTON_LEFT) return; // ignores any other event that was not lmb click

    if (action == GLFW_PRESS) {
        isDragging = true; // start tracking drag movement
        glfwGetCursorPos(window, &lastMouseX, &lastMouseY); // records cursor's current position for delta calculations
    } else if (action == GLFW_RELEASE) {
        isDragging = false; // stops tracking drag movement
    }
}

/**
 * While dragging, feeds mouse movement deltas to the camera as orbit input
 * @param window The window in the current context
 * @param xPos The cursor's current x coordinate
 * @param yPos The cursor's current y coordinate
 */
void cursorPosCallback(GLFWwindow* window, double xPos, double yPos) {
    if (!isDragging || activeCamera == nullptr) return; // ignore if the mouse isn't being dragged, or camera has not been registered

    const float deltaX = static_cast<float>(xPos - lastMouseX); // how far cursor moved since last call in x direction
    const float deltaY = static_cast<float>(yPos - lastMouseY); // same thing but for y direction

    lastMouseX = xPos; // updates the remembered position so the next call computes a fresh delta
    lastMouseY = yPos; // same for y

    activeCamera->processMouseDrag(deltaX, -deltaY); // dragging the mouse up should tilt the view up over the cube, so the screen-space deltaY (which grows downward) is negated for pitch
}

/**
 * Scroll wheel zooms the orbit camera in and out
 * @param window The window in the current context
 * @param xOffset The scroll amount in x direction
 * @param yOffset The scroll amount in y direction
 */
void scrollCallback(GLFWwindow* window, double xOffset, double yOffset) {
    if (activeCamera == nullptr) return; // guard against camera not existing yet
    activeCamera->processScroll(static_cast<float>(yOffset)); // forwards vertical scroll amount to the camera
}

int main() {
    // ensures glfw was initialized
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // opengl version hints
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Fixes compatability issues with Mac
    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    const GLFWvidmode* VideoMode = glfwGetVideoMode(glfwGetPrimaryMonitor()); // retrieves current viewing specifications of the current monitor
    const auto ScreenHeight = VideoMode->height; // retreives the current monitor's height (height is the limiting variable in terms of screen dimensions, usually)

    GLFWwindow* window = glfwCreateWindow(ScreenHeight / 2, ScreenHeight / 2, "Hello Physics Engine!", NULL, NULL); // sets up a window that is visible on all monitors

    // if the window object returns null, prevent any errors by closing the program early
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window); // important to call before proceeding with any further OpenGL functions

    // ensures glad was initialized
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // Event handlers
    glfwSetFramebufferSizeCallback(window, framebufferSizeChanged);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);

    Render renderer;
    Cube cube(1.0f);
    Camera camera;
    activeCamera = &camera;

    RigidBody body; // body.position starts at (0, 5, 0) as set in rigidbody.h, so that it falls
    PhysicsSolver physicsSolver; // floor collision logic

    // vvvvv Fixed timestep setup + accumulator vvvvv

    static constexpr float FIXED_DT = 1.0f / 60.0f; // simulation always "steps" in exactly 1/60 second increments, regardless of the frame rendering speed
    float accumulator = 0.0f; // tracks the leftover real time that hasn't been consumed by the physics steps yet
    float lastTime = static_cast<float>(glfwGetTime()); // records the previous frame started, so we can compute how long this frame actually took

    // Render whilst the window is open, listen for any inputs
    while (!glfwWindowShouldClose(window)) {
        const float currentTime = static_cast<float>(glfwGetTime());
        float frameTime = currentTime - lastTime; // how many real seconds have elapsed sinced the last frame

        frameTime = std::min(frameTime, 0.25f); // if the frame took more than 0.25 seconds (e.g. the window was dragged), clamp it. The accumulator could build up so much time that the loop may run hundreds of steps at once, causing the cube to teleport
        lastTime = currentTime; // updates the reference point
        accumulator += frameTime; // adds this frame's real elapsed time to the pool of time waiting to be simulate

        /**
         * Drains the accumulator in fixed 1/60 second chunks, stepping physics sinulation once per chunk.
         * If the frame was fast (e.g. 1/120 seconds), this runs once or none at all.
         * If the frame was slow (e.g. 1/20 seconds), this runs 3 times to catch up, always using the fixed step size.
         */
        while (accumulator >= FIXED_DT) {
            physicsSolver.step(body, FIXED_DT);
            accumulator -= FIXED_DT; // subtracts one consumed step
        }

        // ===== Render ===== //

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        const float aspectRatio = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f; // avoids the divide by zero, preventing crash from minimilizing the window
        
        renderer.renderFrame(cube, camera, aspectRatio, body.position);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // ^^^^^ Fixed timestep setup + accumulator ^^^^^

    glfwTerminate();
    return 0;
}

