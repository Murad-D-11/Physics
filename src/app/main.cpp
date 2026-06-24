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

    RigidBody body;

    // Render the window whilst it is open, listen for any inputs
    while (!glfwWindowShouldClose(window)) {
        int width, height;

        glfwGetFramebufferSize(window, &width, &height); // queries the window's current framebuffer size, so that every frame during resizing of the window updates the aspect ratio immediately
        const float aspectRatio = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f; // avoids a divide-by-zero crash if the window is minimized (height becomes zero)

        body.position += glm::vec3 {0.0001, 0.0001, 0.0001}; // rigidbody test

        renderer.renderFrame(cube, camera, aspectRatio, body.position); // clears the screen and draws the cube using the camera's current view/projection matrices and this frame's aspect ratio
        glfwSwapBuffers(window); // OpenGL renders to an off-screen "back buffer", so this swapts it with the visible "front buffer" in order for the newly-drawn frame to actually appear on screen
        glfwPollEvents(); // process any pending window/input events
    }

    glfwTerminate();
    return 0;
}

