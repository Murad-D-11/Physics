#include <iostream>
#include <glad/glad.h> // must come before glfw
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace std; 

/** 
 * Renders the window in the current context
 * @param window The window in the current context
*/
void render(GLFWwindow* window) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // rgb values of black, make it opaque
    glClear(GL_COLOR_BUFFER_BIT);
}

/** 
 * Supports the frame of the window to be changed in real time
 * @param window The GLFW window in the current context
 * @param width The updated width
 * @param height The updated height
*/
void framebufferSizeChanged(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    render(window);
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
        return -1;
    }

    glfwMakeContextCurrent(window); // important to call before proceeding with any further OpenGL functions

    // ensures glad was initialized
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glfwSetFramebufferSizeCallback(window, framebufferSizeChanged);

    // render the window whilst it is open, listen for any inputs
    while (!glfwWindowShouldClose(window)) {
        render(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

