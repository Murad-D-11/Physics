#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include "cube.h"
#include "camera.h"

/**
 * Owns the shader program and knows how to draw a Cube from a Camera's
 * point of view. Deliberately has no knowledge of GLFW, windows, or input (main.cpp does it)
 */
class Render {
    public:
        Render(); // constructor: compiles and links the shader program, and enables depth testing
        ~Render(); // destructor: deletes the shader program from the GPU
        Render(const Render&) = delete;
        Render& operator=(const Render&) = delete;

        void renderFrame(const Cube& cube, const Camera& camera, float aspectRatio) const; // clears the screen and draws cube as seen through camera

    private:
        GLuint compileShader(const std::string& source, GLenum shaderType) const; // compiles one shader (vertex or fragment), returns its OpenGL handle
        GLuint linkProgram(GLuint vertexShader, GLuint fragmentShader) const; // links a compiled vertex shader and fragment shader together into one usable shader program, and returns the program's handle
        GLuint shaderProgram = 0;

        static const char* vertexShaderSource; // static string holding the vertex shader's GLSL source code, the actual text lives in render.cpp
        static const char* fragmentShaderSource; // same for the fragment shader
};