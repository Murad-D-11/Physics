#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include "cube.h"
#include "ground.h"
#include "camera.h"

/**
 * Owns shader programs and knows how to draw Cubes and the Ground plane.
 * Has no knowledge of GLFW, windows, or input.
 */
class Render {
public:
    Render();
    ~Render();
    Render(const Render&) = delete;
    Render& operator=(const Render&) = delete;

    void beginFrame() const;
    void drawBody(const Cube& cube, const Camera& camera, float aspectRatio, glm::vec3 position, glm::quat orientation, bool isColliding) const;
    void drawGround(const Ground& ground, const Camera& camera, float aspectRatio) const;

private:
    GLuint compileShader(const std::string& source, GLenum shaderType) const;
    GLuint linkProgram(GLuint vertexShader, GLuint fragmentShader) const;

    GLuint shaderProgram = 0;       // cube shader
    GLuint groundShaderProgram = 0; // ground grid shader

    static const char* vertexShaderSource;
    static const char* fragmentShaderSource;
    static const char* groundVertexShaderSource;
    static const char* groundFragmentShaderSource;
};
