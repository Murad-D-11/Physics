#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include "cube.h"
#include "sphere.h"
#include "ground.h"
#include "camera.h"

/**
 * Owns shader programs and knows how to draw Cubes, the Ground plane,
 * and overlay UI elements (pause icon). No knowledge of GLFW or input.
 */
class Render {
public:
    Render();
    ~Render();
    Render(const Render&) = delete;
    Render& operator=(const Render&) = delete;

    void beginFrame() const;
    void drawBody(const Cube& cube, const Camera& camera, float aspectRatio, glm::vec3 position, glm::quat orientation, glm::vec3 scale, bool isColliding) const;
    void drawSphere(const Sphere& sphere, const Camera& camera, float aspectRatio, glm::vec3 position, glm::quat orientation, float radius, bool isColliding) const;
    void drawLine(const Camera& camera, float aspectRatio, glm::vec3 from, glm::vec3 to, glm::vec3 color) const;
    void drawGround(const Ground& ground, const Camera& camera, float aspectRatio) const;
    void drawPauseIcon() const;

private:
    GLuint compileShader(const std::string& source, GLenum shaderType) const;
    GLuint linkProgram(GLuint vertexShader, GLuint fragmentShader) const;

    GLuint shaderProgram = 0;
    GLuint groundShaderProgram = 0;
    GLuint pauseShaderProgram = 0;
    GLuint pauseVAO = 0, pauseVBO = 0;
    GLuint lineVAO = 0, lineVBO = 0;

    static const char* vertexShaderSource;
    static const char* fragmentShaderSource;
    static const char* groundVertexShaderSource;
    static const char* groundFragmentShaderSource;
    static const char* pauseVertexShaderSource;
    static const char* pauseFragmentShaderSource;
};
