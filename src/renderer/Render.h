#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>
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
    // Draw a poly-line as a dotted/dashed line: renders alternating short
    // segments with gaps between them. Used for predicted trajectories.
    void drawDottedPath(const Camera& camera, float aspectRatio,
                        const std::vector<glm::vec3>& points, glm::vec3 color) const;
    void drawGround(const Ground& ground, const Camera& camera, float aspectRatio) const;
    void drawPauseIcon() const;
    // Overlay HUD status indicator: a small filled bar in the top-left, stacked
    // by `slot` (0,1,2,...), green when `on`, red when off. A font-free stand-in
    // for a text readout (Recording / Prediction state).
    void drawStatusBar(int slot, bool on) const;

private:
    GLuint compileShader(const std::string& source, GLenum shaderType) const;
    GLuint linkProgram(GLuint vertexShader, GLuint fragmentShader) const;

    GLuint shaderProgram = 0;
    GLuint groundShaderProgram = 0;
    GLuint pauseShaderProgram = 0;
    GLuint statusShaderProgram = 0; // screen-space colored quad (HUD bars)
    GLuint pauseVAO = 0, pauseVBO = 0;
    GLuint lineVAO = 0, lineVBO = 0;
    GLuint statusVAO = 0, statusVBO = 0; // screen-space HUD status bars

    static const char* vertexShaderSource;
    static const char* fragmentShaderSource;
    static const char* groundVertexShaderSource;
    static const char* groundFragmentShaderSource;
    static const char* pauseVertexShaderSource;
    static const char* pauseFragmentShaderSource;
};
