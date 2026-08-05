#include "render.h"
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

// ============================================================================
// Cube Shaders
// ============================================================================

const char* Render::vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;

    out vec3 vColor;

    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
        vColor = aColor;
    }
)";

const char* Render::fragmentShaderSource = R"(
    #version 330 core
    in vec3 vColor;
    out vec4 FragColor;
    uniform bool colliding;

    void main() {
        if (colliding) {
            FragColor = vec4(1.0, 0.15, 0.15, 1.0);
        } else {
            FragColor = vec4(vColor, 1.0);
        }
    }
)";

// ============================================================================
// Ground Shaders (procedural grid)
// ============================================================================

const char* Render::groundVertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;

    out vec3 worldPos;

    void main() {
        vec4 world = model * vec4(aPos, 1.0);
        worldPos = world.xyz;
        gl_Position = projection * view * world;
    }
)";

const char* Render::groundFragmentShaderSource = R"(
    #version 330 core
    in vec3 worldPos;
    out vec4 FragColor;

    void main() {
        // Grid spacing: 1 world unit between lines
        float gridSize = 1.0;

        // Compute grid lines from world-space xz coordinates
        vec2 coord = worldPos.xz / gridSize;
        vec2 grid = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
        float line = min(grid.x, grid.y);

        // line = 0 on the grid line, increases away from it
        // Convert to a 0-1 mask: 1 on lines, 0 away from lines
        float gridMask = 1.0 - min(line, 1.0);

        // White ground, dark gray lines
        vec3 groundColor = vec3(0.92, 0.92, 0.92);
        vec3 lineColor = vec3(0.4, 0.4, 0.4);
        vec3 color = mix(groundColor, lineColor, gridMask);

        // Fade out grid at distance to avoid moire
        float dist = length(worldPos.xz);
        float fade = 1.0 - smoothstep(10.0, 20.0, dist);
        color = mix(vec3(0.92), color, fade);

        FragColor = vec4(color, 1.0);
    }
)";

// ============================================================================
// Construction / Destruction
// ============================================================================

Render::Render() {
    // Compile cube shader
    GLuint vs = compileShader(vertexShaderSource, GL_VERTEX_SHADER);
    GLuint fs = compileShader(fragmentShaderSource, GL_FRAGMENT_SHADER);
    shaderProgram = linkProgram(vs, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);

    // Compile ground shader
    GLuint gvs = compileShader(groundVertexShaderSource, GL_VERTEX_SHADER);
    GLuint gfs = compileShader(groundFragmentShaderSource, GL_FRAGMENT_SHADER);
    groundShaderProgram = linkProgram(gvs, gfs);
    glDeleteShader(gvs);
    glDeleteShader(gfs);

    glEnable(GL_DEPTH_TEST);
}

Render::~Render() {
    glDeleteProgram(shaderProgram);
    glDeleteProgram(groundShaderProgram);
}

GLuint Render::compileShader(const std::string& source, GLenum shaderType) const {
    GLuint shader = glCreateShader(shaderType);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader compilation failed ("
                  << (shaderType == GL_VERTEX_SHADER ? "vertex" : "fragment")
                  << "): " << infoLog << std::endl;
    }
    return shader;
}

GLuint Render::linkProgram(GLuint vertexShader, GLuint fragmentShader) const {
    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Shader program linking failed: " << infoLog << std::endl;
    }
    return program;
}

// ============================================================================
// Drawing
// ============================================================================

void Render::beginFrame() const {
    glClearColor(0.08f, 0.08f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Render::drawBody(const Cube& cube, const Camera& camera, float aspectRatio, glm::vec3 position, glm::quat orientation, bool isColliding) const {
    glUseProgram(shaderProgram);

    const glm::mat4 translation = glm::translate(glm::mat4(1.0f), position);
    const glm::mat4 rotation = glm::mat4_cast(orientation);
    const glm::mat4 model = translation * rotation;
    const glm::mat4 view = camera.getViewMatrix();
    const glm::mat4 projection = camera.getProjectionMatrix(aspectRatio);

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniform1i(glGetUniformLocation(shaderProgram, "colliding"), isColliding ? 1 : 0);

    cube.draw();
}

void Render::drawGround(const Ground& ground, const Camera& camera, float aspectRatio) const {
    glUseProgram(groundShaderProgram);

    const glm::mat4 model = glm::mat4(1.0f); // ground sits at origin, no transform needed
    const glm::mat4 view = camera.getViewMatrix();
    const glm::mat4 projection = camera.getProjectionMatrix(aspectRatio);

    glUniformMatrix4fv(glGetUniformLocation(groundShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(groundShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(groundShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    ground.draw();
}
