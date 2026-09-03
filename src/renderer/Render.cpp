#include "render.h"
#include <glm/gtc/matrix_transform.hpp>
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
        float gridSize = 1.0;

        vec2 coord = worldPos.xz / gridSize;
        vec2 grid = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
        float line = min(grid.x, grid.y);
        float gridMask = 1.0 - min(line, 1.0);

        vec3 groundColor = vec3(0.92, 0.92, 0.92);
        vec3 lineColor = vec3(0.4, 0.4, 0.4);
        vec3 color = mix(groundColor, lineColor, gridMask);

        float dist = length(worldPos.xz);
        float fade = 1.0 - smoothstep(10.0, 20.0, dist);
        color = mix(vec3(0.92), color, fade);

        FragColor = vec4(color, 1.0);
    }
)";

// ============================================================================
// Pause Icon Shaders (screen-space overlay)
// ============================================================================

const char* Render::pauseVertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    void main() {
        gl_Position = vec4(aPos, 0.0, 1.0);
    }
)";

const char* Render::pauseFragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;
    void main() {
        FragColor = vec4(1.0, 1.0, 1.0, 0.85);
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

    // Compile pause icon shader
    GLuint pvs = compileShader(pauseVertexShaderSource, GL_VERTEX_SHADER);
    GLuint pfs = compileShader(pauseFragmentShaderSource, GL_FRAGMENT_SHADER);
    pauseShaderProgram = linkProgram(pvs, pfs);
    glDeleteShader(pvs);
    glDeleteShader(pfs);

    // Pause icon geometry: two vertical bars in NDC (top-left corner)
    float pauseVerts[] = {
        // Bar 1 (two triangles)
        -0.92f,  0.82f,
        -0.88f,  0.82f,
        -0.88f,  0.94f,
        -0.92f,  0.82f,
        -0.88f,  0.94f,
        -0.92f,  0.94f,
        // Bar 2 (two triangles)
        -0.85f,  0.82f,
        -0.81f,  0.82f,
        -0.81f,  0.94f,
        -0.85f,  0.82f,
        -0.81f,  0.94f,
        -0.85f,  0.94f,
    };

    glGenVertexArrays(1, &pauseVAO);
    glGenBuffers(1, &pauseVBO);
    glBindVertexArray(pauseVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pauseVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(pauseVerts), pauseVerts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // Status HUD shader: screen-space quad in NDC with a uniform color.
    {
        const char* svs = R"(
            #version 330 core
            layout (location = 0) in vec2 aPos;
            void main() { gl_Position = vec4(aPos, 0.0, 1.0); }
        )";
        const char* sfs = R"(
            #version 330 core
            out vec4 FragColor;
            uniform vec3 uColor;
            void main() { FragColor = vec4(uColor, 0.9); }
        )";
        GLuint v = compileShader(svs, GL_VERTEX_SHADER);
        GLuint f = compileShader(sfs, GL_FRAGMENT_SHADER);
        statusShaderProgram = linkProgram(v, f);
        glDeleteShader(v);
        glDeleteShader(f);
    }

    // Status HUD geometry: a dynamic 2D quad (6 verts, 2 floats each) whose NDC
    // rectangle is rewritten per draw so one buffer serves every slot.
    glGenVertexArrays(1, &statusVAO);
    glGenBuffers(1, &statusVBO);
    glBindVertexArray(statusVAO);
    glBindBuffer(GL_ARRAY_BUFFER, statusVBO);
    glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // Line rendering: dynamic VBO for 2 vertices (pos+color = 6 floats each)
    glGenVertexArrays(1, &lineVAO);
    glGenBuffers(1, &lineVBO);
    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
}

Render::~Render() {
    glDeleteProgram(shaderProgram);
    glDeleteProgram(groundShaderProgram);
    glDeleteProgram(pauseShaderProgram);
    glDeleteVertexArrays(1, &pauseVAO);
    glDeleteBuffers(1, &pauseVBO);
    glDeleteVertexArrays(1, &lineVAO);
    glDeleteBuffers(1, &lineVBO);
    glDeleteProgram(statusShaderProgram);
    glDeleteVertexArrays(1, &statusVAO);
    glDeleteBuffers(1, &statusVBO);
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

void Render::drawBody(const Cube& cube, const Camera& camera, float aspectRatio, glm::vec3 position, glm::quat orientation, glm::vec3 scale, bool isColliding) const {
    glUseProgram(shaderProgram);

    const glm::mat4 translation = glm::translate(glm::mat4(1.0f), position);
    const glm::mat4 rotation = glm::mat4_cast(orientation);
    const glm::mat4 scaling = glm::scale(glm::mat4(1.0f), scale);
    const glm::mat4 model = translation * rotation * scaling;
    const glm::mat4 view = camera.getViewMatrix();
    const glm::mat4 projection = camera.getProjectionMatrix(aspectRatio);

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniform1i(glGetUniformLocation(shaderProgram, "colliding"), isColliding ? 1 : 0);

    cube.draw();
}

void Render::drawSphere(const Sphere& sphere, const Camera& camera, float aspectRatio, glm::vec3 position, glm::quat orientation, float radius, bool isColliding) const {
    glUseProgram(shaderProgram);

    const glm::mat4 translation = glm::translate(glm::mat4(1.0f), position);
    const glm::mat4 rotation = glm::mat4_cast(orientation);
    const glm::mat4 scaling = glm::scale(glm::mat4(1.0f), glm::vec3(radius)); // uniform scale
    const glm::mat4 model = translation * rotation * scaling;
    const glm::mat4 view = camera.getViewMatrix();
    const glm::mat4 projection = camera.getProjectionMatrix(aspectRatio);

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniform1i(glGetUniformLocation(shaderProgram, "colliding"), isColliding ? 1 : 0);

    sphere.draw();
}

void Render::drawLine(const Camera& camera, float aspectRatio, glm::vec3 from, glm::vec3 to, glm::vec3 color) const {
    glUseProgram(shaderProgram);

    const glm::mat4 model = glm::mat4(1.0f); // identity — vertices are in world space
    const glm::mat4 view = camera.getViewMatrix();
    const glm::mat4 projection = camera.getProjectionMatrix(aspectRatio);

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniform1i(glGetUniformLocation(shaderProgram, "colliding"), 0);

    // Upload 2 vertices: position (3) + color (3) each
    float verts[12] = {
        from.x, from.y, from.z, color.x, color.y, color.z,
        to.x,   to.y,   to.z,   color.x, color.y, color.z
    };

    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);
}

void Render::drawGround(const Ground& ground, const Camera& camera, float aspectRatio) const {
    glUseProgram(groundShaderProgram);

    const glm::mat4 model = glm::mat4(1.0f);
    const glm::mat4 view = camera.getViewMatrix();
    const glm::mat4 projection = camera.getProjectionMatrix(aspectRatio);

    glUniformMatrix4fv(glGetUniformLocation(groundShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(groundShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(groundShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    ground.draw();
}

void Render::drawPauseIcon() const {
    glUseProgram(pauseShaderProgram);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBindVertexArray(pauseVAO);
    glDrawArrays(GL_TRIANGLES, 0, 12);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

void Render::drawDottedPath(const Camera& camera, float aspectRatio,
                            const std::vector<glm::vec3>& points, glm::vec3 color) const {
    if (points.size() < 2) return;
    // Render every other span between consecutive samples so the trajectory
    // reads as a dotted/dashed line rather than a solid one. Reuses drawLine.
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        if ((i & 1u) == 0u) { // draw even spans, skip odd ones -> dashes
            drawLine(camera, aspectRatio, points[i], points[i + 1], color);
        }
    }
}

void Render::drawPath(const Camera& camera, float aspectRatio,
                      const std::vector<glm::vec3>& points, glm::vec3 color) const {
    if (points.size() < 2) return;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        drawLine(camera, aspectRatio, points[i], points[i + 1], color);
    }
}

void Render::drawStatusBar(int slot, bool on) const {
    // A small filled rectangle in the top-left, stacked downward by slot.
    // Green = on, red = off. Font-free stand-in for a text status readout.
    const float w = 0.05f;   // NDC width
    const float h = 0.03f;   // NDC height
    const float x0 = -0.98f; // left edge
    const float top = 0.78f; // first bar top (just below the pause icon area)
    const float gap = 0.02f;

    const float yTop = top - static_cast<float>(slot) * (h + gap);
    const float yBot = yTop - h;
    const float x1 = x0 + w;

    const float verts[12] = {
        x0, yBot,  x1, yBot,  x1, yTop,
        x0, yBot,  x1, yTop,  x0, yTop,
    };

    const glm::vec3 color = on ? glm::vec3(0.15f, 0.85f, 0.25f)   // green
                               : glm::vec3(0.85f, 0.2f, 0.2f);    // red

    glUseProgram(statusShaderProgram);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUniform3fv(glGetUniformLocation(statusShaderProgram, "uColor"), 1, glm::value_ptr(color));

    glBindVertexArray(statusVAO);
    glBindBuffer(GL_ARRAY_BUFFER, statusVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}
