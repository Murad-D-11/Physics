#include "render.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>


// R"(...)" is a C++ raw string literal: everything between the parentheses is taken word for word, so newlines/quotes don't need escaping.
// This whole block is plain GLSL (OpenGL Shading Language) source, compiled by the GPU driver at runtime, not by the C++ compiler.
// Copied what each line does:
//   #version 330 core                    -> use GLSL 3.30, core profile
//   layout (location = 0) in vec3 aPos;  -> per-vertex position, matches
//                                            glVertexAttribPointer(0, ...) in cube.cpp
//   layout (location = 1) in vec3 aColor;-> per-vertex color, matches
//                                            glVertexAttribPointer(1, ...) in cube.cpp
//   uniform mat4 model/view/projection;  -> values set once per draw call
//                                            from C++ (see renderFrame below),
//                                            shared by every vertex in this draw call
//   out vec3 vColor;                     -> passes color to the fragment shader,
//                                            interpolated automatically across each triangle
//   gl_Position = ...;                    -> required output: this vertex's final
//                                            clip-space position, via model -> view -> projection
//   vColor = aColor;                      -> forwards the input color unchanged
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

// The fragment shader runs once per pixel covered by a triangle.
//   in vec3 vColor;      -> interpolated color value from the vertex shader
//                           (blended between the 3 triangle corners' colors)
//   out vec4 FragColor;  -> required output: this pixel's final RGBA color
//   FragColor = vec4(vColor, 1.0);  -> uses the interpolated color, fully opaque
const char* Render::fragmentShaderSource = R"(
    #version 330 core
    in vec3 vColor;
    out vec4 FragColor;

    void main() {
        FragColor = vec4(vColor, 1.0);
    }
)";

Render::Render() {
    GLuint vertexShader = compileShader(vertexShaderSource, GL_VERTEX_SHADER); // compiles the vertex shader source into a GPU shader object
    GLuint fragmentShader = compileShader(fragmentShaderSource, GL_FRAGMENT_SHADER); // same thing, but with fragment

    shaderProgram = linkProgram(vertexShader, fragmentShader); // links both compiled shader objects into one shader program, stored for later use

    glDeleteShader(vertexShader); // frees the vertex shader container
    glDeleteShader(fragmentShader); // does the same with the fragment

    glEnable(GL_DEPTH_TEST); // turns on depth testing: OpenGL will now use the depth buffer to figure out which fragments are closest to camera, for correct drawing purposes
}

Render::~Render() {
    glDeleteProgram(shaderProgram); // frees the shader program container object to parse in the next one
}

GLuint Render::compileShader(const std::string& source, GLenum shaderType) const {
    GLuint shader = glCreateShader(shaderType); // new, empty shader

    const char* src = source.c_str(); // glShaderSource() needs a raw C-string pointer, not a std::string, so c_str() converts it
    glShaderSource(shader, 1, &src, nullptr); // Attaches the source code text to the shader object (the one that is shown in the R"(...)")
    // Additional info: '1' means we are passing one string, &src is a pointer to that string (the API expects an array of strings, even if there is just one),
    // nullptr means each string is null=terminated (so OpenGL does not need explicit lengths)

    glCompileShader(shader); // compiles into a GPU-executable form
    
    GLint success; // will hold whether compilation succeeded (GL_TRUE) or did not (GL_FALSE)
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success); // queries the shader object for its compile status, storing its status in 'success'

    if (!success) {
        char infoLog[512]; // a fixed-size buffer to receive the human-readable error message
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);

        std::cerr << "Shader compilation failed (" << (shaderType == GL_VERTEX_SHADER ? "vertex" : "fragment") << "): " << infoLog << std::endl;
    }

    return shader;
}

GLuint Render::linkProgram(GLuint vertexShader, GLuint fragmentShader) const {
    GLuint program = glCreateProgram(); // new, empty shader program object

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);

    glLinkProgram(program); // links all attached shaders together into one runnable GPU pipeline

    GLint success; // holds whether linking succeeded
    glGetProgramiv(program, GL_LINK_STATUS, &success);

    // Basically same thing with shaders, but for program
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Shader program linking failed: " << infoLog << std::endl;
    }

    return program;
}

void Render::renderFrame(const Cube& cube, const Camera& camera, float aspectRatio, glm::vec3 position) const {
    glClearColor(0.08f, 0.08f, 0.1f, 1.0f); // dark blue-gray for the screen, opaque
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clears two buffers: color and depth buffer (so that depth testing starts fresh with each frame)

    glUseProgram(shaderProgram); // makes this shader program the active one

    glm::mat4 model = glm::translate(glm::mat4(1.0f), position); // 4x4 identity matrix, no additional repositioning (now includes translation for RigidBody test)
    glm::mat4 view = camera.getViewMatrix(); // current view matrix (depends on its yaw, pitch, distance, since last frame from input)
    glm::mat4 projection = camera.getProjectionMatrix(aspectRatio); // projection matrix from the camera object

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model)); // finds where the "model" uniform is located to upload 'model' matrix to that location
    // Additional info: '1' = uploading 1 matrix, GL_FALSE = don't transpose it (GLM already stores matrices in the alyout OpenGL expects), 
    // glm::value_ptr(model) = raw float pointer to the matrix's data

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view)); // same process, uploading 'view' matrix to shader's "view" uniform
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection)); // same process, but with projection

    cube.draw(); // finally issues the actual GPU draw call, binds the cube's VAO and calls glDrawElements, using the program and unfirom set up above
}
