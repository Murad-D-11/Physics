#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

class Cube {
    public:
        explicit Cube(float sideLength = 1.0f); // constructor
        ~Cube(); // destructor: runs automatically when a Cube object is destroyed
        Cube(const Cube&) = delete;
        Cube& operator=(const Cube&) = delete;

        void draw() const; // issues the actual GPU draw call

    private:
        void buildGeometry(float sideLength); // fills vertices and indices with this cube's data
        void setupBuffers(); // takes the CPU-side vertices and indices data and uploads it into GPU buffer objects (VBO, EBO) wired up via a VAO
        
        std::vector<float> vertices; // flat list of floats, 6 per vertex: x, y, z (position), r, g, b (colour)
        std::vector<unsigned int> indices; // each group of 3 indices describes one triangle by referencing which vertices make up its 3 corners
        
        GLuint VAO = 0; // vertex array object handle. This remebers how to interpret a VBO's bytes, reducing the need to re-specify that every frame
        GLuint VBO = 0; // vertex buffer object handle. A block of GPU memory holding vertices
        GLuint EBO = 0; // element buffer object handle. A block of GPU memory holding indices, so glDrawElements knows which vertices to connect into triangles
};