#pragma once

#include <glad/glad.h>
#include <vector>

/**
 * A UV-sphere mesh for rendering spherical rigid bodies.
 * Follows the same pattern as Cube: owns GPU buffers and exposes draw().
 * The sphere has unit radius; scaling is applied via the model matrix.
 */
class Sphere {
public:
    explicit Sphere(int stacks = 16, int slices = 24);
    ~Sphere();
    Sphere(const Sphere&) = delete;
    Sphere& operator=(const Sphere&) = delete;

    void draw() const;

private:
    void buildGeometry(int stacks, int slices);
    void setupBuffers();

    std::vector<float> vertices;       // x,y,z, r,g,b per vertex
    std::vector<unsigned int> indices;
    int indexCount = 0;

    GLuint VAO = 0, VBO = 0, EBO = 0;
};
