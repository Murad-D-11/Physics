#pragma once

#include <glad/glad.h>

/**
 * A large flat quad at y=0, used as the visible ground plane.
 * The procedural grid pattern is handled entirely in the shader —
 * this class just owns the geometry (4 vertices, 2 triangles).
 */
class Ground {
public:
    explicit Ground(float halfExtent = 20.0f);
    ~Ground();
    Ground(const Ground&) = delete;
    Ground& operator=(const Ground&) = delete;

    void draw() const;

private:
    GLuint VAO = 0, VBO = 0, EBO = 0;
};
