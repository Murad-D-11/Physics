#include "sphere.h"
#include <cmath>

static constexpr float PI = 3.14159265358979323846f;

Sphere::Sphere(int stacks, int slices) {
    buildGeometry(stacks, slices);
    setupBuffers();
}

Sphere::~Sphere() {
    if (VAO) glDeleteVertexArrays(1, &VAO);
    if (VBO) glDeleteBuffers(1, &VBO);
    if (EBO) glDeleteBuffers(1, &EBO);
}

void Sphere::buildGeometry(int stacks, int slices) {
    // Unit sphere (radius 1). Scaling handled by model matrix.
    // Vertex format: x, y, z, r, g, b (matches Cube).
    const float baseR = 0.3f, baseG = 0.55f, baseB = 0.85f; // blueish tint

    for (int i = 0; i <= stacks; ++i) {
        const float phi = PI * static_cast<float>(i) / static_cast<float>(stacks); // 0..PI
        const float y = std::cos(phi);
        const float ringR = std::sin(phi);

        for (int j = 0; j <= slices; ++j) {
            const float theta = 2.0f * PI * static_cast<float>(j) / static_cast<float>(slices);
            const float x = ringR * std::cos(theta);
            const float z = ringR * std::sin(theta);

            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);

            // Slight colour variation based on normal direction for shading effect
            const float shade = 0.6f + 0.4f * std::max(0.0f, y * 0.5f + 0.5f);
            vertices.push_back(baseR * shade);
            vertices.push_back(baseG * shade);
            vertices.push_back(baseB * shade);
        }
    }

    // Indices: two triangles per quad
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            const unsigned int a = i * (slices + 1) + j;
            const unsigned int b = a + slices + 1;

            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(a + 1);

            indices.push_back(a + 1);
            indices.push_back(b);
            indices.push_back(b + 1);
        }
    }

    indexCount = static_cast<int>(indices.size());
}

void Sphere::setupBuffers() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                 vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                 indices.data(), GL_STATIC_DRAW);

    // Position attribute (location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color attribute (location 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void Sphere::draw() const {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
