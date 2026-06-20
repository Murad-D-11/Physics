#include "cube.h"

Cube::Cube(float sideLength) {
    // Constructor body: two steps, each delegated to a private helper for clarity
    // Step 1: compute all vertex positions/colours and triangle indices, in CPU memory
    // Step 2: upload that CPU data into GPU buffers (VBO/EBO) and configure VAO
    buildGeometry(sideLength);
    setupBuffers();
}

Cube::~Cube() {
    // Destructor: GPU resources aren't automatically freed like CPU memory is
    // We must explicitly delete them here to avoid leaking GPU memory
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

void Cube::buildGeometry(float sideLength) {
    const float h = sideLength / 2.0f; // cube is centered at the origin, so each face sits 'h' units away from the center

    struct FaceColor { float r, g, b; }; // groups three floats into readable colour value instead of passing r/g/b seperately
    const FaceColor front {1.0f, 0.2f, 0.2f}; // +z red
    const FaceColor back {0.2f, 1.0f, 0.2f}; // -z green
    const FaceColor left {0.2f, 0.4f, 1.0f}; // -x blue
    const FaceColor right {1.0f, 1.0f, 0.2f}; // +x yellow
    const FaceColor top {1.0f, 0.2f, 1.0f}; // +y magenta
    const FaceColor bottom {0.2f, 1.0f, 1.0f}; // -y cyan

    auto pushFace = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, FaceColor col) {
        // A lambda (i.e. an anonymous function) capturing the encolsing scope by reference ([&])
        // so it can modify vertices and indices directly. Takes 4 corner point + 1 colour, and
        // appends the data for one full cube face.

        const unsigned int base = static_cast<unsigned int>(vertices.size() / 6); // 'vertices' currently holds N*6 floats for N vertices already added, so dividing by 6 gives the index of the next vertex to be added
        const glm::vec3 corners[4] = {a, b, c, d}; // bundles 4 corner arguments into a small fixed-size array, instead of repeating code 4 times

        for (const auto& corner : corners) {
            // Iterates over each of the 4 corners
            // 'const auto&' avoids copying each glm::vec3 unnecessarily

            vertices.push_back(corner.x); // appends the x-component for this vertex
            vertices.push_back(corner.y); // same with y
            vertices.push_back(corner.z); // same with z
            vertices.push_back(col.r); // appends the red colour component
            vertices.push_back(col.g); // same with green
            vertices.push_back(col.b); // same with blue
        }

        // Adds 6 indices (2 triangles) describing how to connect this face's 4 corners.
        // 'base' lets these indices point at the correct absolute position in the shared 'vertices' array, regardless
        // of how many vertices earlier faces already added
        // Triangle 1: corners 0, 1, 2. Triangle 2: corners 2, 3, 0 (together cover the whole quad with no overlap)
        indices.insert(indices.end(), {
            base + 0, base + 1, base + 2, 
            base + 2, base + 3, base + 0
        });
    };

    // Each face listed with corners going counter-clockwise as seen from outside the cube
    pushFace({-h, -h, h}, {h, -h, h}, {h, h, h}, {-h, h, h}, front); // +z
    pushFace({h, -h, -h}, {-h, -h, -h}, {-h, h, -h}, {h, h, -h}, back); // -z
    pushFace({-h, -h, -h}, {-h, -h, h}, {-h, h, h}, {-h, h, -h}, left); // -x
    pushFace({h, -h, h}, {h, -h, -h}, {h, h, -h}, {h, h, h}, right); // +x
    pushFace({-h, h, h}, {h, h, h}, {h, h, -h}, {-h, h, -h}, top); // +y
    pushFace({-h, -h, -h}, {h, -h, -h}, {h, -h, h}, {-h, -h, h}, bottom); // -y
}

void Cube::setupBuffers() {
    glGenVertexArrays(1, &VAO); // asks OpenGL to reserve 1 new vertex array object and store its id in VAO
    glGenBuffers(1, &VBO); // reserves 1 new buffer object for vertex data, id stored in VBO
    glGenBuffers(1, &EBO); // reserves 1 new buffer object for index data, id stored in EBO

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    // Uploads the CPU-side vertices data into the currently bound VBO
    // - size in bytes = number of floats * sizeof(float)
    // - vertices.data() gives a raw pointer to the underlying array
    // - GL_STATIC_DRAW hints to the driver this data won't change often, so it can optimize storage accordingly (opposite of GL_DYNAMIC_DRAW)
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Binds EBO as the active GL_ELEMENT_ARRAY_BUFFER, this binding also gets remembered as part of the VAO's own state
    // Uploads the CPU-side indices data into the currently-bound EBO, same reasoning as the vertex upload above
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    const GLsizei stride = 6 * sizeof(float); // how many bytes to get from one vertex's data to the next. Each vertex is 6 floats (3 position + 3 colour), so a stride is 6 floats' worth of bytes
    
    // Describes attribute #0, matches layout location = 0 in the vertex shader
    // 3 components (x, y, z), each a GL_FLOAT, not normalized (GL_FALSE), 'stride' bytes betweem consecutive vertices' position data,
    // starting at a byte offset 0 within each vertex
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(0); // attributes are disabled by default, this turns attribute #0 on so the GPU actually reads from it when drawing

    // Describes attribute #1, matches the colour layout location = 1 in the vertex shader
    // Same behaviour
    // Same stride, but starting at byte offset (3 * sizeof(float)) within each vertex (this is where the colour section appears in the vertex data)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1); // enables the colour attrib
    glBindVertexArray(0); // unbinds the VAO so later, unrelated OpenGL calls elsewhere in the program don't accidentally modify this cube's VAO state by mistake
}

void Cube::draw() const {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}