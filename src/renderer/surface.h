#pragma once

#include <vector>

struct Surface {
    std::vector<float> xyzArray; // list of all coordinates

    // define a 2D surface
    Surface(std::size_t VertexCountPerSide = 60) {
        const std::size_t CoordinateCount = VertexCountPerSide * VertexCountPerSide * 3;

        /** 
         * range of -1 to 1 coordinate = 2. 
         * if there are n vertices, the number of equivalent segments between them is n - 1.
         * therefore, distance from one vertex to another is length of coordinate axis / number of equivalent segments.
        */
        const float vertexStep = 2.0 / (VertexCountPerSide - 1);
        
        xyzArray.resize(CoordinateCount); // set the size of the list to the number of all possible coordinates
        
        // iterate through all possible vertices present on the surface, append each vertex set of coordinates to xyzArray
        for (std::size_t row = 0; row < VertexCountPerSide; ++row) {
            for (std::size_t column = 0; column < VertexCountPerSide; ++column) {
                const auto x = -1.0f + column * vertexStep;
                const auto y = -1.0f + row * vertexStep;
                const auto z = 0.0f;

                // define the start of the vertex coordinate point for xyzArray
                const auto vertexStartIndex = (row * VertexCountPerSide + column) * 3;

                // x, y, z format
                xyzArray[vertexStartIndex + 0] = x;
                xyzArray[vertexStartIndex + 1] = y;
                xyzArray[vertexStartIndex + 2] = z;
            }
        }
    }
};