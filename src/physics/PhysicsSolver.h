#pragma once
#include <glm/glm.hpp>
#include "rigidbody.h"

class PhysicsSolver {
    public:
        PhysicsSolver();
        ~PhysicsSolver();

        void step(RigidBody& body, float deltaTime); // ensures that deltaTime is always 1/60 seconds, never the raw frame time; does that so by "stepping" the simulation forward by deltaTime seconds

    private:
        static constexpr float halfHeight = 0.5f; // half of the cube's side length
        static constexpr float floorY = 0.0f; // the y-coordinate of the floor plane
        static constexpr float restitution = 0.5f; // reciprocity of forces, i.e. the "bounciness" of the cube (half its previous height)
};