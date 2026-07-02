#pragma once
#include <glm/glm.hpp>
#include "rigidbody.h"

class PhysicsSolver {
    public:
        PhysicsSolver();
        ~PhysicsSolver();

        void integrate(RigidBody& body, float deltaTime);
        void floorCollision(RigidBody& body);
        void detectAndResolve(std::vector<RigidBody>& bodies);

    private:
        static constexpr float floorY = 0.0f; // the y-coordinate of the floor plane
        static constexpr float restitution = 0.5f; // reciprocity of forces, i.e. the "bounciness" of the cube (half its previous height)
};