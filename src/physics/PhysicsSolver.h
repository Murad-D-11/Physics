#pragma once
#include <glm/glm.hpp>
#include <vector>
#include "rigidbody.h"
#include "collisioninfo.h"

class PhysicsSolver {
    public:
        PhysicsSolver();
        ~PhysicsSolver();

        void integrate(RigidBody& body, float deltaTime);
        void floorCollision(RigidBody& body);
        void detectAndResolve(std::vector<RigidBody>& bodies);

        int lastContactCount = 0; // updated by detectAndResolve() --> number of active contacts

    private:
        /**
         * One colliding pair and its precomputed geometry
         * Built inside detectAndResolve()
         */
        struct Contact {
            std::size_t indexA;
            std::size_t indexB;
            CollisionInfo info;
        };

        void applyImpulse(RigidBody& a, RigidBody& b, const CollisionInfo& info); // computes and applies a single collision impulse using relative velocity

        // Tuning Constants:

        static constexpr int SOLVER_ITERATIONS = 10; // how many times the impulse pass revisits each contact per step
        static constexpr float FLOOR_Y = 0.0f; // the y-coordinate of the floor plane
        static constexpr float REST_THRESHOLD = 0.5f; // reciprocity of forces, i.e. the "bounciness" of the cube (half its previous height)
        static constexpr float PENETRATION_SLOP = 0.02f; // minimum penetration depth position before position correction executes
        static constexpr float PENETRATION_CORRECTION = 0.8f; // fraction of excess penetration corrected per step --> correct 80%, but leave 20% for the next step
};