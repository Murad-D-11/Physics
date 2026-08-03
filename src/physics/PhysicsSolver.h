#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "rigidbody.h"
#include "collisioninfo.h"

class PhysicsSolver {
    public:
        PhysicsSolver();
        ~PhysicsSolver();

        void integrate(RigidBody& body, float deltaTime);
        // void floorCollision(RigidBody& body);
        void detectAndResolve(std::vector<RigidBody>& bodies);

        int lastContactCount = 0; // updated by detectAndResolve() --> number of active contacts

    private:
        /**
         * One colliding pair and its precomputed geometry
         * Built inside detectAndResolve()
         */
        struct Contact {
            RigidBody* a;
            RigidBody* b;
            CollisionInfo info;
        };

        /**
         * AUDIT: settleFlatIfResting() is disabled for being too unrealistic + its helper constants ANGULAR_REST_THRESHOLD and SETTLE_DISTANCE 
        */ 
        void settleFlatIfResting(RigidBody& body); // once a body's spin has decayed below ANGULAR_REST_THRESHOLD, and it is resting close to the floor, this snaps its orientation to lay it flat on the ground
        void applyImpulse(RigidBody& a, RigidBody& b, const CollisionInfo& info); // computes and applies a single collision impulse using relative velocity
        // void applyFloorImpulse(RigidBody& body, const glm::vec3& contactPoint); // resolves a single box-plane contact point. called once per touching corner, per solver iteration, from floorCollision()

        std::vector<CollisionInfo> generateFloorContacts(const RigidBody& body) const; // builds the floor's contact manifold for one body
        RigidBody floorBody; // the floor, represented as ordinary static RigidBody (inverseMass = 0)

        // Tuning Constants:

        static constexpr int SOLVER_ITERATIONS = 10; // how many times the impulse pass revisits each contact per step
        static constexpr float FLOOR_Y = 0.0f; // the y-coordinate of the floor plane
        static constexpr float FLOOR_THICKNESS = 1.0f; // floorBody's own extend along y
        static constexpr float FLOOR_HALF_EXTENT = 500.0f; // floorBody's half-extent along x/z
        static constexpr float REST_THRESHOLD = 0.5f; // reciprocity of forces, i.e. the "bounciness" of the cube (half its previous height)
        static constexpr float PENETRATION_SLOP = 0.02f; // minimum penetration depth position before position correction executes
        static constexpr float PENETRATION_CORRECTION = 0.8f; // fraction of excess penetration corrected per step --> correct 80%, but leave 20% for the next step
        static constexpr float FACE_CONTACT_EPSILON = 0.005f; // corners within this height of the single deepest corner are treated as touching the same resting face (what would otherwise be a single-corner contact, it becomes a multi-point face contact)
        
        /**
         * AUDIT: DISABLED vvvv
         */
        static constexpr float ANGULAR_REST_THRESHOLD = 0.02f; // torque that opposes the spin <--- (also disabled)
        static constexpr float SETTLE_DISTANCE = 0.05f; // how close a body's lowest corner must be to the floor before settleFlatIfResting() snaps its orientation flat on the ground <--- (also also disabled)
};