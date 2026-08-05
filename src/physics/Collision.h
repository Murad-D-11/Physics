#pragma once
#include "obb.h"
#include "CollisionInfo.h"
#include <vector>

struct RigidBody;

struct SATResult {
    bool colliding = false;
    float penetration = 0.0f;
    
    glm::vec3 normal = glm::vec3(0.0f); // always points from A toward B
    
    int axisType = -1; // 0-2: face of A, 3-5: face of B, 6-14: edge-edge
    int axisIndexA = -1; // which axis of A (for edge-edge: which edge direction)
    int axisIndexB = -1; // which axis of B (for edge-edge: which edge direction)
};

/**
 * Handles collision detection and contact manifold generation for OBBs.
 */
class Collision {
    public:
        /**
         * Full OBB-SAT test: returns whether the two oriented boxes overlap,
         * the minimum penetration depth, and the separating axis.
         */
        static SATResult testOBB(const OBB& a, const OBB& b);

        /**
         * Given a positive SAT result, generates the contact manifold
         * (1-N contact points with penetration depths and feature IDs).
         */
        static std::vector<CollisionInfo> generateManifold(const OBB& a, const OBB& b, const SATResult& sat);

        // Position correction (kept for reference but replaced by Baumgarte in the solver)
        // static std::vector<CollisionInfo> test(const AABB& a, const AABB& b); // tests whether two AABBs overlap, returns a CollisionInfo which describes status of the collision event
        static void resolvePenetration(RigidBody& a, RigidBody& b, const CollisionInfo& info); // pushes body A and B so they no longer overlap
    private:
        /**
         * Sutherland-Hodgman: clips polygon input against a single plane
         * defined by (planeNormal, planeOffset). Returns the clipped polygon.
         */
        static std::vector<glm::vec3> clipPolygonAgainstPlane(const std::vector<glm::vec3>& input, const glm:;vec3& planeNormal, float planeOffset);

        // For edge-edge contacts: finds the closest points on two line segments
        static void closestPointsOnSegments(const glm::vec3& p1, const glm::vec3& d1, float len1, const glm::vec3& p2, const glm::vec3& d2, float len2, glm::vec3& out1, glm::vec3& out2);
};