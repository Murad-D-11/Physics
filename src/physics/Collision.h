#pragma once
#include "obb.h"
#include "CollisionInfo.h"
#include <vector>

struct RigidBody;

// Speculative-contact margin (metres). Contacts are generated when bodies are
// within this distance of touching, not only once they already overlap. The
// velocity solver then removes exactly the closing velocity that would breach
// the surface this step, so bodies settle at the surface instead of sinking in
// and being pushed back out (the source of resting jitter / slide / sink-in).
inline constexpr float SPECULATIVE_MARGIN = 0.05f;

struct SATResult {
    bool colliding = false;
    float penetration = 0.0f;

    glm::vec3 normal = glm::vec3(0.0f); // always points from A toward B

    int axisType = -1; // 0-2: face of A, 3-5: face of B, 6-14: edge-edge
    int axisIndexA = -1; // which axis of A (for edge-edge: which edge direction)
    int axisIndexB = -1; // which axis of B (for edge-edge: which edge direction)
};

/**
 * Separation query result used by continuous collision detection.
 * distance > 0  : boxes are apart by this gap (normal points A -> B)
 * distance <= 0 : boxes overlap; -distance is the SAT penetration depth
 */
struct DistanceResult {
    bool overlapping = false;
    float distance = 0.0f;
    glm::vec3 normal = glm::vec3(0.0f); // points from A toward B
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

        /**
         * SAT-based separation distance between two OBBs. When disjoint, returns
         * the gap and the axis of maximum separation (a conservative lower bound
         * on true distance, which is exactly what conservative advancement needs).
         */
        static DistanceResult distanceOBB(const OBB& a, const OBB& b);

        // Position correction (kept for reference but replaced by Baumgarte in the solver)
        static void resolvePenetration(RigidBody& a, RigidBody& b, const CollisionInfo& info); // pushes body A and B so they no longer overlap
    private:
        /**
         * Sutherland-Hodgman: clips polygon input against a single plane
         * defined by (planeNormal, planeOffset). Returns the clipped polygon.
         */
        static std::vector<glm::vec3> clipPolygonAgainstPlane(const std::vector<glm::vec3>& input, const glm::vec3& planeNormal, float planeOffset);

        // For edge-edge contacts: finds the closest points on two line segments
        static void closestPointsOnSegments(const glm::vec3& p1, const glm::vec3& d1, float len1, const glm::vec3& p2, const glm::vec3& d2, float len2, glm::vec3& out1, glm::vec3& out2);
};
