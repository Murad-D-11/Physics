#include "collision.h"
#include "rigidbody.h"
#include <algorithm>
#include <cmath>

// std::vector<CollisionInfo> Collision::test(const AABB& a, const AABB& b) {
//     std::vector<CollisionInfo> manifold;

//     // vvvvv Separating Axis Theorem (simplified!!) vvvvv //
//     const float xOverlap = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
//     const float yOverlap = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
//     const float zOverlap = std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z);

//     if (xOverlap <= 0.0f || yOverlap <= 0.0f || zOverlap <= 0.0f) {
//         return manifold; // no collision
//     }

//     const glm::vec3 centerA = (a.min + a.max) * 0.5f;
//     const glm::vec3 centerB = (b.min + b.max) * 0.5f;
//     const glm::vec3 fromAtoB = centerB - centerA;

//     glm::vec3 normal;
//     float penetration;
//     int axis; // 0 = x, 1 = y, 2 = z -- which axis the contact "face" is perpendicular to

//     if (xOverlap <= yOverlap && xOverlap <= zOverlap) {
//         axis = 0;
//         penetration = xOverlap;
//         normal = glm::vec3(fromAtoB.x > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
//     } else if (yOverlap <= xOverlap && yOverlap <= zOverlap) {
//         axis = 1;
//         penetration = yOverlap;
//         normal = glm::vec3(0.0f, fromAtoB.y > 0.0f ? 1.0f : -1.0f, 0.0f);
//     } else {
//         axis = 2;
//         penetration = zOverlap;
//         normal = glm::vec3(0.0f, 0.0f, fromAtoB.z > 0.0f ? 1.0f : -1.0f);
//     }
//     // ^^^^^ Separating Axis Theorem (simplified!!) ^^^^^ //

//     /**
//      * The contact "manifold" is the overlap rectangle on the two axes
//      * perpendicular to the normal -- same rectangle the old single-point
//      * version collapsed down to its centroid. Every AABB is axis-aligned,
//      * so unlike generateFloorContacts (rotated box vs. plane, where each
//      * corner has its own depth), all 4 corners here share one penetration
//      * and one normal -- only their position differs, which is exactly
//      * what gives multi-point contact its resistance to torque.
//      */
//     const glm::vec3 overlapMin(std::max(a.min.x, b.min.x), std::max(a.min.y, b.min.y), std::max(a.min.z, b.min.z));
//     const glm::vec3 overlapMax(std::min(a.max.x, b.max.x), std::min(a.max.y, b.max.y), std::min(a.max.z, b.max.z));
//     const glm::vec3 mid = (overlapMin + overlapMax) * 0.5f;

//     glm::vec3 corners[4];
//     if (axis == 0) {
//         corners[0] = {mid.x, overlapMin.y, overlapMin.z};
//         corners[1] = {mid.x, overlapMax.y, overlapMin.z};
//         corners[2] = {mid.x, overlapMax.y, overlapMax.z};
//         corners[3] = {mid.x, overlapMin.y, overlapMax.z};
//     } else if (axis == 1) {
//         corners[0] = {overlapMin.x, mid.y, overlapMin.z};
//         corners[1] = {overlapMax.x, mid.y, overlapMin.z};
//         corners[2] = {overlapMax.x, mid.y, overlapMax.z};
//         corners[3] = {overlapMin.x, mid.y, overlapMax.z};
//     } else {
//         corners[0] = {overlapMin.x, overlapMin.y, mid.z};
//         corners[1] = {overlapMax.x, overlapMin.y, mid.z};
//         corners[2] = {overlapMax.x, overlapMax.y, mid.z};
//         corners[3] = {overlapMin.x, overlapMax.y, mid.z};
//     }

//     manifold.reserve(4);
//     for (const auto& corner : corners) {
//         CollisionInfo info;
//         info.collided = true;
//         info.point = corner;
//         info.normal = normal;
//         info.penetration = penetration;
//         manifold.push_back(info);
//     }

//     return manifold;
// }

/**
 * ======================================
 * OBB-SAT: 15-axis Separating Axis Test
 * ======================================
 */

/**
 * Projects an OBB onto a world-space axis and returns the half-extent of
 * the projection (the radius of the OBB's shadow on that axis)
 * 
 * Formula: sum of |dot(axis, obbAxis[i])| * halfExtent[i] for i in {0, 1, 2}
 */
static float projectOBBOntoAxis(const OBB& obb, const glm::vec3& axis) {
    return std::abs(glm::dot(axis, obb.axes[0])) * obb.halfExtents.x
        + std::abs(glm::dot(axis, obb.axes[1])) * obb.halfExtents.y
        + std::abs(glm::dot(axis, obb.axes[2])) * obb.halfExtents.z;
}

SATResult Collision::testOBB(const OBB& a, const OBB& b) {
    SATResult result;
    result.colliding = false;

    const glm::vec3 d = b.center - a.center; // vector between center A and center B

    float minPenetration = std::numeric_limits<float>::max();
    glm::vec3 bestAxis(0.0f);
    int bestAxisType = -1;
    int bestAxisA = -1;
    int bestAxisB = -1;

    // Helper lambda: tests one separating axis. Returns falls if separated (early out), true if overlapping on this axis
    auto testAxis = [&](const glm::vec3& axis, int axisType, int idxA, int idxB) -> bool {
        const float len = glm::length(axis);
        if (len < 1e-6f) return true; // degenerate axis (parallel edges), skip

        const glm::vec3 n = axis / len; // normalize

        const float projA = projectOBBOntoAxis(a, n);
        const float projB = projectOBBOntoAxis(b, n);
        const float dist = std::abs(glm::dot(d, n));

        const float overlap = projA + projB - dist;

        if (overlap <= 0.0f) {
            result.colliding = false;
            return false; // separating axis found; no collision
        }

        /**
         * Track minimum penetration axis.
         * For edge-edge axes, bias slightly to prefer face contacts when depths are similar.
         * This avoids jittery edge contacts for resting face-to-face configurations.
         */
        const float biasedOverlap = (axisType >= 6) ? overlap * 1.05f : overlap;

        if (biasedOverlap < minPenetration) {
            minPenetration = biasedOverlap;
            bestAxis = n;
            bestAxisType = axisType;
            bestAxisA = idxA;
            bestAxisB = idxB;
        }

        return true;
    };

    // axes 0-2: face normals of A
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(a.axes[i], i, i, -1)) return result;
    }

    // axes 3-5: face normals of B
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(b.axes[i], 3 + i, -1, i)) return result;
    }

    // axes 6-14: cross products of edge direction (one from each box)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const glm::vec3 cross = glm::cross(a.axes[i], b.axes[j]);
            if (!testAxis(cross, 6 + i * 3 + j, i, j)) return result;
        }
    }

    // all 15 axes overlap: collision confirmed
    result.colliding = true;

    // Use the un-biased penetration for the best axis
    if (bestAxisType >= 6) {
        // recompute without bias for edge-edge
        const float len = glm::length(glm::cross(a.axes[bestAxisA], b.axes[bestAxisB]));
        if (len > 1e-6f) {
            const glm::vec3 n = glm::cross(a.axes[bestAxisA], b.axes[bestAxisB]) / len;

            const float projA = projectOBBOntoAxis(a, n);
            const float projB = projectOBBOntoAxis(b, n);
            const float dist = std::abs(glm::dot(d, n));

            result.penetration = projA + projB - dist;
            bestAxis = n;
        } else {
            result.penetration = minPenetration / 1.05f;
        }
    } else {
        result.penetration = minPenetration / 1.05f;
    }

    // Ensure normal points from A toward B
    if (glm::dot(bestAxis, d) < 0.0f) {
        bestAxis = -bestAxis;
    }

    result.normal = bestAxis;
    result.axisType = bestAxisType;
    result.axisIndexA = bestAxisA;
    result.axisIndexB = bestAxisB;

    return result;
}

/**
 * ================================
 * Contact Manifold Generation
 * ================================
 */

std::vector<glm::vec3> Collision::clipPolygonAgainstPlane(const std::vector<glm::vec3>& input, const glm::vec3& planeNormal, float planeOffset) {
    /**
     * Sutherland-Hodgman clipping of a convex polygon against a half-space.
     * Keeps vertices on the positive side of: dot(vertex, planeNormal) <= planeOffset
     */
    std::vector<glm::vec3> output;
    if (input.empty()) return output;

    for (std::size_t i = 0; i < input.size(); i++) {
        const glm::vec3& current = input[i];
        const glm::vec3& next = input[(i + 1) % input.size()];

        const float dCurrent = glm::dot(planeNormal, current) - planeOffset;
        const float dNext = glm::dot(planeNormal, next) - planeOffset;

        if (dCurrent <= 0.0f) {
            // current vertex is inside
            output.push_back(current);

            if (dNext > 0.0f) {
                // next is outside: emit intersection
                const float t = dCurrent / (dCurrent - dNext);
                output.push_back(current + t * (next - current));
            }
        } else if (dNext <= 0.0f) {
            // current outside, next inside: emit intersection
            const float t = dCurrent / (dCurrent - dNext);
            output.push_back(current + t * (next - current));
        }
    }

    return output;
}

void Collision::closestPointsOnSegments(const glm::vec3& p1, const glm::vec3& d1, float len1, const glm::vec3& p2, const glm::vec3& d2, float len2, glm::vec3& out1, glm::vec3& out2) {
    /**
     * Finds the closest points on two line segments:
     *  segment1: p1 + t*d1, t in [0, len1]
     *  segment2: p2 + s*d2, s in [0, len2]
     * d1 and d2 must be unit vectors
     */
    const glm::vec3 r = p1 - p2;

    const float a = glm::dot(d1, d1); // = 1 (unit)
    const float e = glm::dot(d2, d2); // = 1 (unit)
    const float f = glm::dot(d2, r);
    const float c = glm::dot(d1, r);
    const float b = glm::dot(d1, d2);

    const float denom = a * e - b * b;

    float t, s;
    if (std::abs(denom) < 1e-8f) {
        // segments are nearly parallel
        t = 0.0f;
        s = f / e;
    } else {
        t = (b * f - c * e) / denom;
        s = (a * f - b * c) / denom;
    }

    // Clamp to segment ranges
    t = std::clamp(t, 0.0f, len1);
    s = std::clamp(s, 0.0f, len2);

    out1 = p1 + t * d1;
    out2 = p2 + s * d2;
}

std::vector<CollisionInfo> Collision::generateManifold(const OBB& a, const OBB& b, const SATResult& sat) {
    std::vector<CollisionInfo> manifold;

    if (!sat.colliding) return manifold;

    if (sat.axisType >= 6) {
        // ====== Edge-edge contact: single contact point ======
        const int edgeA = sat.axisIndexA; // which axis of A the edge runs along
        const int edgeB = sat.axisIndexB;

        /**
         * Pick one edge from each box. The edge is at the center of the face perpendicular
         * to the separating axis, oriented along the edge direction. We need the edge closest
         * to the other box.
         */
        const glm::vec3 edgeDirA = a.axes[edgeA];
        const glm::vec3 edgeDirB = b.axes[edgeB];

        /**
         * Edge midpoint: center of the box +/- halfExtents along the two non-edge axes,
         * choosing the signs that bring it closest to the other box.
         */
        glm::vec3 edgeMidA = a.center;
        for (int i = 0; i < 3; ++i) {
            if (i == edgeA) continue;
            const float sign = glm::dot(a.axes[i], sat.normal) > 0.0f ? 1.0f : -1.0f;
            // we want the edge closest to B, so move toward B (along normal direction)
            edgeMidA += a.axes[i] * (a.halfExtents[i] * sign);
        }

        glm::vec3 edgeMidB = b.center;
        for (int i = 0; i < 3; ++i) {
            if (i == edgeB) continue;
            const float sign = glm::dot(b.axes[i], sat.normal) < 0.0f ? 1.0f : -1.0f;
            // move toward A (against normal direction)
            edgeMidB += b.axes[i] * (b.halfExtents[i] * sign);
        }

        glm::vec3 ptA, ptB;
        closestPointsOnSegments(edgeMidA, edgeDirA, a.halfExtents[edgeA] * 2.0f, edgeMidB, edgeDirB, b.halfExtents[edgeB] * 2.0f, ptA, ptB);

        CollisionInfo info;
        info.collided = true;
        info.point = (ptA + ptB) * 0.5f;
        info.normal = sat.normal;
        info.penetration = sat.penetration;
        // Feature ID: high bit set to mark edge-edge, encode edge indices
        info.featureId = 0x80000000u | (static_cast<uint32_t>(edgeA) << 16) | static_cast<uint32_t>(edgeB);
        manifold.push_back(info);
    } else {
        // ======= Face-face contact: clip incident face against reference face =======

        // Determine reference face and incident face
        const OBB* refBox;
        const OBB* incBox;
        int refAxisIndex;
        float refSign;
        bool flipNormal; // true if we swapped ref/inc relative to A/B ordering

        if (sat.axisType <= 2) {
            // reference face is on A
            refBox = &a;
            incBox = &b;
            refAxisIndex = sat.axisType;
            refSign = glm::dot(a.axes[refAxisIndex], sat.normal) > 0.0f ? 1.0f : -1.0f;
            flipNormal = false;
        } else {
            // reference face is on B
            refBox = &b;
            incBox = &a;
            refAxisIndex = sat.axisType - 3;
            refSign = glm::dot(b.axes[refAxisIndex], sat.normal) > 0.0f ? 1.0f : -1.0f;
            flipNormal = true;
        }

        const glm::vec3 refNormal = refBox->axes[refAxisIndex] * refSign;

        // Find incident face: the face of incBox most anti-parallel to refNormal
        int incAxisIndex = 0;
        float incSign = 1.0f;
        float minDot = std::numeric_limits<float>::max();

        for (int i = 0; i < 3; ++i) {
            float d = glm::dot(incBox->axes[i], refNormal);
            if (d < minDot) { minDot = d; incAxisIndex = i; incSign = 1.0f; }
            if (-d < minDot) { minDot = -d; incAxisIndex = i; incSign = -1.0f; }
        }

        /**
         * The incident face normal should point most against the reference normal
         * so the incident face is the one at -incSign direction (facing toward ref)
         */
        incSign = -incSign;

        // Get incident face vertices
        glm::vec3 incFaceVerts[4];
        incBox->getFaceVertices(incAxisIndex, incSign, incFaceVerts);

        // Build clipping panes from the 4 side edges of the reference face
        const int refT1 = (refAxisIndex + 1) % 3;
        const int refT2 = (refAxisIndex + 2) % 3;

        struct ClipPlane {
            glm::vec3 normal;
            float offset;
        };

        // The 4 side planes of the reference face
        const glm::vec3 refCenter = refBox->center + refBox->axes[refAxisIndex] * (refBox->halfExtents[refAxisIndex] * refSign);

        ClipPlane sidePlanes[4];
        sidePlanes[0] = { refBox->axes[refT1], glm::dot(refBox->axes[refT1], refCenter) + refBox->halfExtents[refT1] };
        sidePlanes[1] = { -refBox->axes[refT1], -glm::dot(refBox->axes[refT1], refCenter) + refBox->halfExtents[refT1] };
        sidePlanes[2] = { refBox->axes[refT2], glm::dot(refBox->axes[refT2], refCenter) + refBox->halfExtents[refT2] };
        sidePlanes[3] = { -refBox->axes[refT2], -glm::dot(refBox->axes[refT2], refCenter) + refBox->halfExtents[refT2] };

        // Clip the incident polygon against each side plane
        std::vector<glm::vec3> polygon(incFaceVerts, incFaceVerts + 4);

        for (int i = 0; i < 4; ++i) {
            polygon = clipPolygonAgainstPlane(polygon, sidePlanes[i].normal, sidePlanes[i].offset);
            if (polygon.empty()) break;
        }

        // Keep only points behind (or on) the reference face plane
        const float refFaceOffset = glm::dot(refNormal, refCenter);

        for (std::size_t i = 0; i < polygon.size(); ++i) {
            const float depth = refFaceOffset - glm::dot(refNormal, polygon[i]);

            if (depth >= 0.0f) {
                CollisionInfo info;
                info.collided = true;
                info.point = polygon[i];
                info.normal = sat.normal;
                info.penetration = depth;

                // Feature ID: encode referenc eface an dclipped vertex index
                const uint32_t refFaceId = static_cast<uint32_t>(refAxisIndex) | (refSign > 0.0f ? 0x10u : 0x00u);
                info.featureId = (refFaceId << 16) | static_cast<uint32_t>(i);
                if (flipNormal) info.featureId |= 0x40000000u; // mark ref=B

                manifold.push_back(info);
            }
        }
    }

    return manifold;
}

/**
 * ===============================================================================================
 * Position correction from a collision (legacy, replaced by Baumgarte bias in the solver)
 * ===============================================================================================
 */
void Collision::resolvePenetration(RigidBody& a, RigidBody& b, const CollisionInfo& info) {
    const float invMassSum = a.inverseMass + b.inverseMass;
    if (invMassSum == 0.0f) return; // both static, nothing to correct

    const float ratioA = a.inverseMass / invMassSum; // heavier / more-immovable body corrects less
    const float ratioB = b.inverseMass / invMassSum;

    a.position -= info.normal * info.penetration * ratioA;
    b.position += info.normal * info.penetration * ratioB;
}