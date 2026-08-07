#include "collision.h"
#include "rigidbody.h"
#include <algorithm>
#include <cmath>

// ============================================================================
// OBB-SAT: 15-axis Separating Axis Test
// ============================================================================

/**
 * Projects an OBB onto a world-space axis and returns the half-extent of
 * the projection (the "radius" of the OBB's shadow on that axis).
 *
 * Formula: sum of |dot(axis, obbAxis[i])| * halfExtent[i] for i in {0,1,2}
 */
static float projectOBBOntoAxis(const OBB& obb, const glm::vec3& axis) {
    return std::abs(glm::dot(axis, obb.axes[0])) * obb.halfExtents.x
         + std::abs(glm::dot(axis, obb.axes[1])) * obb.halfExtents.y
         + std::abs(glm::dot(axis, obb.axes[2])) * obb.halfExtents.z;
}

SATResult Collision::testOBB(const OBB& a, const OBB& b) {
    SATResult result;
    result.colliding = false;

    const glm::vec3 d = b.center - a.center; // vector from A center to B center

    float minPenetration = std::numeric_limits<float>::max();
    glm::vec3 bestAxis(0.0f);
    int bestAxisType = -1;
    int bestAxisA = -1;
    int bestAxisB = -1;

    // Helper lambda: tests one separating axis.
    // Returns false if separated (early out), true if overlapping on this axis.
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
            return false; // separating axis found — no collision
        }

        // Track minimum penetration axis
        // For edge-edge axes, bias slightly to prefer face contacts when depths are similar.
        // This avoids jittery edge contacts for resting face-to-face configurations.
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

    // Axes 0-2: face normals of A
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(a.axes[i], i, i, -1)) return result;
    }

    // Axes 3-5: face normals of B
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(b.axes[i], 3 + i, -1, i)) return result;
    }

    // Axes 6-14: cross products of edge directions (one from each box)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const glm::vec3 cross = glm::cross(a.axes[i], b.axes[j]);
            if (!testAxis(cross, 6 + i * 3 + j, i, j)) return result;
        }
    }

    // All 15 axes overlap: collision confirmed
    result.colliding = true;
    // Use the un-biased penetration for the best axis
    if (bestAxisType >= 6) {
        // Recompute without bias for edge-edge
        const float len = glm::length(glm::cross(a.axes[bestAxisA], b.axes[bestAxisB]));
        if (len > 1e-6f) {
            const glm::vec3 n = glm::cross(a.axes[bestAxisA], b.axes[bestAxisB]) / len;
            const float projA = projectOBBOntoAxis(a, n);
            const float projB = projectOBBOntoAxis(b, n);
            const float dist = std::abs(glm::dot(d, n));
            result.penetration = projA + projB - dist;
            bestAxis = n;
        } else {
            result.penetration = minPenetration;
        }
    } else {
        result.penetration = minPenetration;
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

// ============================================================================
// Contact Manifold Generation
// ============================================================================

std::vector<glm::vec3> Collision::clipPolygonAgainstPlane(
    const std::vector<glm::vec3>& input,
    const glm::vec3& planeNormal,
    float planeOffset)
{
    /**
     * Sutherland-Hodgman clipping of a convex polygon against a half-space.
     * Keeps vertices on the positive side of: dot(vertex, planeNormal) <= planeOffset
     */
    std::vector<glm::vec3> output;
    if (input.empty()) return output;

    for (std::size_t i = 0; i < input.size(); ++i) {
        const glm::vec3& current = input[i];
        const glm::vec3& next = input[(i + 1) % input.size()];

        const float dCurrent = glm::dot(planeNormal, current) - planeOffset;
        const float dNext = glm::dot(planeNormal, next) - planeOffset;

        if (dCurrent <= 0.0f) {
            // Current vertex is inside
            output.push_back(current);
            if (dNext > 0.0f) {
                // Next is outside: emit intersection
                const float t = dCurrent / (dCurrent - dNext);
                output.push_back(current + t * (next - current));
            }
        } else if (dNext <= 0.0f) {
            // Current outside, next inside: emit intersection
            const float t = dCurrent / (dCurrent - dNext);
            output.push_back(current + t * (next - current));
        }
    }
    return output;
}

void Collision::closestPointsOnSegments(const glm::vec3& p1, const glm::vec3& d1, float len1, const glm::vec3& p2, const glm::vec3& d2, float len2, glm::vec3& out1, glm::vec3& out2) {
    /**
     * Finds the closest points on two line segments:
     *   segment1: p1 + t*d1, t in [0, len1]
     *   segment2: p2 + s*d2, s in [0, len2]
     * d1 and d2 must be unit vectors.
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
        // Segments are nearly parallel
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
        // ==== Edge-edge contact: single contact point ====
        const int edgeA = sat.axisIndexA; // which axis of A the edge runs along
        const int edgeB = sat.axisIndexB;

        // Pick one edge from each box. The edge is at the center of the face
        // perpendicular to the separating axis, oriented along the edge direction.
        // We need the edge closest to the other box.
        const glm::vec3 edgeDirA = a.axes[edgeA];
        const glm::vec3 edgeDirB = b.axes[edgeB];

        // Edge midpoint: center of the box ± halfExtents along the two non-edge axes,
        // choosing the signs that bring it closest to the other box.
        glm::vec3 edgeMidA = a.center;
        for (int i = 0; i < 3; ++i) {
            if (i == edgeA) continue;
            const float sign = glm::dot(a.axes[i], sat.normal) > 0.0f ? 1.0f : -1.0f;
            // We want the edge closest to B, so move toward B (along normal direction)
            edgeMidA += a.axes[i] * (a.halfExtents[i] * sign);
        }

        glm::vec3 edgeMidB = b.center;
        for (int i = 0; i < 3; ++i) {
            if (i == edgeB) continue;
            const float sign = glm::dot(b.axes[i], sat.normal) < 0.0f ? 1.0f : -1.0f;
            // Move toward A (against normal direction)
            edgeMidB += b.axes[i] * (b.halfExtents[i] * sign);
        }

        const glm::vec3 startA = edgeMidA - edgeDirA * a.halfExtents[edgeA];
        const glm::vec3 startB = edgeMidB - edgeDirB * b.halfExtents[edgeB];

        glm::vec3 ptA, ptB;
        closestPointsOnSegments(startA, edgeDirA, a.halfExtents[edgeA] * 2.0f, startB, edgeDirB, b.halfExtents[edgeB] * 2.0f, ptA, ptB);
        
        CollisionInfo info;
        info.collided = true;
        info.point = (ptA + ptB) * 0.5f;
        info.normal = sat.normal;
        info.penetration = sat.penetration;
        // Feature ID: high bit set to mark edge-edge, encode both edge indices
        info.featureId = 0x80000000u | (static_cast<uint32_t>(edgeA) << 16) | static_cast<uint32_t>(edgeB);
        manifold.push_back(info);

    } else {
        // ==== Face-face contact: clip incident face against reference face ====

        // Determine reference face and incident face
        const OBB* refBox;
        const OBB* incBox;
        int refAxisIndex;
        float refSign;
        bool flipNormal; // true if we swapped ref/inc relative to A/B ordering

        if (sat.axisType <= 2) {
            refBox = &a; incBox = &b;
            refAxisIndex = sat.axisType;
            flipNormal = false;
        } else {
            refBox = &b; incBox = &a;
            refAxisIndex = sat.axisType - 3;
            flipNormal = true;
        }

        // Reference face must point FROM the reference box TOWARD the incident box.
        // sat.normal is A->B, so toward-incident is +normal when ref=A, -normal when ref=B.
        const glm::vec3 refToIncident = flipNormal ? -sat.normal : sat.normal;
        refSign = glm::dot(refBox->axes[refAxisIndex], refToIncident) > 0.0f ? 1.0f : -1.0f;
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

        // Get incident face vertices
        glm::vec3 incFaceVerts[4];
        incBox->getFaceVertices(incAxisIndex, incSign, incFaceVerts);

        // Build clipping planes from the 4 side edges of the reference face
        const int refT1 = (refAxisIndex + 1) % 3;
        const int refT2 = (refAxisIndex + 2) % 3;

        struct ClipPlane {
            glm::vec3 normal;
            float offset;
        };

        // The 4 side planes of the reference face
        const glm::vec3 refCenter = refBox->center + refBox->axes[refAxisIndex] * (refBox->halfExtents[refAxisIndex] * refSign);

        ClipPlane sidePlanes[4];
        sidePlanes[0] = { refBox->axes[refT1],  glm::dot( refBox->axes[refT1], refCenter) + refBox->halfExtents[refT1]};
        sidePlanes[1] = {-refBox->axes[refT1], -glm::dot( refBox->axes[refT1], refCenter) + refBox->halfExtents[refT1]};
        sidePlanes[2] = { refBox->axes[refT2],  glm::dot( refBox->axes[refT2], refCenter) + refBox->halfExtents[refT2]};
        sidePlanes[3] = {-refBox->axes[refT2], -glm::dot( refBox->axes[refT2], refCenter) + refBox->halfExtents[refT2]};

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

                // Feature ID: encode reference face and clipped vertex index
                const uint32_t refFaceId = static_cast<uint32_t>(refAxisIndex) | (refSign > 0.0f ? 0x10u : 0x00u);
                info.featureId = (refFaceId << 16) | static_cast<uint32_t>(i);
                if (flipNormal) info.featureId |= 0x40000000u; // mark ref=B

                manifold.push_back(info);
            }
        }
    }

    return manifold;
}

// ============================================================================
// Position Correction (legacy, replaced by Baumgarte bias in the solver)
// ============================================================================

void Collision::resolvePenetration(RigidBody& a, RigidBody& b, const CollisionInfo& info) {
    const float invMassSum = a.inverseMass + b.inverseMass;
    if (invMassSum == 0.0f) return;

    const float ratioA = a.inverseMass / invMassSum;
    const float ratioB = b.inverseMass / invMassSum;

    a.position -= info.normal * info.penetration * ratioA;
    b.position += info.normal * info.penetration * ratioB;
}

DistanceResult Collision::distanceOBB(const OBB& a, const OBB& b) {
    DistanceResult result;
    const glm::vec3 d = b.center - a.center;

    // 15 candidate separating axes: 3 A faces, 3 B faces, 9 edge-edge crosses
    glm::vec3 axes[15];
    int n = 0;
    for (int i = 0; i < 3; ++i) axes[n++] = a.axes[i];
    for (int i = 0; i < 3; ++i) axes[n++] = b.axes[i];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            axes[n++] = glm::cross(a.axes[i], b.axes[j]);

    float bestGap = -std::numeric_limits<float>::max();
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);

    for (int k = 0; k < 15; ++k) {
        const float len = glm::length(axes[k]);
        if (len < 1e-6f) continue; // degenerate (parallel edges)
        const glm::vec3 axis = axes[k] / len;

        const float rA = std::abs(glm::dot(axis, a.axes[0])) * a.halfExtents.x
                       + std::abs(glm::dot(axis, a.axes[1])) * a.halfExtents.y
                       + std::abs(glm::dot(axis, a.axes[2])) * a.halfExtents.z;
        const float rB = std::abs(glm::dot(axis, b.axes[0])) * b.halfExtents.x
                       + std::abs(glm::dot(axis, b.axes[1])) * b.halfExtents.y
                       + std::abs(glm::dot(axis, b.axes[2])) * b.halfExtents.z;

        const float centerDist = glm::dot(d, axis);
        const float gap = std::abs(centerDist) - (rA + rB);

        if (gap > bestGap) {
            bestGap = gap;
            bestNormal = (centerDist < 0.0f) ? -axis : axis; // orient A -> B
        }
    }

    result.distance = bestGap;
    result.overlapping = (bestGap <= 0.0f);
    result.normal = bestNormal;
    return result;
}
