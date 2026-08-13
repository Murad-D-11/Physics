#include "collision.h"
#include "rigidbody.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

// ============================================================================
// OBB-SAT: 15-axis Separating Axis Test
// ============================================================================

static float projectOBBOntoAxis(const OBB& obb, const glm::vec3& axis) {
    return std::abs(glm::dot(axis, obb.axes[0])) * obb.halfExtents.x
         + std::abs(glm::dot(axis, obb.axes[1])) * obb.halfExtents.y
         + std::abs(glm::dot(axis, obb.axes[2])) * obb.halfExtents.z;
}

SATResult Collision::testOBB(const OBB& a, const OBB& b) {
    SATResult result;
    result.colliding = false;

    const glm::vec3 d = b.center - a.center;

    float minPenetration = std::numeric_limits<float>::max();
    glm::vec3 bestAxis(0.0f);
    int bestAxisType = -1;
    int bestAxisA = -1;
    int bestAxisB = -1;

    auto testAxis = [&](const glm::vec3& axis, int axisType, int idxA, int idxB) -> bool {
        const float len = glm::length(axis);
        if (len < 1e-6f) return true;

        const glm::vec3 n = axis / len;

        const float projA = projectOBBOntoAxis(a, n);
        const float projB = projectOBBOntoAxis(b, n);
        const float dist = std::abs(glm::dot(d, n));

        const float overlap = projA + projB - dist;

        // was: if (overlap <= 0.0f) { ... return false; }
        if (overlap <= -SPECULATIVE_MARGIN) {
            result.colliding = false;
            return false;
        }

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

    for (int i = 0; i < 3; ++i) {
        if (!testAxis(a.axes[i], i, i, -1)) return result;
    }
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(b.axes[i], 3 + i, -1, i)) return result;
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const glm::vec3 cross = glm::cross(a.axes[i], b.axes[j]);
            if (!testAxis(cross, 6 + i * 3 + j, i, j)) return result;
        }
    }

    result.colliding = true;

    if (bestAxisType >= 6) {
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
// Tagged Sutherland-Hodgman clipping (stable per-vertex feature identity)
// ============================================================================

namespace {
    // A polygon vertex carrying a stable geometric feature tag:
    //   0..3  -> original incident-face corner index
    //   4..11 -> intersection on clip plane p: 4 + p*2 + dir (dir 0 exit, 1 enter)
    // A surviving vertex keeps its own tag, so a contact point's featureId no
    // longer shifts when a *different* vertex appears/disappears at a clip edge.
    struct ClipVert {
        glm::vec3 pos;
        uint32_t feature;
    };

    std::vector<ClipVert> clipTagged(const std::vector<ClipVert>& input,
                                     const glm::vec3& planeNormal, float planeOffset,
                                     int planeId) {
        std::vector<ClipVert> output;
        if (input.empty()) return output;

        const uint32_t exitTag  = 4u + static_cast<uint32_t>(planeId) * 2u + 0u;
        const uint32_t enterTag = 4u + static_cast<uint32_t>(planeId) * 2u + 1u;

        for (std::size_t i = 0; i < input.size(); ++i) {
            const ClipVert& cur = input[i];
            const ClipVert& nxt = input[(i + 1) % input.size()];

            const float dCur = glm::dot(planeNormal, cur.pos) - planeOffset;
            const float dNxt = glm::dot(planeNormal, nxt.pos) - planeOffset;

            if (dCur <= 0.0f) {
                output.push_back(cur);
                if (dNxt > 0.0f) {
                    const float t = dCur / (dCur - dNxt);
                    output.push_back({ cur.pos + t * (nxt.pos - cur.pos), exitTag });
                }
            } else if (dNxt <= 0.0f) {
                const float t = dCur / (dCur - dNxt);
                output.push_back({ cur.pos + t * (nxt.pos - cur.pos), enterTag });
            }
        }
        return output;
    }
}

// ============================================================================
// Contact Manifold Generation
// ============================================================================

std::vector<glm::vec3> Collision::clipPolygonAgainstPlane(
    const std::vector<glm::vec3>& input,
    const glm::vec3& planeNormal,
    float planeOffset)
{
    // Retained for API compatibility (no longer used by generateManifold).
    std::vector<glm::vec3> output;
    if (input.empty()) return output;

    for (std::size_t i = 0; i < input.size(); ++i) {
        const glm::vec3& current = input[i];
        const glm::vec3& next = input[(i + 1) % input.size()];

        const float dCurrent = glm::dot(planeNormal, current) - planeOffset;
        const float dNext = glm::dot(planeNormal, next) - planeOffset;

        if (dCurrent <= 0.0f) {
            output.push_back(current);
            if (dNext > 0.0f) {
                const float t = dCurrent / (dCurrent - dNext);
                output.push_back(current + t * (next - current));
            }
        } else if (dNext <= 0.0f) {
            const float t = dCurrent / (dCurrent - dNext);
            output.push_back(current + t * (next - current));
        }
    }
    return output;
}

void Collision::closestPointsOnSegments(const glm::vec3& p1, const glm::vec3& d1, float len1, const glm::vec3& p2, const glm::vec3& d2, float len2, glm::vec3& out1, glm::vec3& out2) {
    const glm::vec3 r = p1 - p2;
    const float a = glm::dot(d1, d1);
    const float e = glm::dot(d2, d2);
    const float f = glm::dot(d2, r);
    const float c = glm::dot(d1, r);
    const float b = glm::dot(d1, d2);

    const float denom = a * e - b * b;

    float t, s;
    if (std::abs(denom) < 1e-8f) {
        t = 0.0f;
        s = f / e;
    } else {
        t = (b * f - c * e) / denom;
        s = (a * f - b * c) / denom;
    }

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
        const int edgeA = sat.axisIndexA;
        const int edgeB = sat.axisIndexB;

        const glm::vec3 edgeDirA = a.axes[edgeA];
        const glm::vec3 edgeDirB = b.axes[edgeB];

        glm::vec3 edgeMidA = a.center;
        for (int i = 0; i < 3; ++i) {
            if (i == edgeA) continue;
            const float sign = glm::dot(a.axes[i], sat.normal) > 0.0f ? 1.0f : -1.0f;
            edgeMidA += a.axes[i] * (a.halfExtents[i] * sign);
        }

        glm::vec3 edgeMidB = b.center;
        for (int i = 0; i < 3; ++i) {
            if (i == edgeB) continue;
            const float sign = glm::dot(b.axes[i], sat.normal) < 0.0f ? 1.0f : -1.0f;
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
        info.featureId = 0x80000000u | (static_cast<uint32_t>(edgeA) << 16) | static_cast<uint32_t>(edgeB);
        manifold.push_back(info);

    } else {
        // ==== Face-face contact: clip incident face against reference face ====

        const OBB* refBox;
        const OBB* incBox;
        int refAxisIndex;
        float refSign;
        bool flipNormal;

        if (sat.axisType <= 2) {
            refBox = &a; incBox = &b;
            refAxisIndex = sat.axisType;
            flipNormal = false;
        } else {
            refBox = &b; incBox = &a;
            refAxisIndex = sat.axisType - 3;
            flipNormal = true;
        }

        const glm::vec3 refToIncident = flipNormal ? -sat.normal : sat.normal;
        refSign = glm::dot(refBox->axes[refAxisIndex], refToIncident) > 0.0f ? 1.0f : -1.0f;
        const glm::vec3 refNormal = refBox->axes[refAxisIndex] * refSign;

        int incAxisIndex = 0;
        float incSign = 1.0f;
        float minDot = std::numeric_limits<float>::max();
        for (int i = 0; i < 3; ++i) {
            float dd = glm::dot(incBox->axes[i], refNormal);
            if (dd < minDot) { minDot = dd; incAxisIndex = i; incSign = 1.0f; }
            if (-dd < minDot) { minDot = -dd; incAxisIndex = i; incSign = -1.0f; }
        }

        glm::vec3 incFaceVerts[4];
        incBox->getFaceVertices(incAxisIndex, incSign, incFaceVerts);

        // Tag the four incident corners with stable identities 0..3
        std::vector<ClipVert> polygon;
        polygon.reserve(8);
        for (int k = 0; k < 4; ++k) {
            polygon.push_back({ incFaceVerts[k], static_cast<uint32_t>(k) });
        }

        const int refT1 = (refAxisIndex + 1) % 3;
        const int refT2 = (refAxisIndex + 2) % 3;

        struct ClipPlane { glm::vec3 normal; float offset; };

        const glm::vec3 refCenter = refBox->center + refBox->axes[refAxisIndex] * (refBox->halfExtents[refAxisIndex] * refSign);

        ClipPlane sidePlanes[4];
        sidePlanes[0] = { refBox->axes[refT1],  glm::dot( refBox->axes[refT1], refCenter) + refBox->halfExtents[refT1]};
        sidePlanes[1] = {-refBox->axes[refT1], -glm::dot( refBox->axes[refT1], refCenter) + refBox->halfExtents[refT1]};
        sidePlanes[2] = { refBox->axes[refT2],  glm::dot( refBox->axes[refT2], refCenter) + refBox->halfExtents[refT2]};
        sidePlanes[3] = {-refBox->axes[refT2], -glm::dot( refBox->axes[refT2], refCenter) + refBox->halfExtents[refT2]};

        for (int p = 0; p < 4; ++p) {
            polygon = clipTagged(polygon, sidePlanes[p].normal, sidePlanes[p].offset, p);
            if (polygon.empty()) break;
        }

        const float refFaceOffset = glm::dot(refNormal, refCenter);
        const uint32_t refFaceId = static_cast<uint32_t>(refAxisIndex) | (refSign > 0.0f ? 0x10u : 0x00u);

        for (const ClipVert& v : polygon) {
            const float depth = refFaceOffset - glm::dot(refNormal, v.pos);
            if (depth >= -SPECULATIVE_MARGIN) {          // was: depth >= 0.0f
                CollisionInfo info;
                info.collided = true;
                info.point = v.pos;
                info.normal = sat.normal;
                info.penetration = depth;                 // signed: >0 overlap, <0 gap
                info.featureId = (refFaceId << 8) | (v.feature & 0xFFu);
                if (flipNormal) info.featureId |= 0x40000000u;
                manifold.push_back(info);
            }
        }
    }

    return manifold;
}

// ============================================================================
// Position Correction (legacy)
// ============================================================================

void Collision::resolvePenetration(RigidBody& a, RigidBody& b, const CollisionInfo& info) {
    const float invMassSum = a.inverseMass + b.inverseMass;
    if (invMassSum == 0.0f) return;

    const float ratioA = a.inverseMass / invMassSum;
    const float ratioB = b.inverseMass / invMassSum;

    a.position -= info.normal * info.penetration * ratioA;
    b.position += info.normal * info.penetration * ratioB;
}

// ============================================================================
// OBB separation distance (for continuous collision detection)
// ============================================================================

DistanceResult Collision::distanceOBB(const OBB& a, const OBB& b) {
    DistanceResult result;
    const glm::vec3 d = b.center - a.center;

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
        if (len < 1e-6f) continue;
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
            bestNormal = (centerDist < 0.0f) ? -axis : axis;
        }
    }

    result.distance = bestGap;
    result.overlapping = (bestGap <= 0.0f);
    result.normal = bestNormal;
    return result;
}
