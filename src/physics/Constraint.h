#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class RigidBody;

// ============================================================================
// Spring Constraint (Hooke's Law + damping)
//
// Applies F = -k * (|x| - restLength) * dir - c * v_rel_along_spring
// at the actual attachment points on each body, generating both translational
// force and torque (since attachment points are generally off-centre).
// ============================================================================

struct SpringConstraint {
    // --- Parameters (set by user) ---
    RigidBody* bodyA = nullptr;       // first connected body (nullptr = world anchor)
    RigidBody* bodyB = nullptr;       // second connected body (nullptr = world anchor)
    glm::vec3 localAnchorA = glm::vec3(0.0f); // attachment in A's local space
    glm::vec3 localAnchorB = glm::vec3(0.0f); // attachment in B's local space
    float restLength  = 1.0f;         // natural length (no force)
    float stiffness   = 50.0f;        // spring constant k (N/m)
    float damping     = 2.0f;         // damping coefficient c (N*s/m)

    // --- Runtime state (computed each step, AI-readable) ---
    float currentLength    = 0.0f;    // actual distance between world anchors
    float extension        = 0.0f;    // currentLength - restLength (>0 stretched)
    float forceMagnitude   = 0.0f;    // |F| applied this step
    glm::vec3 worldAnchorA = glm::vec3(0.0f);
    glm::vec3 worldAnchorB = glm::vec3(0.0f);
    glm::vec3 forceOnA     = glm::vec3(0.0f); // force vector applied to body A
    glm::vec3 forceOnB     = glm::vec3(0.0f); // force vector applied to body B
};

// ============================================================================
// Hinge Constraint (revolute joint)
//
// Enforces:
//   - Coincident anchor positions (3 translational DOF removed)
//   - Shared rotation axis (2 angular DOF removed)
//   - Leaves 1 DOF: rotation about the hinge axis
//
// Solved via sequential impulses alongside contact constraints.
// ============================================================================

struct HingeConstraint {
    // --- Parameters (set by user) ---
    RigidBody* bodyA = nullptr;       // first body (nullptr = world/static anchor)
    RigidBody* bodyB = nullptr;       // second body (nullptr = world/static anchor)
    glm::vec3 localAnchorA = glm::vec3(0.0f); // pivot in A's local space
    glm::vec3 localAnchorB = glm::vec3(0.0f); // pivot in B's local space
    glm::vec3 localAxisA   = glm::vec3(0.0f, 1.0f, 0.0f); // hinge axis in A's local space
    glm::vec3 localAxisB   = glm::vec3(0.0f, 1.0f, 0.0f); // hinge axis in B's local space

    // Optional angular limits (radians). If min >= max, limits are disabled.
    float angleLimitMin = 0.0f;
    float angleLimitMax = 0.0f; // min >= max => no limits

    // Optional motor: if maxMotorTorque > 0, applies torque toward targetAngularVelocity.
    float targetAngularVelocity = 0.0f;
    float maxMotorTorque        = 0.0f; // 0 = motor disabled

    // --- Runtime state (computed each step, AI-readable) ---
    glm::vec3 worldAnchorA  = glm::vec3(0.0f);
    glm::vec3 worldAnchorB  = glm::vec3(0.0f);
    glm::vec3 worldAxisA    = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 worldAxisB    = glm::vec3(0.0f, 1.0f, 0.0f);
    float currentAngle      = 0.0f;    // relative angle about hinge axis (radians)
    float currentAngularVel = 0.0f;    // relative angular velocity about hinge axis
    glm::vec3 positionError = glm::vec3(0.0f); // world-space anchor separation
    float angularError      = 0.0f;    // angular misalignment (axis error magnitude)

    // Accumulated impulses for warm-starting (3 positional + 2 angular + 1 limit + 1 motor)
    glm::vec3 accumulatedLinearImpulse  = glm::vec3(0.0f);
    glm::vec2 accumulatedAngularImpulse = glm::vec2(0.0f); // two axes perpendicular to hinge
    float accumulatedLimitImpulse       = 0.0f;
    float accumulatedMotorImpulse       = 0.0f;
};

// ============================================================================
// Rope Constraint (one-sided distance constraint)
//
// Enforces: distance(anchorA, anchorB) <= maxLength
//
// When slack (distance < maxLength): exerts ZERO force (compression-free).
// When taut (distance >= maxLength): generates tension opposing further
// separation, like a rigid rod but only in tension.
//
// NOT a spring — does not oscillate or pull back when slack.
// ============================================================================

struct RopeConstraint {
    // --- Parameters (set by user) ---
    RigidBody* bodyA = nullptr;       // first body (nullptr = world anchor)
    RigidBody* bodyB = nullptr;       // second body (nullptr = world anchor)
    glm::vec3 localAnchorA = glm::vec3(0.0f); // attachment in A's local space
    glm::vec3 localAnchorB = glm::vec3(0.0f); // attachment in B's local space
    float maxLength = 2.0f;           // maximum rope length (constraint activates when exceeded)

    // --- Runtime state (computed each step, AI-readable) ---
    float currentLength    = 0.0f;    // actual distance between world anchors
    bool taut              = false;   // true when constraint is active (distance >= maxLength)
    float tension          = 0.0f;    // constraint impulse magnitude (0 when slack)
    glm::vec3 worldAnchorA = glm::vec3(0.0f);
    glm::vec3 worldAnchorB = glm::vec3(0.0f);
    glm::vec3 direction    = glm::vec3(0.0f); // unit vector A->B when taut

    // Accumulated impulse for warm-starting
    float accumulatedImpulse = 0.0f;
};

// ============================================================================
// Pulley Constraint (rope over a fixed pulley)
//
// Enforces: lengthA + lengthB <= totalRopeLength
//   where lengthA = distance(pulleyPos, anchorA)
//         lengthB = distance(pulleyPos, anchorB)
//
// Models an ideal frictionless pulley: when one side descends, the other
// ascends. The constraint force is tension transmitted through the rope.
// ============================================================================

struct PulleyConstraint {
    // --- Parameters (set by user) ---
    RigidBody* bodyA = nullptr;       // body on one side of the pulley
    RigidBody* bodyB = nullptr;       // body on the other side
    glm::vec3 localAnchorA = glm::vec3(0.0f);
    glm::vec3 localAnchorB = glm::vec3(0.0f);
    glm::vec3 pulleyPos    = glm::vec3(0.0f, 5.0f, 0.0f); // fixed pulley position (world)
    float totalRopeLength  = 4.0f;    // total rope length over the pulley
    float pulleyRadius     = 0.25f;   // RENDER ONLY: wheel radius for tangent-point drawing.
                                      // Does NOT affect the ideal-rope physics.

    // --- Runtime state (computed each step, AI-readable) ---
    float lengthA          = 0.0f;    // distance from pulley to anchor A
    float lengthB          = 0.0f;    // distance from pulley to anchor B
    float currentTotal     = 0.0f;    // lengthA + lengthB
    bool taut              = false;
    float tension          = 0.0f;    // constraint impulse this step
    glm::vec3 worldAnchorA = glm::vec3(0.0f);
    glm::vec3 worldAnchorB = glm::vec3(0.0f);
    glm::vec3 dirA         = glm::vec3(0.0f); // unit vec pulley->A
    glm::vec3 dirB         = glm::vec3(0.0f); // unit vec pulley->B

    // Accumulated impulse for warm-starting (shared scalar tension impulse).
    float accumulatedImpulse = 0.0f;
};
