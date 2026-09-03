#pragma once
// ===========================================================================
// Material — named physical-material presets for rigid bodies.
//
// Each preset carries a density (kg/m^3), a friction coefficient, and a
// restitution (bounciness). applyMaterial() writes friction/restitution onto a
// body and derives its mass from density * volume (using the body's current
// shape/scale), then recomputes the inertia tensor. This keeps mass physically
// consistent with size + material rather than an arbitrary number.
//
// Header-only and dependency-free (just RigidBody + glm) so both the app and
// the headless ML/dataset tools can use it.
// ===========================================================================

#include <glm/glm.hpp>
#include "rigidbody.h"

enum class MaterialType : int {
    Steel    = 0,
    Aluminum = 1,
    Wood     = 2,
    Rubber   = 3,
    Ice      = 4,
    Count    = 5
};

struct Material {
    const char* name;
    float density;      // kg/m^3
    float friction;     // Coulomb coefficient (dimensionless)
    float restitution;  // 0 = inelastic, 1 = perfectly elastic
};

// Realistic-ish presets. Values are representative engineering figures, chosen
// so relative behaviour is correct (steel heavy + low bounce, rubber light +
// bouncy, ice low friction, wood middling).
inline const Material& materialPreset(MaterialType t) {
    static const Material presets[] = {
        { "Steel",    7850.0f, 0.60f, 0.25f },
        { "Aluminum", 2700.0f, 0.55f, 0.30f },
        { "Wood",      700.0f, 0.50f, 0.35f },
        { "Rubber",   1100.0f, 0.90f, 0.80f },
        { "Ice",       917.0f, 0.05f, 0.15f },
    };
    int i = static_cast<int>(t);
    if (i < 0 || i >= static_cast<int>(MaterialType::Count)) i = 0;
    return presets[i];
}

inline const char* materialName(MaterialType t) { return materialPreset(t).name; }

// Approximate collision volume (m^3) from the body's shape and size.
inline float bodyVolume(const RigidBody& b) {
    if (b.shape == ShapeType::Sphere) {
        const float r = b.radius;
        return (4.0f / 3.0f) * 3.14159265358979f * r * r * r;
    }
    // Box: full extents are `scale`.
    return b.scale.x * b.scale.y * b.scale.z;
}

// Apply a material to a body: set friction/restitution, derive mass from
// density * volume, and refresh the inertia tensor. Static bodies (inverseMass
// already 0) keep their static status.
inline void applyMaterial(RigidBody& b, MaterialType t) {
    const Material& m = materialPreset(t);
    b.friction = m.friction;
    b.restitution = m.restitution;

    const bool wasStatic = (b.inverseMass == 0.0f);
    if (!wasStatic) {
        const float volume = bodyVolume(b);
        const float mass = (volume > 0.0f) ? m.density * volume : b.mass;
        b.mass = mass;
        b.inverseMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    }
    b.materialType = static_cast<int>(t);
    b.updateInertiaTensor();
}
