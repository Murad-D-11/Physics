#pragma once
#include "aabb.h"
#include "CollisionInfo.h"

struct RigidBody; // forward the declaration

/**
 * Only answers questions about geometry and adjusts positions of two colliding objects
 */
class Collision {
    public:
        static CollisionInfo test(const AABB& a, const AABB& b); // tests whether two AABBs overlap, returns a CollisionInfo which describes status of the collision event
        static void resolvePenetration(RigidBody& a, RigidBody& b, const CollisionInfo& info); // pushes body A and B so they no longer overlap
};