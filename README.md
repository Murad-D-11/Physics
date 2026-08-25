Demonstration & progress videos are available on my Instagram: https://www.instagram.com/conciliatory.murad/

PHYSICS
A rigid-body physics engine built from scratch in C++
=======================================================

OVERVIEW
--------
Physics is a self-directed, from-scratch rigid-body physics engine written in
C++, rendered with OpenGL and built on GLM and GLFW. It's compiled with CMake
(MinGW on Windows). The project has grown incrementally over many development
sessions, starting from a single rendered cube and progressively adding real
simulation: gravity, fixed-timestep integration, collision detection and
response, friction, and full rotational dynamics.

Throughout development the priority has been physical correctness over
shortcuts: artificial damping, snapping, and other stabilization hacks have
been deliberately removed in favor of implementations that are actually
correct, even when that's harder to get right.

CURRENT STATE
-------------
The engine currently supports:

  - Fixed-timestep simulation using semi-implicit (symplectic) Euler
    integration
  - AABB broad-phase collision detection
  - Impulse-based collision response, including multi-point OBB contact
    manifold generation (replacing earlier single-point contact handling)
  - Friction and restitution
  - Full rotational dynamics: quaternion-based orientation and proper
    per-body inertia tensors, with torque correctly applied from normal and
    friction impulses
  - Sphere collision/response, in addition to box (OBB) shapes
  - Resting contact and static friction handling, including stacking and
    sleeping bodies
  - Constraint types: springs, hinges, ropes, and pulleys, validated against
    analytical cases such as the Atwood machine
  - Slope / inclined-plane behavior

A number of debugging passes have gone into hardening the solver: fixing an
off-by-one error in floor contact generation, correcting mass vs.
inverse-mass usage in penetration resolution, and adding torque application
that had been missing from normal impulses, among others.

PHYSICS TESTS: src/app/physics_tests.cpp
-----------------------------------------
The engine's correctness is checked by a headless, deterministic validation
suite in src/app/physics_tests.cpp. Rather than eyeballing whether the
simulation "looks right," this file drives the real PhysicsSolver at a fixed
1/60 s timestep and checks the measured results against closed-form
analytical predictions, reporting measured value, expected value,
absolute/relative error, and a PASS / FAIL / WARN verdict for every check.

The suite is organized into 17 sections, run in order from main():

   1. Linear mechanics       - gravity/integration, momentum, elastic and
                                inelastic collisions
   2. Rotational mechanics   - inertia tensor, angular momentum, torque,
                                gyroscopic behavior
   3. Contact mechanics      - friction, stacking, rolling (no-slip),
                                resting contact, manifold correctness
   4. Energy                 - restitution loss, free-fall energy
                                conservation, no spurious energy creation
   5. Numerical stability    - tall towers, domino chains, high-speed
                                impacts, piles of bodies
   6. Resting contact & static friction (task-specified test cases)
   7. Static contact networks - stacking, convergence, sleeping, solve order
   8. Mechanical energy & momentum audit
   9. Contact manifold quality audit
  10. Rotational contact mechanics audit
  11. Adversarial numerical robustness
  12. Performance scaling benchmark
  13. Sphere physics validation
  14. Slope / inclined-plane validation
  15. Constraint validation (springs + hinges)
  16. Rope & pulley validation
  17. Atwood machine validation

A small `Suite` harness (near / atMost / isTrue / note) drives all of this
and prints a final summary of how many checks passed, failed, or landed in
"known limitation" territory. Because the engine integrates with
semi-implicit Euler, tests compare against both the exact discrete-scheme
prediction and the continuous analytical solution -- matching the discrete
scheme confirms the integrator itself is implemented correctly, while the
residual gap to the continuous solution is the expected O(dt) discretization
error rather than a bug. Conservation checks (momentum, angular momentum),
by contrast, are exact properties of the impulse formulation, so any failure
there points to a real implementation error rather than integration error.
Altogether the suite runs well over 200 individual assertions across these
categories.

SUGGESTING A TEST CASE
-----------------------
If you have a scenario you think the physics validation suite should cover
(an edge case, a known-tricky configuration, a regression you've hit, or
just a physical situation you'd like verified) open an issue on this
repository describing the scenario and the expected behavior. Test case
ideas are welcome even if you don't want to write the C++ yourself. Look for
my contact info in my profile.
