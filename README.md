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

More recently the project has grown outward from a pure solver into a small
research platform: a modular scene library of physics experiments, an
interactive sandbox editor, live debugging overlays, a renderer-independent
observation API for machine learning, a deterministic dataset generator, and a
trajectory-prediction interface. None of that touches the solver's physics; it
all sits in layers around it.

For a map of how the pieces fit together, see docs/ARCHITECTURE.md. For
measured performance and validation numbers, see docs/BENCHMARKS.md.

CURRENT STATE
-------------
The engine currently supports:

  - Fixed-timestep simulation using semi-implicit (symplectic) Euler
    integration
  - AABB broad-phase collision detection (spatial-hash accelerated)
  - Impulse-based collision response, including multi-point OBB contact
    manifold generation (replacing earlier single-point contact handling)
  - Continuous collision detection (CCD) via conservative advancement, so
    fast bodies don't tunnel through thin geometry
  - Friction and restitution
  - Full rotational dynamics: quaternion-based orientation and proper
    per-body inertia tensors, with torque correctly applied from normal and
    friction impulses
  - Sphere collision/response, in addition to box (OBB) shapes
  - Resting contact and static friction handling, including stacking and
    sleeping bodies
  - Constraint types: springs, hinges, ropes, and pulleys, validated against
    analytical cases such as the Atwood machine
  - Slope / inclined-plane behavior with arbitrary plane normals
  - Physically based aerodynamics: quadratic drag from relative airflow, with
    orientation-dependent projected area and off-centre-of-pressure torque
  - Per-step research telemetry (energy, momentum, contacts, constraint error,
    sleep transitions) with zero cost when disabled

A number of debugging passes have gone into hardening the solver: fixing an
off-by-one error in floor contact generation, correcting mass vs.
inverse-mass usage in penetration resolution, and adding torque application
that had been missing from normal impulses, among others.

BUILDING
--------
Prerequisites:

  - A C++17 compiler (the project is developed with MinGW / g++ via the UCRT64
    toolchain on Windows)
  - CMake 3.10 or newer
  - GLFW and GLM are vendored under include/ and lib/; glfw3.dll ships in the
    repo root and is copied next to the executable automatically after a build

Configure and build the main application:

  cmake -S . -B build
  cmake --build build --target Physics

The build produces build/Physics.exe. glfw3.dll is copied beside it as a
post-build step, so the app can be launched directly.

There are three build targets:

  Physics           - the interactive OpenGL application (needs a display)
  PhysicsTests      - headless deterministic validation suite (no OpenGL)
  DatasetGenerator  - headless ML dataset generator (no OpenGL)

Build a specific target with, for example:

  cmake --build build --target PhysicsTests

RUNNING
-------
Launch the interactive app:

  build/Physics.exe

It opens paused in the interactive Sandbox scene. Press Space to start the
simulation. The current scene, recording state, prediction state, and loaded
model are printed to the console whenever they change, and shown as small
colored status bars in the top-left corner of the window.

CONTROLS
--------
Camera (mouse):

  Left-drag (empty space) . orbit the camera around the origin
  Scroll wheel ........... zoom in / out

Selection & sandbox editing (mouse):

  Left-click ............. select the body under the cursor
  Left-drag (on a body,
    while paused) ........ reposition it on its horizontal plane
  Right-drag ............. fling the selected body (drag direction + length set
                           the impulse)

Simulation (keyboard):

  Space .................. play / pause
  . (period) ............. single-frame step (advance exactly one fixed step)
  Backspace .............. restart the current scene (deterministic reset)
  1 - 9 .................. jump to scenes 1-9 (see the scene list below)
  N / B .................. next / previous scene (reaches all 13)

Sandbox object tools (keyboard):

  C ...................... spawn a cube
  V ...................... spawn a sphere
  X or Delete ............ delete the selected body

Materials (keyboard, applied to the selected body):

  F6 ..... Steel     F7 ..... Aluminum   F8 ..... Wood
  F9 ..... Rubber    F10 .... Ice

  Each preset sets realistic friction and restitution and derives the body's
  mass from the material density and its volume.

Environment (keyboard, affects the running simulation):

  G ...................... toggle aerodynamics
  Shift+G ................ toggle gravity
  [ / ] .................. decrease / increase air density
  Left / Right arrows .... adjust wind strength along X

Debugging overlays (keyboard):

  F1 ..... contact normals        F2 ..... angular-velocity axis
  F3 ..... center of mass         F4 ..... bounding volumes
  F5 ..... sleeping-body markers

Data & prediction (keyboard):

  R ...................... toggle recording (capture every timestep in memory)
  E ...................... export the recording to recording.csv
  P ...................... toggle path prediction for the selected body
                           (solid green = physics ground truth,
                            dotted purple = model prediction)

SCENE LIBRARY
-------------
Scenes are modular and deterministic: each one builds from fixed constants in
load(), so restarting reproduces it exactly. Every scene carries a name, a
short description, and the physical principle it demonstrates, plus adjustable
parameters. Number keys 1-9 load the first nine; N / B cycle through all
thirteen.

  Mechanics
    1  Empty Sandbox   - a bare ground plane with a marker cube
    2  Domino Spiral   - an Archimedean spiral of dominoes toppling in a cascade
    3  Newton's Cradle - suspended balls exchanging momentum on impact
    4  Atwood Machine  - two masses over an ideal pulley
    5  Inclined Plane  - a block on an adjustable ramp; slides or grips by friction

  Structures
    6  Rope Bridge       - a plank walkway suspended by rope segments
    7  Suspension Bridge - a plank deck hung from cables between two towers
    8  Cantilever Beam   - hinged box segments fixed at one end, drooping under gravity
    9  Hanging Chain     - a flexible chain settling into a catenary curve

  Dynamics (reach with N / B)
       Double Pendulum - two hinged arms; chaotic, sensitive to initial conditions
       Spring Pendulum - a bob on a damped spring (coupled swing + bob)
       Trebuchet       - a counterweight arm that flings a projectile

  Interactive (reach with N / B)
       Sandbox         - the editable playground you spawn into

All physics scenes live in src/app/Scenes.h; none are hardcoded in main().

MACHINE-LEARNING TOOLS
----------------------
The engine exposes a renderer-independent layer under src/ml/ so simulation
data can drive learning experiments without any OpenGL dependency.

  Environment (src/ml/Environment.h)
    A headless, gym-style wrapper around the solver: reset(), step(dt),
    getObservation(), and applyAction(id, impulse). An Observation is a flat,
    value-typed record of a body (id, mass, shape, material, position,
    velocity, angular velocity, orientation, sleeping) with no pointers into
    live state, so it is safe to copy, buffer, or serialise.

  PathPredictor (src/ml/PathPredictor.h)
    A swappable trajectory predictor with a fixed interface:
    loadModel(path) and predict(observation, futureFrames). With no model
    loaded it returns a deterministic physics rollout; an ONNX backend is
    isolated behind a compile guard, so the engine builds and runs without
    ONNX installed.

  DatasetGenerator (src/ml/DatasetGenerator.h + src/app/dataset_main.cpp)
    Generates supervised-learning datasets from random scenes and writes one
    flat CSV. Each row is a body's state at a frame plus that body's position
    a fixed number of frames in the future (default 30) as the label. Output
    is deterministic from the seed.

Generate a dataset:

  cmake --build build --target DatasetGenerator
  build/DatasetGenerator.exe dataset.csv --episodes 20 --frames 120 --seed 42

Options:

  output.csv .......... first positional argument (default dataset.csv)
  --episodes N ........ number of random scenes
  --frames N .......... simulated steps per episode
  --horizon N ......... label = position this many frames ahead (default 30)
  --seed N ............ master RNG seed (reproducible)
  --min-bodies N ...... minimum bodies per scene (default 3)
  --max-bodies N ...... maximum bodies per scene (default 20)

CSV columns: episode, frame, object_id, shape, material, mass, position (x,y,z),
velocity (x,y,z), angular velocity (x,y,z), orientation quaternion (w,x,y,z),
sleeping, and the future position label (x,y,z).

The interactive app can also record and export a run: press R to toggle
recording and E to write recording.csv.

PROJECT LAYOUT
--------------
  src/physics/    the solver and its supporting types (no rendering)
                    PhysicsSolver, RigidBody, OBB, Collision, Constraint,
                    Material, Telemetry
  src/renderer/   OpenGL rendering (Render, Camera, Cube, Sphere, Ground, glad)
  src/app/        the application layer: entry point, scene system, recorder
                    main.cpp, Scene / SceneManager / Scenes, SimulationRecorder,
                    physics_tests.cpp, dataset_main.cpp
  src/ml/         renderer-independent ML layer
                    Environment, PathPredictor, DatasetGenerator
  include/, lib/  vendored GLM, GLFW, and glad headers/libraries
  docs/           architecture and benchmark documentation

PHYSICS TESTS: src/app/physics_tests.cpp
-----------------------------------------
The engine's correctness is checked by a headless, deterministic validation
suite in src/app/physics_tests.cpp. Rather than eyeballing whether the
simulation "looks right," this file drives the real PhysicsSolver at a fixed
1/60 s timestep and checks the measured results against closed-form
analytical predictions, reporting measured value, expected value,
absolute/relative error, and a PASS / FAIL / WARN verdict for every check.

Build and run it:

  cmake --build build --target PhysicsTests
  build/PhysicsTests.exe

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
categories. See docs/BENCHMARKS.md for the latest measured results.

SUGGESTING A TEST CASE
-----------------------
If you have a scenario you think the physics validation suite should cover
(an edge case, a known-tricky configuration, a regression you've hit, or
just a physical situation you'd like verified) open an issue on this
repository describing the scenario and the expected behavior. Test case
ideas are welcome even if you don't want to write the C++ yourself. Look for
my contact info in my profile.
