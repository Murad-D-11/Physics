Demonstration & progress videos are available on my Instagram: https://www.instagram.com/conciliatory.murad/

PHYSICS
A rigid-body physics engine built from scratch in C++
=======================================================

OVERVIEW
--------
Physics is a self-directed, from-scratch rigid-body physics engine written in
C++, rendered with OpenGL and built on GLM and GLFW, with a Dear ImGui
interface. It's compiled with CMake (MinGW / UCRT64 on Windows). The project has
grown incrementally over many development sessions, starting from a single
rendered cube and progressively adding real simulation: gravity, fixed-timestep
integration, collision detection and response, friction, and full rotational
dynamics.

Throughout development the priority has been physical correctness over
shortcuts: artificial damping, snapping, and other stabilization hacks have
been deliberately removed in favor of implementations that are actually
correct, even when that's harder to get right. Where the solver genuinely
cannot reproduce an effect (for example gyroscopic precession, or an elastic
cantilever with no bending-stiffness primitive), that limitation is documented
honestly in the code rather than faked.

More recently the project has grown outward from a pure solver into a small
research platform: a modular scene library of physics experiments, an
interactive sandbox editor, an ImGui laboratory browser with live telemetry,
debugging overlays, a renderer-independent observation API for machine
learning, a deterministic dataset generator, a trajectory-prediction interface,
and several headless validation harnesses. None of that touches the solver's
physics; it all sits in layers around it.

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
  - Friction and restitution (with a resting-contact threshold so stacks don't
    jitter or bounce)
  - Full rotational dynamics: quaternion-based orientation and proper
    per-body inertia tensors, with torque correctly applied from normal and
    friction impulses
  - Sphere collision/response, in addition to box (OBB) shapes
  - Resting contact and static friction handling, including stacking and
    sleeping bodies
  - Constraint types: springs, hinges (with optional angle limits and a
    velocity motor), ropes (one-sided/inextensible), and pulleys, validated
    against analytical cases such as the Atwood machine
  - Slope / inclined-plane behavior with arbitrary plane normals
  - Physically based aerodynamics: quadratic drag from relative airflow, with
    orientation-dependent projected area and off-centre-of-pressure torque
  - Per-step research telemetry (energy, momentum, contacts, constraint error,
    sleep transitions) with zero cost when disabled

BUILDING
--------
Prerequisites:

  - A C++17 compiler (developed with MinGW / g++ via the UCRT64 toolchain on
    Windows)
  - CMake 3.10 or newer
  - GLFW, GLM, glad, and Dear ImGui are vendored under include/, lib/, and
    vendor/; glfw3.dll ships in the repo root and is copied next to the
    executable automatically after a build

Configure and build the main application:

  cmake -S . -B build
  cmake --build build --target Physics

The build produces build/Physics.exe. glfw3.dll is copied beside it as a
post-build step, so the app can be launched directly.

There are five build targets:

  Physics           - the interactive OpenGL + ImGui application (needs a display)
  PhysicsTests      - headless analytical validation suite (no OpenGL)
  SceneTests        - headless scene success-condition harness (no OpenGL)
  ValidationSuite   - headless physics validation laboratory (no OpenGL)
  DatasetGenerator  - headless ML dataset generator (no OpenGL)

Build a specific target with, for example:

  cmake --build build --target SceneTests

Building with no --target builds all five.

RUNNING
-------
Launch the interactive app:

  build/Physics.exe

It opens paused in the interactive Sandbox scene. Press Space to start the
simulation. The window shows the live 3D view with two ImGui panels overlaid:
a "Physics Laboratory" browser on the left and a "Telemetry" panel on the
right (details below). The current scene, recording state, prediction state,
and loaded model are also printed to the console whenever they change, and
shown as small colored status bars in the top-left corner.

INTERFACE (Dear ImGui)
----------------------
Two panels are drawn over the 3D view:

  Physics Laboratory (left)
    - Play / Pause / Restart / Step buttons for the simulation
    - A search box that filters the scene list by name (case-insensitive)
    - A scrollable list of every registered scene; click one to load it (the
      active scene is highlighted)
    - Metadata for the active scene: body count, constraint count, the physical
      principle it demonstrates, and a short description
    - Live parameter sliders for the active scene (e.g. explosion energy, chain
      jerk force, trebuchet counterweight, number of balls). Moving a slider
      rebuilds the scene with the new value.

  Telemetry (right)
    - FPS and run state (PAUSED / RUNNING)
    - Body count, awake vs. asleep counts, contact count, constraint count
    - Kinetic, potential, and total energy (joules), read from the solver's
      per-step telemetry

A large scene-title banner fades in briefly whenever a new scene loads.

VISUALIZING DATA / GRAPHS
-------------------------
The app itself does not draw charts or line graphs; its rendering is the 3D
view plus the ImGui numeric telemetry panel. To make and view graphs, export a
run to CSV and plot it in an external tool (pandas + matplotlib, a spreadsheet,
etc.). Two CSV paths exist:

  - Interactive recording: press R to toggle recording and E to export the
    current run to recording.csv (see "Data & prediction" below and the
    RECORDING & EXPORT section).
  - Offline datasets: the DatasetGenerator target (see MACHINE-LEARNING TOOLS).

Example (plot a body's height and every body's speed over time):

  import pandas as pd, matplotlib.pyplot as plt
  df = pd.read_csv("build/recording.csv")
  b0 = df[df.object_id == 0]
  plt.plot(b0.time, b0.pos_y); plt.xlabel("time (s)"); plt.ylabel("height (m)"); plt.show()
  df["speed"] = (df.vel_x**2 + df.vel_y**2 + df.vel_z**2) ** 0.5
  for oid, g in df.groupby("object_id"): plt.plot(g.time, g.speed, label=f"body {oid}")
  plt.legend(); plt.show()

CONTROLS
--------
Most actions are available both in the ImGui panels and via keyboard.

Camera (mouse):

  - Left-drag (empty space) . orbit the camera around the origin
  - Scroll wheel ........... zoom in / out

Selection & sandbox editing (mouse):

  - Left-click ............. select the body under the cursor
  - Left-drag (on a body,
    - while paused) ........ reposition it on its horizontal plane
  - Right-drag ............. fling the selected body (drag direction + length set
                           the impulse)

  (Clicks over an ImGui panel are consumed by the UI and do not pick/orbit.)

Simulation (keyboard):

  - Space .................. play / pause
  - . (period) ............. single-frame step (advance exactly one fixed step)
  - Backspace .............. restart the current scene (deterministic reset)
  - 1 - 9 .................. jump to scenes 1-9 (see the scene list below)
  - N / B .................. next / previous scene (cycles through all of them)

Sandbox object tools (keyboard):

  - C ...................... spawn a cube
  - V ...................... spawn a sphere
  - X or Delete ............ delete the selected body

Materials (keyboard, applied to the selected body):

  - F6 ..... Steel
  - F7 ..... Aluminum
  - F8 ..... Wood
  - F9 ..... Rubber
  - F10 .... Ice

  Each preset sets realistic friction and restitution and derives the body's
  mass from the material density and its volume.

Environment (keyboard, affects the running simulation):

  - G ...................... toggle aerodynamics
  - Shift+G ................ toggle gravity
  - [ / ] .................. decrease / increase air density
  - Left / Right arrows .... adjust wind strength along X

Debugging overlays (keyboard):

  - F1 ..... contact normals
  - F2 ..... angular-velocity axis
  - F3 ..... center of mass
  - F4 ..... bounding volumes
  - F5 ..... sleeping-body markers

Data & prediction (keyboard):

  - R ...................... toggle recording (capture every timestep in memory)
  - E ...................... export the recording to recording.csv
  - P ...................... toggle path prediction for the selected body
                           (solid green = physics ground truth,
                            dotted purple = model prediction)

SCENE LIBRARY
-------------
Scenes are modular and deterministic: each one builds from fixed constants in
load(), so restarting reproduces it exactly. Every scene carries a name, a
short description, and the physical principle it demonstrates, plus adjustable
parameters exposed as sliders in the Laboratory panel. Number keys 1-9 load the
first nine; N / B cycle through all of them; or click any scene in the panel.

  Mechanics
    Empty Sandbox    - a bare ground plane with a marker cube
    Domino Spiral    - an Archimedean spiral of dominoes toppling in a cascade
    Newton's Cradle  - bifilar-suspended balls exchanging momentum on impact,
                       tuned for a clean end-to-end transfer (0 -> 100 -> 0)
    Atwood Machine   - two masses over an ideal pulley, confined by guide shafts
    Inclined Plane   - a block on a ramp; slides or grips depending on friction

  Structures
    Rope Bridge        - a plank walkway suspended by rope segments
    Suspension Bridge  - a plank deck hung from cables between two towers
    Hanging Chain      - a flexible chain settling into a catenary curve
    Cable-Stayed Bridge- a deck held in tension by a fan of stay cables

  Dynamics
    Double Pendulum  - two balls joined by inextensible ropes; chaotic motion
    Spring Laboratory- a bob on a damped spring (coupled swing + bob), plus a
                       coupled-oscillator pair
    Trebuchet        - a counterweight arm on a fixed axle that flings a
                       projectile (counterweight load adjustable up to 120)
    Ballistics       - a fan of projectiles verifying the vacuum parabola and
                       45-degree max-range result
    Gyroscope        - a spinning flywheel on an offset support (see note below)

  Waves & many-body
    Hanging Chain Wave - a chain held taut on the ground; a sideways "jerk
                         force" flick launches a transverse wave to the centre
    Object Volume      - a dense pile of boxes and spheres dropped into a bin;
                         a many-body contact / direction-resolution stress test
    Explosion          - a cluster blown radially outward by an energy-controlled
                         blast (KE budget split across fragments)

  Interactive
    Sandbox          - the editable playground you spawn into

Honest limitations noted in the code: the Gyroscope spins and stays upright but
the sequential-impulse solver dissipates gyroscopic coupling, so it does not
reproduce quantitative precession. A Cantilever Beam scene was removed because
the solver has no bending-stiffness constraint, so an articulated beam always
collapses instead of bending elegantly. All physics scenes live in
src/app/Scenes.h; none are hardcoded in main().

RECORDING & EXPORT
------------------
Recording is on by default and captures a read-only snapshot of every body
after each physics step (it never touches the solver). Press R to toggle it and
E to write the current recording to recording.csv in the working directory. The
recording clears automatically on any structural change (scene switch, restart,
spawn, delete, or moving a body) so object ids stay stable within a run; export
before switching scenes if you want to keep the data.

recording.csv columns (one row per step per body):

  - step, time, object_id, shape,
  - pos_x, pos_y, pos_z,
  - vel_x, vel_y, vel_z,
  - ang_vel_x, ang_vel_y, ang_vel_z,
  - quat_w, quat_x, quat_y, quat_z

(shape: 0 = box, 1 = sphere.)

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
    ONNX installed. This is what the P key visualizes.

  TelemetryExport (src/ml/TelemetryExport.h)
    Observes the solver's per-step TelemetryFrame and appends long-format rows
    (per-body state plus system aggregates: kinetic/potential/mechanical
    energy, linear and angular momentum, contact count, per-constraint-type
    error, penetration, aero work). Richer than recording.csv and well suited
    to physics plots (energy conservation, momentum transfer, constraint error).

  DatasetGenerator (src/ml/DatasetGenerator.h + src/app/dataset_main.cpp)
    Generates supervised-learning datasets from random scenes and writes one
    flat CSV. Each row is a body's state at a frame plus that body's position a
    fixed number of frames in the future (default 30) as the label. Output is
    deterministic from the seed.

Generate a dataset:

  cmake --build build --target DatasetGenerator
  build/DatasetGenerator.exe dataset.csv --episodes 20 --frames 120 --seed 42

Options:

  output.csv .......... first positional argument (default dataset.csv)
  - --episodes N ........ number of random scenes
  - --frames N .......... simulated steps per episode
  - --horizon N ......... label = position this many frames ahead (default 30)
  - --seed N ............ master RNG seed (reproducible)
  - --min-bodies N ...... minimum bodies per scene (default 3)
  - --max-bodies N ...... maximum bodies per scene (default 20)

CSV columns: episode, frame, object_id, shape, material, mass, position (x,y,z),
velocity (x,y,z), angular velocity (x,y,z), orientation quaternion (w,x,y,z),
sleeping, and the future position label (x,y,z).

PROJECT LAYOUT
--------------
  src/physics/    the solver and its supporting types (no rendering)
                    PhysicsSolver, RigidBody, OBB, Collision, Constraint,
                    Material, Telemetry
  src/renderer/   OpenGL rendering (Render, Camera, Cube, Sphere, Ground, glad)
  src/app/        the application layer: entry point, scene system, harnesses
                    main.cpp, Scene / SceneManager / Scenes, SimulationRecorder,
                    physics_tests.cpp, scene_tests.cpp, validation_suite.cpp,
                    ValidationLab.h, dataset_main.cpp
  src/ml/         renderer-independent ML layer
                    Environment, PathPredictor, DatasetGenerator, TelemetryExport
  vendor/imgui/   vendored Dear ImGui (GUI; linked only by the app)
  include/, lib/  vendored GLM, GLFW, and glad headers/libraries
  docs/           architecture and benchmark documentation

TESTING & VALIDATION
--------------------
The engine's correctness is checked by three headless, deterministic harnesses
(no OpenGL required), each a separate build target.

  PhysicsTests (src/app/physics_tests.cpp)
    Drives the real PhysicsSolver at a fixed 1/60 s timestep and checks measured
    results against closed-form analytical predictions, reporting measured
    value, expected value, absolute/relative error, and a PASS / FAIL / WARN
    verdict per check. Organized into 17 sections run in order:

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

    Because the engine integrates with semi-implicit Euler, tests compare
    against both the exact discrete-scheme prediction and the continuous
    analytical solution: matching the discrete scheme confirms the integrator
    is correct, while the residual gap to the continuous solution is the
    expected O(dt) discretization error, not a bug. Conservation checks
    (momentum, angular momentum) are exact properties of the impulse
    formulation, so a failure there points to a real bug. The suite runs well
    over 300 assertions; two known-limitation cases (ideal-pulley tension in
    the Atwood validation) are documented rather than hidden.

    Build and run:
      cmake --build build --target PhysicsTests
      build/PhysicsTests.exe

  SceneTests (src/app/scene_tests.cpp)
    Loads every registered scene through the real SceneManager + solver and
    checks two layers of invariants: generic ones (state stays finite, no
    explosion, no runaway speed, no floor sink) for every scene, plus
    scene-specific success conditions that verify the intended behaviour (e.g.
    Newton's Cradle transfers momentum cleanly through the line, the chain wave
    reaches the centre bead, the Object Volume pile settles inside its bin, the
    Explosion field expands radially, the trebuchet launches its projectile).
    Reproduces scene crashes deterministically.

    Build and run:
      cmake --build build --target SceneTests
      build/SceneTests.exe

  ValidationSuite (src/app/validation_suite.cpp + src/app/ValidationLab.h)
    A physics validation laboratory that runs analytic experiments through
    layered checks (numerical integrity, constraint validation, physical-law
    comparison with %% error, and randomized stress testing). Deterministic;
    the scientific regression benchmark for the engine.

    Build and run:
      cmake --build build --target ValidationSuite
      build/ValidationSuite.exe

See docs/BENCHMARKS.md for the latest measured results.

SUGGESTING A TEST CASE
-----------------------
If you have a scenario you think the validation suite should cover (an edge
case, a known-tricky configuration, a regression you've hit, or just a physical
situation you'd like verified) open an issue on this repository describing the
scenario and the expected behavior. Test case ideas are welcome even if you
don't want to write the C++ yourself. Look for my contact info in my profile.
