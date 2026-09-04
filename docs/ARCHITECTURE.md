# Architecture

This document maps how the Physics engine is put together: its layers, the
dependency graph between modules, what each module is responsible for, and the
per-step data-flow pipeline. It is meant as a companion to the
[README](../README.md).

The guiding rule is a strict dependency direction: **physics knows nothing
about rendering, and the ML layer knows nothing about OpenGL.** Everything
points inward toward the solver.

---

## Layers

```
                        +-----------------------------+
                        |         Application         |   src/app/
                        |  main.cpp  (window + loop)  |
                        |  Scene / SceneManager       |
                        |  Scenes  SimulationRecorder |
                        |  physics_tests  dataset_main|
                        +--------------+--------------+
                                       |
             +-------------------------+-------------------------+
             |                         |                         |
             v                         v                         v
   +-------------------+     +-------------------+     +-------------------+
   |     Renderer      |     |   ML / Research   |     |      Physics      |
   |   src/renderer/   |     |     src/ml/       |     |   src/physics/    |
   |                   |     |                   |     |                   |
   | Render  Camera    |     | Environment       |     | PhysicsSolver     |
   | Cube  Sphere      |     | PathPredictor     |     | RigidBody  OBB    |
   | Ground  glad      |     | DatasetGenerator  |     | Collision         |
   |                   |     |                   |     | Constraint        |
   |                   |     |                   |     | Material Telemetry|
   +---------+---------+     +---------+---------+     +---------+---------+
             |                         |                         ^
             | (draws state)           | (owns a solver)         |
             +-------------------------+-------------------------+
                                       |
                                       v
                              (depends on Physics)
```

- **Physics** is the core and depends on nothing else in the project (only GLM).
- **Renderer** depends only on GLM/glad/GLFW; it draws whatever state it is
  handed and never touches the solver's internals.
- **ML / Research** depends only on Physics. It has **no** rendering
  dependency, so it compiles and runs headless.
- **Application** is the only layer allowed to wire the other three together.
  It also owns the on-screen GUI: **Dear ImGui** (vendored under `vendor/imgui`,
  GLFW + OpenGL3 backends) is linked into the `Physics` app target only. The
  headless targets (`PhysicsTests`, `SceneTests`, `ValidationSuite`,
  `DatasetGenerator`) do **not** link ImGui or any GL, so they stay pure.

---

## Module dependency graph

Edges are real `#include` relationships between the project's own headers
(third-party and standard-library includes omitted). Read `A --> B` as
"A includes / depends on B".

```
main.cpp
  ├──> renderer/render.h ──┬──> renderer/cube.h
  │                        ├──> renderer/sphere.h
  │                        ├──> renderer/ground.h
  │                        └──> renderer/camera.h
  ├──> renderer/camera.h
  ├──> renderer/cube.h
  ├──> renderer/sphere.h
  ├──> renderer/ground.h
  ├──> physics/rigidbody.h
  ├──> physics/physicssolver.h ─┬──> physics/obb.h ──> physics/rigidbody.h
  │                             ├──> physics/rigidbody.h
  │                             ├──> physics/collisioninfo.h
  │                             ├──> physics/Constraint.h
  │                             └──> physics/Telemetry.h
  ├──> physics/Material.h ──> physics/rigidbody.h
  ├──> app/Scene.h ─────────┬──> physics/rigidbody.h
  │                         ├──> physics/physicssolver.h
  │                         └──> physics/Constraint.h
  ├──> app/SceneManager.h ──┬──> app/Scene.h
  │                         ├──> physics/physicssolver.h
  │                         └──> physics/rigidbody.h
  ├──> app/Scenes.h ────────────> app/Scene.h
  ├──> app/SimulationRecorder.h ──> physics/rigidbody.h
  └──> ml/PathPredictor.h ──┬──> physics/rigidbody.h
                            └──> ml/Environment.h ──┬──> physics/rigidbody.h
                                                    └──> physics/physicssolver.h

dataset_main.cpp
  └──> ml/DatasetGenerator.h ─┬──> physics/rigidbody.h
                              ├──> physics/Material.h ──> physics/rigidbody.h
                              └──> ml/Environment.h ──> physics/physicssolver.h

physics_tests.cpp
  ├──> physics/rigidbody.h
  └──> physics/physicssolver.h

Implementation units (.cpp) and the headers they compile against:
  physics/PhysicsSolver.cpp ──> physicssolver.h, obb.h, collision.h
  physics/Collision.cpp     ──> collision.h ──> obb.h, CollisionInfo.h ; rigidbody.h
  physics/OBB.cpp           ──> obb.h, rigidbody.h
  renderer/Render.cpp       ──> render.h (+ cube/sphere/ground/camera)
  renderer/{Cube,Sphere,Ground,Camera}.cpp ──> their own headers
```

A few things worth noting from the graph:

- **`physics/rigidbody.h` is the hub.** Almost every module includes it because
  it is the shared data type that flows through the whole system. It has no
  project dependencies of its own.
- **The ML layer's only door into physics is `Environment.h`**, which owns a
  `PhysicsSolver` and a `std::vector<RigidBody>`. `PathPredictor` and
  `DatasetGenerator` go through it, so nothing in `src/ml/` links OpenGL.
- **No cycles.** The include graph is a DAG; the arrows only ever point from
  application → (renderer | ml) → physics.

---

## Module responsibilities

### `src/physics/` — the solver (no rendering)

| Module | Responsibility |
| --- | --- |
| `RigidBody.h` | The core body type: shape, mass/inverse-mass, position, velocity, orientation (quaternion), angular velocity, inertia tensor, restitution/friction, sleeping flags, aerodynamic properties, and a passive `materialType` annotation. |
| `PhysicsSolver` | The engine loop: `step()` integrates (semi-implicit Euler), runs CCD, generates contacts, and solves velocities/positions with sequential impulses. Owns planes, springs, hinges, ropes, pulleys, sleeping, aerodynamics, and optional telemetry. |
| `OBB` | Oriented bounding box built from a `RigidBody`; the geometric primitive for box collision. |
| `Collision` | SAT-based OBB collision detection and multi-point contact manifold generation. |
| `CollisionInfo.h` | Plain value type describing a contact (point, normal, penetration). |
| `Constraint.h` | Spring, hinge, rope, and pulley constraint descriptions/state. |
| `Material.h` | Named material presets (Steel/Aluminum/Wood/Rubber/Ice) with density/friction/restitution and `applyMaterial()` (mass from density × volume). |
| `Telemetry.h` | A flat, value-typed per-step snapshot (energy, momentum, contacts, constraint error, sleep transitions) for research/observation. |

### `src/renderer/` — OpenGL rendering

| Module | Responsibility |
| --- | --- |
| `Render` | All draw calls: bodies, static collision planes (ramps/tables drawn as thin slabs), ground grid, lines, dotted/solid trajectory paths, status bars, pause icon. Holds the shaders and GL buffers. |
| `Camera` | Orbit camera: view/projection matrices, mouse-drag orbit, scroll zoom. |
| `Cube`, `Sphere`, `Ground` | Mesh geometry for the three drawable primitives. |
| `glad.c` | OpenGL function loader. |

### `src/app/` — application and tooling

| Module | Responsibility |
| --- | --- |
| `main.cpp` | Window/GL setup, the fixed-timestep loop, input callbacks, overlays, HUD, the Dear ImGui **laboratory browser** (searchable scene list with per-scene body/constraint metadata + principle, play/pause/step, parameter sliders, and a live telemetry panel) plus the fading scene-title banner, and the wiring of renderer + scenes + recorder + predictor. |
| `Scene.h` | Base scene: `load()/update()/reset()`, owned bodies + constraints, name/description/principle metadata, adjustable parameters, and reusable **apparatus builders** (`sceneAddStaticBox`, `sceneAddGuideShaft`, `sceneAddRampSlab`) for assembling scenes from real physical parts. `HingeDesc` exposes revolute angle-limits + motor. |
| `SceneManager.h` | Registers scenes, switches between them, and re-points the solver's constraint lists at the active scene's stable storage. |
| `Scenes.h` | The 13 concrete experiments (Mechanics / Structures / Dynamics / Sandbox). |
| `SimulationRecorder.h` | Read-only per-step capture of body state into memory and CSV export. |
| `physics_tests.cpp` | The headless deterministic validation suite (17 sections). |
| `dataset_main.cpp` | CLI entry point for the dataset generator. |

### `src/ml/` — renderer-independent research layer

| Module | Responsibility |
| --- | --- |
| `Environment.h` | Headless gym-style wrapper (`reset/step/getObservation/applyAction`) owning a solver + bodies. Defines the flat `Observation` type. |
| `PathPredictor.h` | Swappable trajectory predictor: physics rollout by default, ONNX behind a compile guard. |
| `DatasetGenerator.h` | Random-scene generation + supervised CSV export with future-position labels. |

---

## Per-step data-flow pipeline

Inside `PhysicsSolver::step()` the frame flows through a fixed pipeline. This
is the "geometry → collision → contacts → constraints → forces → integration →
sleeping → telemetry" chain the engine is built around:

```
  bodies (state at t)
      |
      v
  [ forces ]      apply gravity + aerodynamic drag (relative airflow, drag,
      |           off-COM torque)
      v
  [ integrate ]   semi-implicit Euler: v += a·dt ; x += v·dt ; advance orientation
      |
      v
  [ CCD ]         conservative advancement finds earliest time-of-impact so fast
      |           bodies don't tunnel; substep up to the impact
      v
  [ broad phase ] spatial-hash AABB pairs (prunes the O(n²) pair set)
      |
      v
  [ narrow phase] SAT on OBBs / analytic sphere tests -> contact manifolds
      |           (+ floor and arbitrary static-plane contacts)
      v
  [ solve vel ]   sequential-impulse velocity solve: normal + friction, then
      |           spring/hinge/rope/pulley constraints (warm-started)
      v
  [ solve pos ]   split-impulse position correction removes penetration without
      |           injecting energy
      v
  [ sleeping ]    islands below the rest threshold are put to sleep; contact/
      |           impulse wakes them
      v
  [ telemetry ]   (optional) fill one TelemetryFrame: energy, momentum, contacts,
      |           constraint error, sleep transitions
      v
  bodies (state at t + dt)
```

The application loop calls `step()` at a fixed `dt = 1/60 s` inside an
accumulator, so simulation rate is decoupled from render rate. The recorder and
the ML `Environment` both read body state **after** `step()` returns — they
never participate in the solve, which is what keeps recording and observation
from affecting the physics.

---

## Source size

Approximate line counts by module (implementation + headers), as a rough guide
to where the weight sits.

| Area | Files | Lines |
| --- | --- | ---: |
| Physics solver + types | `PhysicsSolver.{h,cpp}`, `Collision.{h,cpp}`, `OBB.{h,cpp}`, `RigidBody.h`, `Constraint.h`, `CollisionInfo.h`, `Material.h`, `Telemetry.h` | ~2,950 |
| Validation suite | `physics_tests.cpp` | ~3,330 |
| Application | `main.cpp`, `Scene.h`, `SceneManager.h`, `Scenes.h`, `SimulationRecorder.h` | ~2,460 |
| ML / research | `Environment.h`, `PathPredictor.h`, `DatasetGenerator.h`, `dataset_main.cpp` | ~530 |
| Renderer (excl. glad) | `Render.{h,cpp}`, `Camera.{h,cpp}`, `Cube.{h,cpp}`, `Sphere.{h,cpp}`, `Ground.{h,cpp}` | ~770 |
| Vendored loader | `glad.c` | ~1,780 |

The solver and its validation suite together are the bulk of the project — a
deliberate reflection of the "correctness first" priority: there is nearly as
much test code as engine code.
