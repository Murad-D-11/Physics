# Murad Dashdamirov — Portfolio

dashdamirov.murad11@gmail.com | [LinkedIn](https://www.linkedin.com/in/murad-dashdamirov-90461934a) | [GitHub](https://github.com/Murad-D-11)

---

## 3D Rigid-Body Physics Engine & Simulation Lab

*C++17 · OpenGL · GLM · Dear ImGui · CMake · Python*
[GitHub repository](https://github.com/Murad-D-11) &nbsp;·&nbsp; [Technical docs (README / Architecture / Benchmarks)](https://github.com/Murad-D-11)

A 3D rigid-body physics engine written from scratch in C++ — no physics
libraries. The solver, collision detection, constraints, and continuous
collision detection are all hand-implemented, with correctness prioritized over
shortcuts (artificial damping and snapping were deliberately removed in favor of
implementations that are actually correct). Around the solver sits an
interactive OpenGL + Dear ImGui laboratory, a renderer-independent machine-
learning data layer, and three headless validation harnesses.

<!-- ===================================================================== -->
<!-- IMAGES: replace the placeholders below with real screenshots/GIFs.    -->
<!-- Suggested captures (each ~half the row, side by side):                -->
<!--  1. The ImGui "Physics Laboratory" browser + a scene mid-simulation   -->
<!--     (e.g. Newton's Cradle or the Explosion), with the Telemetry panel  -->
<!--     visible showing energy/momentum.                                   -->
<!--  2. A second scene that reads as visually distinct (Domino Spiral,     -->
<!--     Object Volume pile, or Hanging Chain Wave) + a debug overlay on    -->
<!--     (F1 contact normals or F4 bounding volumes).                       -->
<!-- A short GIF of a cascade or the explosion expanding is very strong     -->
<!-- here since the project is inherently visual.                           -->
<!-- ===================================================================== -->

| ![Physics Laboratory GUI with a scene running and live telemetry](docs/img/portfolio_lab.png) | ![A second scene with a debug overlay enabled](docs/img/portfolio_scene.png) |
| :---: | :---: |
| *Dear ImGui laboratory: searchable scene list, live parameter sliders, and an energy/momentum telemetry panel.* | *A scene mid-simulation with a physics-inspector overlay (contact normals / bounding volumes).* |

**Engine & architecture**

- Implemented the full simulation loop from scratch: semi-implicit (symplectic)
  Euler integration, a spatial-hash AABB broad phase, SAT-based oriented-
  bounding-box narrow phase with multi-point contact manifolds, and a warm-
  started sequential-impulse velocity solve followed by a split-impulse position
  correction that removes penetration without injecting energy.
- Modeled full rotational dynamics with quaternion orientation and per-body
  inertia tensors, applying torque from both normal and friction impulses, plus
  four constraint types — springs, hinges (with angle limits and a velocity
  motor), inextensible ropes, and pulleys.
- Enforced a strict inward dependency direction — physics knows nothing about
  rendering, and the ML layer knows nothing about OpenGL — so the same solver
  compiles into the interactive app and into four headless targets unchanged.

**Performance (measured, `dt = 1/60 s`, `-O2`)**

- Kept per-body step cost nearly flat — a **1.33× increase across a 50× growth
  in body count** (0.0047 → 0.0062 ms/body/step from 10 to 500 bodies) — by
  pruning the O(n²) pair set with a spatial hash; 500 interacting bodies solve
  in **~3.1 ms/step**, well under the 16.6 ms real-time budget at 60 Hz.
- Added conservative-advancement continuous collision detection that finds the
  earliest time-of-impact so fast movers substep to contact instead of
  tunnelling — **no tunnelling verified up to 200 m/s impacts** — at a cost of
  **< 0.5% of a step** even in a busy 500-body scene.
- Ran a 150-body domino cascade for 2,400 steps (40 s simulated) with **800 peak
  simultaneous contacts** to completion in ~2 s wall time, all bodies correctly
  reaching sleep (sleeping islands are skipped, so cost tracks the active
  wavefront rather than the total body count).

**Correctness & validation**

- Wrote a headless, deterministic validation suite of **300+ automated
  assertions across 18 categories** that drives the real solver and compares
  measured results to closed-form physics — **pendulum period within 0.2%** of
  `2π√(L/g)`, **angular-momentum conservation to ~1e-6** relative error — plus
  a scene harness that checks every scene's intended behaviour and a four-layer
  physics validation lab.
- Held numerical stability under adversarial sweeps: stacks stable to a
  **1000:1 mass ratio** with near-zero penetration, correct resting heights
  from 0.05 m to 5 m bodies, and Coulomb-correct slope slide/hold behaviour
  across friction coefficients.
- Documented the engine's real limits honestly rather than faking them — e.g.
  the gyroscope stays upright but the sequential-impulse integrator dissipates
  gyroscopic precession, and a cantilever scene was removed because the solver
  has no bending-stiffness primitive.

**Tooling & machine-learning layer**

- Built an interactive Dear ImGui laboratory: a searchable browser of **18
  tunable, deterministic scenes** (Newton's cradle, Atwood machine, trebuchet,
  domino spiral, a many-body contact stress test, an energy-controlled
  explosion, and more), each with live parameter sliders, a real-time
  energy/momentum/contacts telemetry panel, and five physics-inspector overlays.
- Engineered a rendering-independent research layer: a gym-style `Environment`
  (`reset` / `step` / `observe` / `act`), a **deterministic CSV dataset
  generator** that emits supervised state → future-position samples reproducible
  from a seed, and a swappable trajectory predictor with an ONNX slot that falls
  back to a real physics rollout — enabling ML experiments with zero OpenGL
  dependency.

---

<!-- Add your next portfolio project below, following the same structure:   -->
<!--   ## Project Title                                                      -->
<!--   *tech stack* + links                                                  -->
<!--   1–2 sentence description                                              -->
<!--   images (half the space) + bullets grouped by concern                  -->
