# Physics Validation Laboratory

This document is the scientific record of the engine's validation suite. It
describes the methodology, the measured accuracy of every experiment against
closed-form physics, and the known limitations the suite has surfaced.

The suite is headless, deterministic, and free of any artificial stabilization
(no damping, clamping, snapping, or teleportation added to make a result look
right). It is built and run with:

```
cmake --build build --target ValidationSuite
build/ValidationSuite.exe
```

A given seed reproduces an entire run byte-for-byte. The current run uses seed
`20240501`.

---

## The four layers

Every experiment is scrutinised at up to four layers (see `src/app/ValidationLab.h`):

| Layer | Name | What it checks |
| --- | --- | --- |
| **A** | Numerical integrity | finite positions/velocities/spins, normalized quaternions, bounded coordinates and angular speeds, interpenetration ≤ solver slop, deterministic replay |
| **B** | Constraint validation | per-constraint residual error vs. a tolerance (rope 0.5 mm, hinge 0.25 mm, pulley 1 mm), read from the solver's telemetry |
| **C** | Physical law | a measured observable vs. its analytical prediction, reported as theoretical / measured / **percentage error** with a pass band |
| **D** | Stress testing | hundreds of randomized variants; the gate is *numerical stability* across the parameter space, with accuracy reported separately as a distribution |

A result that is stable but not accurate is recorded as a **known limitation** —
logged loudly, never hidden behind a relaxed tolerance.

---

## Measured accuracy (Layer C)

All figures from the seed-`20240501` run, `dt = 1/60 s`, `g = 9.81 m/s²`.

| Experiment | Law | Theoretical | Measured | Error | Stress (Layer D) |
| --- | --- | ---: | ---: | ---: | --- |
| Inclined plane | `a = g(sinθ − μcosθ)` | 3.2059 | 3.2058 | **0.00%** | 200/200 stable |
| Atwood machine | `a = (m₁−m₂)/(m₁+m₂)·g` (3:1) | 4.905 | 4.978 | **1.49%** | 200/200 stable |
| Atwood machine | rope inextensibility (Σℓ const) | 6.0828 | 6.0833 | **0.01%** | — |
| Simple pendulum | `T = 2π√(L/g)` | 3.4746 | 3.4833 | **0.25%** | 120/120 stable |
| Spring oscillator | `T = 2π√(m/k)` | 0.99346 | 1.0000 | **0.66%** | 120/120 stable |
| Terminal velocity | `v_t = √(2mg / ρ C_d A)` | 23.289 | 23.114 | **0.75%** | 150/150 stable |
| Elastic collision | linear momentum | 2.000 | 2.000 | **0.00%** | — |
| Elastic collision | kinetic energy | 2.000 | 2.000 | **0.00%** | — |
| Torque-free spin | angular momentum \|L\| | 0.160 | 0.160 | **0.00%** | — |
| Projectile | time of flight `2v sinθ/g` | 2.8832 | 2.8665 | **0.58%** | 150/150 stable |
| Projectile | range `v²sin2θ/g` | 40.775 | 40.539 | **0.58%** | — |
| Double pendulum | energy drift over 10 s | 0% | 10.1% | (≤15% band) | — |
| Static stack | COM horizontal drift | 0 | 0.0024 m | **0.24%** | 100/100 stable |

**Suite total: 37 passed, 0 failed, 3 known-limitations.** Roughly 1,200
randomized stress variants run per suite execution, all numerically stable.

Every translational / linear-momentum / energy law validates to well under the
1–3% target. The three known limitations below are all rotational.

---

## Known limitations (surfaced by the lab)

These are real, reproducible solver characteristics the suite measured. They are
recorded here rather than hidden, and they cluster around **one root theme: the
rotational / angular dynamics are under-energetic** — the constraint solver
damps rotational motion. A single dedicated follow-up on the angular integrator
and hinge/pulley angular coupling should address all three.

| # | Finding | Measurement | Interpretation |
| --- | --- | --- | --- |
| 1 | Pulley acceleration accuracy | mean 15.3%, max 125.8% error over 193 random mass ratios | The pulley is *stable* everywhere but only *accurate* near calibrated ratios. (Independently confirmed by `PhysicsTests` RP5.) |
| 2 | Gyroscopic precession | Ω measured 0.00 vs. 1.02 rad/s theoretical (100% error) | The angular integrator produces **no** precession — gyroscopic coupling (torque ⟂ spin → precession) is not reproduced. |
| 3 | Double-pendulum chaos | tip separation 0.0007 → 0.0009 m over 8 s (no growth) | A tiny perturbation does not diverge; hinge angular damping suppresses the chaotic behaviour a double pendulum should show. |

Because of #2 and #3, the **Gyroscope** and **chaotic Double-Pendulum**
laboratory scenes are deliberately **not** shipped as "validated" — presenting
them would be physically false. They are deferred until the angular dynamics are
fixed.

---

## Scene status

| Scene | Physics basis | Status |
| --- | --- | --- |
| Ballistics | projectile kinematics (0.58% validated) | **shipped** |
| Inclined Plane | `a = g(sinθ−μcosθ)` (0.00%) | shipped (prior) |
| Atwood Machine | coupled acceleration (1.49% nominal) | shipped (prior); pulley accuracy limited (see #1) |
| Newton's Cradle, Rope/Suspension Bridge, Cantilever, Hanging Chain | static equilibrium + rope tension (validated) | shipped (prior) |
| Double Pendulum, Spring Pendulum | pendulum/spring (validated) | shipped (prior; not claimed chaotic — see #3) |
| Trebuchet | hinge + rope + counterweight | shipped (prior; qualitative) |
| Gyroscope | gyroscopic precession | **deferred** (see #2) |
| Truss Collapse, Cable-Stayed Bridge, Rube Goldberg | large multi-subsystem | **deferred** (scope) |

---

## AI-readable telemetry

`src/ml/TelemetryExport.h` captures the solver's per-step `TelemetryFrame` to a
flat CSV (one row per body per frame: transform, velocity, angular velocity,
orientation quaternion, plus per-frame contacts, energy, momentum, per-
constraint error, and sleeping state). It is rendering-independent and
deterministic — the same simulation produces the same CSV byte-for-byte.

---

## Deferred (not yet built)

- **Dear ImGui dockable GUI** (sidebar / inspector / live telemetry panels) — no
  GUI library is vendored; adding one is a separate effort.
- **Angular-dynamics fix** — the root cause behind limitations #1–#3.
- **Large non-analytic scenes** — Truss Collapse, Cable-Stayed Bridge, Rube
  Goldberg, and the Gyroscope, which depend on the angular fix or are out of
  current scope.
