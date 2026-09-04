# Murad Dashdamirov

dashdamirov.murad11@gmail.com | [LinkedIn](https://www.linkedin.com/in/murad-dashdamirov-90461934a) | [GitHub](https://github.com/Murad-D-11) | Canadian Citizen

---

## Education

**University of Waterloo** — Honours Software Engineering (Co-op), BASc
Sept. 2026 – Apr. 2031 (expected)

---

## Skills

**Languages:** TypeScript, JavaScript, Python, SQL, C++
**Backend:** Node.js, Express, Prisma, PostgreSQL, REST API design, Zod
**Frontend:** React, Vite, Tailwind CSS, Recharts
**Machine Learning / Data:** scikit-learn, pandas, NumPy, ONNX
**Graphics / Systems:** OpenGL, GLM, Dear ImGui
**Tools:** Git, Docker, CMake, Vitest, pytest
**Spoken Languages:** English, Russian

---

## Projects

**Canada Productivity Intelligence** — Full-stack ML forecasting platform &nbsp; | &nbsp; [GitHub](https://github.com/Murad-D-11/canada-productivity-intelligence)
*TypeScript, React, Node/Express, Python, scikit-learn, PostgreSQL, Prisma, Docker*

- Built a three-tier decision-support platform (React + TypeScript frontend, Node/Express API, Python ML package) that ingests 29,443 Statistics Canada labour-productivity observations across 21 industries and 8 measures (1981–2026) into PostgreSQL via a Prisma-backed ETL pipeline.
- Developed a one-quarter-ahead productivity forecasting model in scikit-learn, comparing a naive baseline, ridge regression, and random forest under a leakage-safe pipeline (median imputation + standardization fit only on the training fold) with chronological train/validation/test splits, selecting ridge by validation MAE (2.82, R² 0.69).
- Engineered a per-forecast explainability layer that decomposes each linear prediction exactly into a base value plus per-feature contributions, surfacing the top drivers of every forecast without approximation.
- Designed a REST API of ~10 endpoints (forecast, industry ranking, scenario simulation, data status) with Zod request validation and a JSON stdin/stdout bridge that invokes the Python model from Node, mapping model states to typed HTTP errors so no values are ever fabricated.
- Verified the system with 18 backend (Vitest/Supertest) and ~50 ML (pytest) test cases covering data-leakage prevention, chronological validation, and suppressed-value handling.

**3D Rigid-Body Physics Engine & Simulation Lab** &nbsp; | &nbsp; [GitHub](https://github.com/Murad-D-11/Physics)
*C++17, OpenGL, GLM, Dear ImGui, CMake, Python*

- Built a 3D rigid-body physics engine from scratch in C++ (no physics libraries) with an impulse-based sequential solver, multi-point contact manifolds, quaternion rotational dynamics, and 4 constraint types (springs, hinges, ropes, pulleys), sustaining 500 interacting bodies at ~3 ms per 1/60 s step, within the 16.6 ms real-time budget.
- Cut per-body simulation cost growth to 1.33x across a 50x increase in body count (10 to 500) by implementing a spatial-hash broad phase, replacing naive O(n^2) pair checking with near-linear scaling confirmed by a profiling benchmark.
- Eliminated fast-body tunnelling up to 200 m/s impact speeds by adding conservative-advancement continuous collision detection, and held stacks stable up to a 1000:1 mass ratio with near-zero penetration under adversarial stress sweeps.
- Verified physical correctness against closed-form solutions with a headless suite of 300+ automated assertions across 18 categories, achieving pendulum-period accuracy within 0.2% and angular-momentum conservation to ~1e-6 relative error.
- Engineered a rendering-independent ML data layer (gym-style environment plus a deterministic CSV dataset generator) and an interactive Dear ImGui lab of 18 tunable scenes with live energy/momentum telemetry, producing reproducible trajectory-prediction datasets.

---

## Awards

- **Sir Isaac Newton Exam (Physics) — Distinction Certificate:** 2025, 2026.
