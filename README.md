# virtDrone — A Playground for System-Level Thinking 🚀

## Tagline
**virtDrone** is an early-stage, intentionally simplified drone simulation platform built to explore **architectural decomposition, tradeoffs, and systems-level reasoning**. Think of it as a sandbox for how a complex drone system *could* be structured — not a full simulator.

---

## Overview
The purpose of this project is **not** to build a high-fidelity flight simulator. Instead, virtDrone demonstrates:

* How to think about **simulation, control boundaries, and abstraction layers**
* Explicit **engineering tradeoffs** when designing cyber-physical systems
* Early-stage modular architecture for drones and control interfaces

This repository shows *how I reason about systems*, with inspiration from aerospace, robotics, and safety-critical software design.

---

## Problem Statement
Modern drone and flight software stacks are complex, slow to iterate on, and often hide software design issues behind heavy physics engines and tooling.

virtDrone aims to provide:

* A **lightweight, understandable simulation environment**
* Fast iteration on **system architecture and control logic**
* Clear separation between **plant**, **control**, and **interfaces**

Focus is on *reasoning about the system*, not achieving perfect realism.

---

## Design Goals
* Clear separation of responsibilities between components
* Deterministic, easy-to-debug behavior
* Testable modules with minimal coupling
* Explicit tradeoffs visible in code and documentation
* Foundation for future HIL (hardware-in-the-loop) exploration

---

## Non‑Goals
* High-fidelity aerodynamics
* Accurate propeller or CFD-based modeling
* Detailed IMU / sensor noise simulation
* Autopilot tuning or real flight performance prediction

These are intentional omissions to **prioritize clarity and architectural reasoning**.

---

## System Decomposition
### World / Environment
* Owns simulation time
* Defines reference frames
* Advances the simulation deterministically

### Drone (Plant)
* Owns physical state (position, velocity, orientation)
* Models energy consumption and basic dynamics
* Exposes clear inputs and outputs

### Control Interfaces
* Abstract control inputs (throttle, attitude, etc.)
* Decouple control logic from the underlying plant
* Allow future swapping between simulated and real hardware

### External Interfaces
* Telemetry output
* Control input sources (scripts, tests)

This mirrors the separation of **flight software**, **vehicle dynamics**, and **ground systems**.

---

## Modeling Philosophy
* Simple, predictable, and monotonic models
* Battery modeled for relative trends, not exact flight time
* Motor response simplified to make control relationships obvious
* Environmental effects are minimal by design

> *Does this model help reason about the system?*

---

## Tradeoffs & Intentional Simplifications
* **Clarity > Accuracy**
* **Iteration speed > Realism**
* **Explicit assumptions > Hidden complexity**

Linear/near-linear models and simplified dynamics are intentional placeholders for architectural exploration.

---

## Testing (Runnable Part)
Testing is the **first-class feature currently available**.

### Using Docker (Recommended)
```bash
docker compose run --rm dev
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

### Directly on Host
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

Tests are the only currently runnable component and demonstrate modular behavior.

### Future Directions

* Fault injection and degraded-mode scenarios

* Optional higher-fidelity models

* Minimal interactive CLI or UI

* Hardware-in-the-loop abstractions

These are planned, but the core design reasoning is already visible.

### Final Note

virtDrone is not about building the best drone simulator. It is about demonstrating how complex engineering systems can be approached, decomposed, and reasoned about — the mindset is the real deliverable.

## License

See [LICENSE](LICENSE) for details.
