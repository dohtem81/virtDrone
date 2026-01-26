# virtDrone

## Overview

**virtDrone** is an early-stage, intentionally simplified drone simulation platform.

The purpose of this project is **not** to build a high-fidelity flight simulator, but to demonstrate **systems-level thinking**, architectural decomposition, and explicit engineering tradeoffs when designing complex cyber‑physical systems.

This repository is meant to show *how I think* about problems such as simulation, control boundaries, abstraction layers, and failure handling — the kinds of concerns that matter in aerospace, robotics, and safety‑critical software.

---

## Problem Statement

Modern drone and flight software stacks are complex, slow to iterate on, and often hide software design issues behind heavy physics engines and tooling.

The goal of virtDrone is to provide:

* A **lightweight simulation environment**
* Fast iteration on **system architecture and control logic**
* Clear separation between **plant**, **control**, and **interfaces**

The emphasis is on *reasoning about the system*, not achieving perfect realism.

---

## Design Goals

* Clear separation of responsibilities between components
* Deterministic, easy-to-debug behavior
* Testable modules with minimal coupling
* Ability to evolve toward hardware‑in‑the‑loop (HIL) concepts later
* Make tradeoffs explicit and visible in code and documentation

---

## Non‑Goals (Intentional)

The following are **explicit non‑goals** at this stage:

* High‑fidelity aerodynamics
* Accurate propeller or CFD‑based modeling
* Detailed IMU / sensor noise simulation
* Autopilot tuning or real flight performance prediction

These omissions are intentional.

High realism increases complexity, slows iteration, and obscures architectural and control‑logic issues that are the focus of this project.

---

## System Decomposition

The system is decomposed into a small number of explicit components:

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
* Control input sources (UI, scripts, tests)

This mirrors real-world separation between **flight software**, **vehicle dynamics**, and **ground systems**.

---

## Modeling Philosophy

Models are chosen to be:

* Simple
* Monotonic
* Predictable

For example:

* Battery behavior is modeled to capture *relative* energy consumption trends, not absolute flight time
* Motor response is simplified to make control relationships obvious
* Environmental effects are minimal by design

The guiding question is always:

> *Does this model help reason about the system?*

---

## Tradeoffs & Intentional Simplifications

This project intentionally accepts the following tradeoffs:

* **Accuracy vs clarity**: clarity wins
* **Speed of iteration vs realism**: iteration wins
* **Explicit assumptions vs hidden complexity**: assumptions are documented

Examples:

* Linear or near‑linear models are preferred early
* No attempt is made to perfectly match real hardware
* Control loop timing is explicit and observable

Each simplification is a placeholder, not a blind spot.

---

## Failure‑Mode Thinking (Planned)

A core design goal is to support **fault injection** and degraded‑mode behavior.

Planned examples include:

* Battery voltage sag under load
* Motor efficiency degradation
* Delayed or dropped control updates

The intent is to observe:

* How control logic responds
* Whether system boundaries are respected
* Where assumptions break

This reflects real-world aerospace and robotics failure analysis.

---

## Build & Run

> **Note:** This project is intentionally lightweight. Build instructions favor clarity over tooling complexity.

### Prerequisites

* C++17 compatible compiler (GCC / Clang)
* CMake >= 3.16
* Python 3.9+ (for tooling / UI components, if enabled)
* Git

Optional:

* Docker (for isolated builds)

---

### Clone Repository

```bash
git clone https://github.com/dohtem81/virtDrone.git
cd virtDrone
```

---

### Build (Native)

```bash
mkdir -p build
cd build
cmake ..
cmake --build .
```

The resulting simulator binary will be placed in the build directory.

---

### Run Simulator

```bash
./virtDrone
```

By default, the simulator:

* Starts with a single drone instance
* Advances simulation time deterministically
* Emits basic telemetry to stdout

(Exact runtime behavior may evolve as the project grows.)

---

### Python / UI Components (Optional)

If Python-based tooling or UI components are present:

```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Run the backend or UI as documented in the relevant subdirectory.

---

### Docker (Optional)

If a Dockerfile is provided:

```bash
docker build -t virtdrone .
docker run --rm virtdrone
```

Docker is intended for reproducibility, not performance.

---

## Testing Philosophy

Testing is treated as a **first‑class design tool**, not an afterthought.

* Unit tests validate component‑level behavior
* Tests focus on *interfaces and contracts*, not internal implementation
* Deterministic simulation allows reproducible tests

### Running Tests

```bash
ctest --test-dir build
```

---

## Why Not Use Existing Simulators?

Tools like Gazebo, AirSim, or PX4 SITL are powerful but heavy.

They are excellent for:

* Autopilot validation
* High‑fidelity simulation
* Sensor and perception work

virtDrone exists for a different purpose:

* Architectural exploration
* Control boundary design
* System‑level reasoning

This project can later integrate with heavier tools if needed.

---

## Project Status

This repository represents an **early stage** of development.

Incomplete areas are expected and intentional.

The focus is on:

* Getting the structure right
* Making assumptions explicit
* Creating a foundation that can evolve

---

## Future Directions

Potential future work includes:

* Hardware‑in‑the‑loop abstractions
* Configurable fault injection
* More detailed (optional) models
* Clear separation between control algorithms and vehicle model

None of these are required for the project to meet its primary goal.

---

## Final Note

virtDrone is not about building the *best* drone simulator.

It is about demonstrating **how complex engineering systems should be approached, decomposed, and reasoned about**.

That mindset is the real deliverable.

## License

See [LICENSE](LICENSE) for details.