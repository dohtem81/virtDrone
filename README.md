# Drone Simulation Platform

This is a **hobby project** focused on building a simplified yet reasonably complete drone simulation environment. The goal is not high‑fidelity physics, but rather a clear demonstration of **system architecture**, **component abstraction**, and **testability** — with an eye toward eventually reusing parts of the system on real hardware.

## Project Goals

The project aims to simulate a drone and its surrounding ecosystem, including:

* Drone state (position, speed, orientation)
* Sensors (virtualized and abstracted)
* Battery drain, efficiency, and basic thermal/loss modeling
* Simplified physics and motion dynamics
* Telemetry reporting back to a central "home base"

The emphasis is on **how the system is structured**, not on perfect physical accuracy.

## Architecture Overview

The system is split into two major parts:

### 1. Drone Simulator

The simulator models the internal behavior of a drone:

* Sensor simulation (IMU, GPS-like data, etc.)
* Power consumption and battery state
* Motion and speed calculation based on simplified physics
* Hardware abstraction layers separating logic from implementation

All components are designed to be:

* Modular
* Unit-testable
* Replaceable with real hardware drivers in the future

### 2. Home Base (Web Interface)

The home base is a **web-based control and monitoring interface** that consists of two parts:

#### Backend
- Maintains communication with the simulated drone
- Handles telemetry data exchange and command routing
- Acts as the server-side logic for processing drone updates

#### UI (Web with Real-Time Updates)
- Provides a user interface for monitoring live parameters (battery, speed, position, sensor data)
- Allows **manual control** of drone movement
- Displays real-time updates via web technologies (e.g., WebSockets or polling for live data)

This separation mirrors real-world drone and robotics systems.

## Design Principles

* **Simplified physics**: good enough to reason about behavior and system flow
* **Clear abstractions**: hardware vs software boundaries are explicit
* **Test-first mindset**: components are designed to be unit-tested in isolation
* **Simulation-first**: the same interfaces should work with both simulated and real hardware
* **Educational/demo focus**: architecture and approach matter more than realism

## What This Project Is (and Isn’t)

✅ A demo of:

* Distributed system design
* Simulation-driven development
* Hardware abstraction patterns
* Telemetry and control loops

❌ Not:

* A flight-accurate drone physics engine
* A real-time flight controller replacement
* Production-ready autopilot software

## Why This Exists

This project exists to explore and demonstrate:

* How to design systems that scale from simulation to real hardware
* How to structure code for testing complex, stateful systems
* How simulation can accelerate development and validation

If parts of this project eventually run on real hardware — that’s a success, not a requirement.

---

**Status:** Building foundation components. No backend or frontend yet. Experimental / hobby project

Contributions, feedback, and architectural discussions are welcome.

A drone simulator project with three main components: simulator (physics), drone (model and control), and web-based control interface with visualization.

## Technologies

- **Simulator and Drone**: Implemented in C++ for performance.
- **Web Interface**: Backend in Python using FastAPI, with potential frontend in JavaScript/React.
- **Testing**: Catch2 for C++ unit tests, pytest for Python tests.
- **Build System**: CMake with Ninja for C++, Docker for containerization.

## Components

- **Simulator**: Handles physics simulation including position, speed, tilt, etc. (C++).
- **Drone**: Manages motor speeds, temperatures, and control algorithms (C++).
- **Web Interface**: Provides control interface and visualization via web technologies (Python/FastAPI).

## Folder Structure

- `src/`: Main source code
  - `simulator/`: Physics simulation (C++)
  - `drone/`: Drone logic (C++)
  - `web/`: Web interface (Python)
- `libs/`: Shared libraries (C++)
- `docs/`: Documentation
- `tools/`: Build and utility scripts (includes CMake for C++)
- `tests/`: Unit and integration tests
- `examples/`: Sample projects
- `configs/`: Configuration files

## Getting Started

### Prerequisites

- Docker and Docker Compose
- Git

### Building

#### Using Docker (recommended)
```bash
docker compose run --rm dev
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

### Running Tests

The project uses Catch2 for C++ unit tests and pytest for Python tests.

1. Run C++ tests:
   ```bash
   docker compose run --rm dev ctest
   ```

2. Run Python tests:
   ```bash
   docker compose run --rm dev python -m pytest tests/unit/web/
   ```

### Running the Application

1. Start all components:
   ```bash
   docker compose up --build
   ```

2. Access the web interface at http://localhost:8000

### Local Development

For local development without Docker:

1. Install dependencies:
   - C++: CMake, Ninja, GCC/Clang
   - Python: `pip install -r requirements.txt`

2. Build C++ components:
   ```bash
   mkdir build && cd build
   cmake .. -GNinja
   ninja
   ```

3. Run tests:
   ```bash
   ctest  # For C++ tests
   python -m pytest tests/unit/web/  # For Python tests
   ```

4. Run components individually (refer to `docs/` for details).

## License

See [LICENSE](LICENSE) for details.
