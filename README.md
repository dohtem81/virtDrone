# virtDrone
**Vehicle Subsystem Simulation & Control Framework (Work in Progress)**

## Overview

virtDrone is a modular framework for exploring vehicle subsystem modeling and control-loop integration.

The current implementation is intentionally **simplified** and aimed at engineering workflow exploration, not high-fidelity flight prediction.

## Documentation

Detailed documentation lives in the `docs/` folder:

- [Documentation Index](docs/README.md)
- [Architecture](docs/architecture.md)
- [How to Use](docs/how-to-use.md)
- [Simulation ↔ Real Drone Interaction (Beginner Guide)](docs/tutorials/simulation-real-drone-interaction.md)
- [Current State](docs/current-state.md)
- [Roadmap](docs/roadmap.md)
- [Changelog](docs/changelog.md)

## Quick Start

### Build and Run (Docker)

```bash
docker compose run --rm dev cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
docker compose run --rm dev cmake --build build --target simulator_app
docker compose run --rm dev ./build/simulator_app 1000 0.01
```

### Generate Chart

```bash
docker compose run --rm chart
```

### Run Chart Parser Tests

```bash
docker compose run --rm chart-test
```

Default chart output:

- `docs/tutorials/charts/flight_dashboard.png`

## Status Snapshot

### Completed

- Split control/simulation runtime architecture
- ENU translational dynamics with yaw/pitch/roll thrust projection
- Common + differential motor mixing (`common_motor_rpm` + yaw/pitch/roll terms)
- Battery-aware motor behavior (including depletion cutoff), plus thermal/current telemetry
- GPS perfect-state propagation from ENU to geodetic + noisy GPS sensing path
- Configurable weather disturbance model (steady, gust, turbulence)
- Ground-lock constraint to prevent movement while clamped on ground
- Extended telemetry and charts (weather panel + dual-axis energy/temperature view)

### In Progress

- Multi-axis control expansion beyond altitude-centric loop
- Richer rigid-body rotational dynamics
- Scenario-driven comparison workflows and broader robustness/fault coverage

### Scope Note

The simulator intentionally remains simplified for control-development workflow iteration rather than high-fidelity digital-twin prediction.

## License

See [LICENSE](LICENSE) for details.
