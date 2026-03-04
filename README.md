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
- [Simulation Mission Format](docs/simulation-mission-format.md)
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
- `docs/tutorials/charts/mission_xyz_status.png`

## Status Snapshot

### Completed

- Split control/simulation runtime architecture
- ENU translational dynamics with yaw/pitch/roll thrust projection
- Common + differential motor mixing (`common_motor_rpm` + yaw/pitch/roll terms)
- Battery-aware motor behavior (including depletion cutoff), plus thermal/current telemetry
- GPS perfect-state propagation from ENU to geodetic + noisy GPS sensing path
- Configurable weather disturbance model (steady, gust, turbulence)
- Ground-lock constraint to prevent movement while clamped on ground
- RealDrone-integrated XY position controller (position/velocity feedback -> pitch/roll references)
- Default-enabled XY position hold outside mission mode (auto-latched current XY hold reference)
- Mission YAML support with step-based execution (time-based and completion-based advancement)
- YAML-configurable position-hold tuning (`position_hold_enabled`, `position_hold_kp_pos`, `position_hold_kp_vel`, `position_hold_kd_vel`, `position_hold_max_velocity_mps`, `position_hold_max_tilt_rad`)
- Extended telemetry and charts (weather panel + dual-axis energy/temperature view)
- Mission loader + executor tests and position-controller unit coverage

### In Progress

- Mission strict-validation mode for unknown schema values (currently permissive fallback)
- Richer rigid-body rotational dynamics
- Scenario-driven comparison workflows and broader robustness/fault coverage

### Scope Note

The simulator intentionally remains simplified for control-development workflow iteration rather than high-fidelity digital-twin prediction.

## Known Issues

- Position hold is available but still not consistently reliable in all scenarios (notably takeoff/hover under disturbance and noisy sensing).
- Altitude controller behavior still requires additional tuning/rework for robust lift-off and altitude tracking.
- Use tutorial artifacts for evaluation and debugging: `docs/tutorials/simulation_telemetry.csv`, `docs/tutorials/simulation_events.log`, and charts under `docs/tutorials/charts/`.

## License

See [LICENSE](LICENSE) for details.
