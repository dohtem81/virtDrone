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

## Current Scope Notes

- Plant model is first-order and simplified
- Control and simulation are split (real-drone logic vs plant physics)
- ENU force-vector translation is implemented with yaw/pitch/roll thrust projection
- Motor commands use common + differential split (per-motor mixing)
- Sensor noise is applied on the sim-to-real connection path

## License

See [LICENSE](LICENSE) for details.
