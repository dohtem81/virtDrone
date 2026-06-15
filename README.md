# virtDrone

Vehicle subsystem simulation and control framework.

## What It Is

virtDrone is organized as a split runtime:

- control side: mission and control logic (`drone_app`)
- simulation side: plant/environment model (`simulator_app`)
- boundary: `SimulationGateway` over gRPC (current implementation)

The architecture is intended for control-development workflows, not high-fidelity digital-twin prediction.

## Documentation

- [Documentation Index](docs/README.md)
- [Architecture](docs/architecture.md)
- [How to Use](docs/how-to-use.md)
- [Simulation Mission Format](docs/simulation-mission-format.md)
- [Simulation and Control Interaction](docs/tutorials/simulation-real-drone-interaction.md)
- [Current State](docs/current-state.md)
- [Roadmap](docs/roadmap.md)
- [Changelog](docs/changelog.md)
- [Latest Validation Summary](reports/refactor-validation-summary.html)

## Quick Start (Docker)

Build:

```bash
docker compose run --rm dev cmake -S /workspace -B /workspace/build -G Ninja -DCMAKE_BUILD_TYPE=Debug
docker compose run --rm dev cmake --build /workspace/build -j
```

Run simulation side:

```bash
docker compose run --rm dev bash -lc '/workspace/build/simulator_app config/weather.yaml docs/tutorials 0.0.0.0:50051'
```

Run control side:

```bash
docker compose run --rm dev bash -lc '/workspace/build/drone_app grpc localhost:50051 10000 0.01 config/altitude_controller.yaml config/attitude_controller.yaml config/missions/hover_and_move.yaml docs/tutorials'
```

## Validate

```bash
docker compose run --rm dev bash -lc '/workspace/tools/scripts/refactor-closeout-docker.sh'
```

Latest full validation status:

- 95/95 tests passed in Docker.

## Backend Status

- `grpc`: implemented
- `hardware`: placeholder, not implemented yet

## License

See [LICENSE](LICENSE).