# virtDrone Documentation

This folder contains the project documentation for architecture, runtime usage, and status.

## Contents

- [Architecture](architecture.md)
- [How to Use](how-to-use.md)
- [Simulation Mission Format](simulation-mission-format.md)
- [Simulation and Control Interaction](tutorials/simulation-real-drone-interaction.md)
- [Current State](current-state.md)
- [Roadmap](roadmap.md)
- [Changelog](changelog.md)
- [Latest Validation Summary](../reports/refactor-validation-summary.html)

## Usage Notes

- Run/build/test instructions are documented with Docker commands.
- Runtime artifacts are written under `docs/tutorials/` and `docs/tutorials/charts/` by default.
- Latest validation HTML summary is written to `reports/refactor-validation-summary.html`.
- Transport contract: [proto/simulation_gateway.proto](../proto/simulation_gateway.proto)
- Architecture sketch: [drawings/virtDrone_grpc_process_architecture.excalidraw](drawings/virtDrone_grpc_process_architecture.excalidraw)
