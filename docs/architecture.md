# Architecture

## Runtime Split

virtDrone runs as two explicit sides connected by a boundary contract.

- Control side (can run with real hardware in the future):
  - `drone_app`
  - `drone::runtime::RealDrone`
  - `drone::mission::MissionLoader`, `drone::mission::MissionExecutor`
  - controller config loading and control-session orchestration
- Simulation side (simulation only):
  - `simulator_app`
  - `drone::simulator::QuadroSimulation`
  - physics, weather, GPS synthesis, battery/motor simulation, telemetry CSV/event logs
- Boundary contract:
  - `drone::runtime::SimulationGateway`
  - proto contract: `proto/simulation_gateway.proto`
  - current transport implementation: gRPC client/server adapters

The control process does not call simulator internals directly.

## Hardware vs Simulation

Real-hardware capable side:

- `drone_app`
- `RunControlSession(...)`
- `RealDrone`
- mission execution and control logic

Simulation-only side:

- `simulator_app`
- `QuadroSimulation` and physics/environment stack
- simulation telemetry and simulator event logs

Backend status:

- `grpc` backend: implemented and active
- `hardware` backend: selected via CLI but currently not implemented (factory returns null)

## Per-Tick Flow

Ordered flow inside the control loop:

1. Control side reads `SensorFrame` through `SimulationGateway::readSensors()`.
2. If a mission is loaded, control side runs mission-step update logic.
3. Control side runs `RealDrone::update(...)` to compute `ActuatorFrame`.
4. Control side sends actuator output through `SimulationGateway::applyActuators(...)`.
5. Control side requests simulator advance through `SimulationGateway::step(dt_s)`.
6. Simulation side applies motor commands, advances physics/weather/GPS, and stores updated state for the next sensor read.

This order is fixed: control computes first from the current sample, then plant advances.

## Key Interfaces

- `drone::runtime::SensorSource`
- `drone::runtime::ActuatorSink`
- `drone::runtime::SimulationGateway`
- `drone::runtime::GrpcSimulationGatewayClient`
- `drone::runtime::GrpcSimulationGatewayService`
- `drone::runtime::CreateControlGateway(...)`
- `drone::runtime::RunControlSession(...)`
- `drone::runtime::RealDrone`
- `drone::simulator::QuadroSimulation`

## Data Contracts

Control-to-simulation boundary payloads:

- `SensorFrame`:
  - altitude, ENU position, GPS position/velocity, battery, motor thermal/RPM, yaw/pitch/roll
- `ActuatorFrame`:
  - `common_motor_rpm`
  - yaw/pitch/roll differential terms
  - per-motor RPM refs
  - control diagnostics (target error, PID terms, sensed mirrors)

Contract source of truth: `proto/simulation_gateway.proto`.

## Build/Dependency Model

- Primary C++ build uses CMake + Ninja.
- gRPC/protobuf are resolved via system packages in the Docker dev image and consumed via `find_package(...)`.
- Catch2 remains fetched for tests.

## Migration Notes

- Runtime entrypoints are split:
  - old single-process assumptions should be replaced with `simulator_app` + `drone_app`.
- `drone_app` CLI now includes backend selection:
  - `[backend] [simulator_address] [steps] [dt_s] [altitude_config] [attitude_config] [mission_file] [logs_dir]`
- Mission ownership moved to the control domain (`drone/mission`), not simulator domain.
- Docker is the supported execution path for build/test/run documentation.

## Diagram Reference

- Architecture sketch: `docs/drawings/virtDrone_grpc_process_architecture.excalidraw`