# Current State

## Implemented

- Split runtime architecture with explicit boundary:
  - control side: `drone_app`, `RealDrone`, mission runtime
  - simulation side: `simulator_app`, `QuadroSimulation`, plant/environment
  - boundary: `SimulationGateway` + gRPC transport
- Backend selector in control launcher:
  - `grpc` active
  - `hardware` placeholder (not implemented)
- Control session extraction:
  - reusable `RunControlSession(...)` used by `drone_app`
- Mission runtime in control side:
  - mission loading, mission transitions, completion criteria checks
- Simulation dynamics:
  - battery-aware motor model
  - ENU force integration with weather injection
  - GPS propagation
  - ground-lock behavior
- Telemetry/logging pipeline:
  - simulator telemetry/events
  - control events

## Validation Status

- Docker configure/build/test path is working end-to-end.
- Latest full run: `95/95` tests passed via Docker.
- Closeout script now skips HTML summary generation if `python3` is not available in the container.

## Known Gaps

- Hardware backend implementation is still pending.
- HTML validation summary generation is conditional on `python3` availability.

## Supported Use Cases

- Control-loop integration and regression testing
- Mission state-machine validation
- Simulator-side plant/control interaction debugging across the transport boundary