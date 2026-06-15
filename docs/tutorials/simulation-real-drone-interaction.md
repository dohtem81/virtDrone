# Simulation and Control Interaction

This guide explains how the control process and simulation process interact in the current architecture.

## Components

Control side:

- `drone_app`
- `RunControlSession(...)`
- `RealDrone`

Simulation side:

- `simulator_app`
- `QuadroSimulation`

Boundary:

- `SimulationGateway`
- gRPC transport adapters
- `proto/simulation_gateway.proto`

## Startup Sequence

Simulation process startup (`src/simulator/main.cpp`):

1. Parse CLI arguments.
2. Load weather config.
3. Create simulation instance with `QuadroSimulationFactory(...)`.
4. Configure telemetry/event log outputs.
5. Start simulation runtime.
6. Start gRPC gateway service and wait.

Control process startup (`src/drone/main.cpp`):

1. Parse CLI arguments including backend selector.
2. Resolve logs directory.
3. Create gateway via `CreateControlGateway(...)`.
4. Enter `RunControlSession(...)`.

## One Tick (Ordered)

Inside `RunControlSession(...)` each tick is:

1. Read current sensors from gateway.
2. Update mission state when a mission is loaded.
3. Execute `RealDrone` update to produce actuator command.
4. Send actuator frame to gateway.
5. Ask simulator to advance `step(dt_s)`.

This preserves a clear control-then-plant update order.

## Ownership Boundary

Control side owns:

- mission transitions
- target selection
- control math and actuator requests

Simulation side owns:

- plant state
- motor/battery/force integration
- weather effects
- GPS and simulated sensor state production

## Real Hardware Readiness

The control side is already separated so it can target a hardware backend.

Current backend implementation status:

- `grpc`: implemented
- `hardware`: not implemented yet

## Runtime Mental Model

- `RealDrone` is the controller.
- `QuadroSimulation` is the plant and environment.
- `SimulationGateway` is the only allowed communication boundary.