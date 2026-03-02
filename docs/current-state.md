# Current State

## What is implemented now

- Closed-loop altitude control in split real/sim architecture
- Simplified plant physics:
  - Motor dynamics with ramp limits
  - Thrust model
  - Battery discharge and voltage effects
  - Thermal behavior
  - Vertical dynamics with damping
- Connection-layer sensor noise for real-drone control inputs
- Rich telemetry logging with sensed/perfect pairs and controller terms
- Charting pipeline for flight logs

## Important realism note

The simulator currently uses **simplified, first-order approximations** for control-development workflows.

It is not calibrated as a high-fidelity digital twin.

## Known limitations

- No CFD-grade aerodynamics
- No wind/turbulence environment model
- No structural flexibility or high-order rigid-body model
- Coarse vertical-only dynamics focus in current loop
- Parameters are not tuned to a specific real airframe by default

## What this is good for

- Control-loop integration testing
- Tuning workflow exploration
- Observability/logging and debugging of controller behavior
- Studying saturation, battery effects, and noisy sensing behavior
