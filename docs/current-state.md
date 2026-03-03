# Current State

## What is implemented now

- Closed-loop altitude control in split real/sim architecture
- Common + differential motor command split:
  - Common reference drives shared lift demand
  - Yaw/pitch/roll controller terms are mixed as differential per-motor RPM commands
- Simplified plant physics:
  - Motor dynamics with ramp limits
  - Thrust model
  - Battery discharge and voltage effects
  - Thermal behavior
  - ENU force-vector dynamics with linear damping
  - Euler attitude state (`yaw`, `pitch`, `roll`) used for thrust projection
- Connection-layer sensor noise for real-drone control inputs
- Rich telemetry logging with sensed/perfect pairs and controller terms
- Telemetry includes mixer state (common reference, yaw/pitch/roll mix terms, per-motor references)
- Charting pipeline for flight logs

## Important realism note

The simulator currently uses **simplified, first-order approximations** for control-development workflows.

It is not calibrated as a high-fidelity digital twin.

## Known limitations

- No CFD-grade aerodynamics
- No wind/turbulence environment model
- No structural flexibility or high-order rigid-body model
- Control stack remains altitude-focused (position/attitude control loops are not implemented yet)
- Attitude commands currently drive differential motor references directly; full rotational rigid-body torque integration remains to be expanded
- Parameters are not tuned to a specific real airframe by default

## What this is good for

- Control-loop integration testing
- Tuning workflow exploration
- Observability/logging and debugging of controller behavior
- Studying saturation, battery effects, and noisy sensing behavior
