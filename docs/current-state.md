# Current State

## Completed

- Split runtime architecture: real-drone control loop (`drone/`) and simulation plant (`simulator/`)
- Closed-loop altitude control with configurable PID behavior from YAML
- Common + differential actuation model (`common_motor_rpm` + yaw/pitch/roll differential mixing)
- ENU translational dynamics with yaw/pitch/roll thrust projection
- Battery-aware motor limiting (including depletion cutoff) and thermal/current telemetry
- GPS perfect-state propagation from ENU state to geodetic position + NED velocity representation
- Connection-layer noisy sensing (altitude, GPS position/velocity, battery voltage, temperature)
- Configurable weather disturbance model:
  - steady acceleration
  - sinusoidal gusts
  - seeded turbulence noise
- Ground lock behavior: while grounded (`z <= 0`) the simulator clamps position/velocity/acceleration to prevent drift
- Telemetry extensions:
  - sensed/perfect pairs for altitude, power, temperature, GPS position, GPS velocity
  - controller terms (`TgtErr`, `P`, `I`, `D`)
  - mixer state (`ComRPM`, `MixYPR`, `MRef`)
  - ENU and attitude state (`PosENU`, `VelENU`, `YPR`)
  - weather state (`WTotAcc`, `WSteady`, `WGust`, `WTurb`)
- Charting pipeline:
  - full + minimal dashboards
  - UTF-8/UTF-16 log decoding fallback
  - weather plotting support
  - dual-axis energy/temperature panel in full dashboard
- Test coverage additions for force dynamics, GPS mapping/noise path, weather model/config, mixer behavior, and ground-lock regression

## Important realism note

The simulator currently uses **simplified, first-order approximations** for control-development workflows.

It is not calibrated as a high-fidelity digital twin.

## In progress / next focus

- Broader multi-axis control stack (beyond altitude-centric control)
- Higher-fidelity rotational rigid-body dynamics (torque/inertia integration)
- Scenario-driven repeatable experiments and richer run-to-run comparison tooling
- Expanded failure/degradation scenario coverage

## Known limitations

- No CFD-grade aerodynamics or blade-element fidelity
- No structural flexibility or high-order vibration model
- Default parameters are generic and not identified from a specific airframe dataset
- Current dynamics remain intentionally simplified for control workflow iteration

## What this is good for

- Control-loop integration testing
- Tuning workflow exploration
- Observability/logging and debugging of controller behavior
- Studying saturation, battery effects, and noisy sensing behavior
