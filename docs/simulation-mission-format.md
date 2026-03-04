# Simulation Mission YAML Format

This document describes the YAML schema used to drive step-based simulation missions.

## Top-level structure

```yaml
mission:
  name: "Mission Name"
  description: "Optional description"
  version: "1.0"
  metadata:
    author: "name"
    created: "2026-03-03"
    max_duration_s: 300
  initial_conditions:
    altitude_m: 0.0
    position_enu_m:
      x: 0.0
      y: 0.0
    yaw_rad: 0.0
    battery_soc_percent: 100.0
  steps: []
```

## Step fields

Each element in `steps` supports:

- `step_id` (int, required)
- `name` (string, required)
- `action` (string, required)
- `enabled` (bool, optional, default `true`)
- `advance_mode` (`time_based` or `completion_based`)
- `duration_s` (required when `advance_mode: time_based`)
- `timeout_s` (optional, default `60.0`)
- `on_timeout` (`abort`, `proceed`, `retry`)
- `retry_count` (used when `on_timeout: retry`)
- `fallback_step_id` (reserved for future jump logic)
- `completion_criteria` (used when `advance_mode: completion_based`)

## Actions

### `hover`

Keep altitude and yaw target while maintaining XY hold.

```yaml
action: "hover"
target_altitude_m: 10.0
yaw_rad: 0.0
```

On `hover` step entry, `RealDrone` latches the current XY position as the hold target.

### `go_to_position`

Set XY position target for the RealDrone position controller and altitude hold.

```yaml
action: "go_to_position"
target_position_enu_m:
  x: 20.0
  y: 10.0
target_altitude_m: 10.0
max_tilt_rad: 0.35
max_velocity_mps: 4.0
altitude_mode: "absolute"
```

### `land`

Descend to ground.

```yaml
action: "land"
descent_rate_mps: 1.5
```

### `set_attitude`

Set direct yaw/pitch/roll references.

```yaml
action: "set_attitude"
target_yaw_rad: 0.0
target_pitch_rad: 0.1
target_roll_rad: -0.1
```

### `change_altitude`

Change altitude while not using XY position control.

```yaml
action: "change_altitude"
target_altitude_m: 25.0
rate_mps: 2.0
```

### `rotate_yaw`

Change yaw while keeping other controls external.

```yaml
action: "rotate_yaw"
target_yaw_rad: 1.5708
```

## Completion criteria

Supported `condition_type` values:

- `time_elapsed`
- `altitude_reached`
- `altitude_and_velocity`
- `position_reached`
- `yaw_reached`
- `attitude_reached`
- `velocity_low`
- `landed`

TODO: The parser currently falls back to `time_elapsed` when `condition_type` is unknown; consider adding an optional strict mode to fail mission loading on unknown values.

Common fields:

- `hold_duration_s`
- `timeout_s`

Specific fields (as needed):

- position: `target_position_enu_m`, `position_tolerance_m`
- altitude: `target_altitude_m`, `altitude_tolerance_m`
- yaw: `target_yaw_rad`, `yaw_tolerance_rad`
- attitude: `target_pitch_rad`, `target_roll_rad`, `target_yaw_rad`, `pitch_tolerance_rad`, `roll_tolerance_rad`, `yaw_tolerance_rad`
- velocity: `max_velocity_mps`, `velocity_tolerance_mps`

## Runtime behavior

- Mission is loaded from file into `RealDrone`.
- `RealDrone::updateMission(...)` is called each simulation tick.
- `MissionExecutor` (owned by `RealDrone`) applies mission step action outputs to `RealDrone`.
- `RealDrone` position controller converts XY target vs sensor ENU position/velocity into pitch/roll references.
- Mission step advancement can be time-based or completion-based.

Outside mission mode, XY position hold is still active by default (`position_hold_enabled: true`), and the current XY is used as the hold reference.

Position-hold controller behavior can be tuned from altitude-controller YAML using:

- `position_hold_enabled`
- `position_hold_kp_pos`
- `position_hold_kp_vel`
- `position_hold_kd_vel`
- `position_hold_max_velocity_mps`
- `position_hold_max_tilt_rad`

Example (defaults from `config/altitude_controller.yaml`):

```yaml
position_hold_enabled: true
position_hold_kp_pos: 1.0
position_hold_kp_vel: 10.0
position_hold_kd_vel: 2.0
position_hold_max_velocity_mps: 20.0
position_hold_max_tilt_rad: 1.3
```

## CLI usage

`simulator_app [steps] [dt_s] [altitude_config_file] [weather_config_file] [mission_file]`

Example:

`simulator_app 1200 0.02 config/altitude_controller.yaml config/weather.yaml config/missions/hover_and_move.yaml`

## Logging outputs

Simulation writes two timestamped files:

- `docs/tutorials/simulation_telemetry.csv` (per simulation step)
- `docs/tutorials/simulation_events.log` (mission/runtime events)

### `simulation_telemetry.csv` format

Each row contains a local timestamp and simulation step data. The header defines a stable schema with grouped fields:

- Simulation status: elapsed time, running flag, ground-lock flag
- Drone status from sensor perspective: sensed altitude, ENU position, GPS position/velocity, battery, motor temperature/RPM
- Perfect simulation state: ENU position/velocity, attitude, perfect GPS state, weather components
- Controller internals: target altitude/error, PID terms, common RPM, yaw/pitch/roll mix terms, per-motor RPM references

This file is intended as the primary input for charting and controller troubleshooting.

### `simulation_events.log` format

Each line is:

`local_timestamp,sim_elapsed_s,event_message`

Event messages include:

- simulation start/stop
- config load outcomes
- mission load/start/termination
- mission status transitions
- mission step changes (step id and step name)
