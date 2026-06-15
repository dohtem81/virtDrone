# Simulation Mission YAML Format

This document defines the mission YAML schema consumed by the control side mission runtime.

## Ownership

- Mission parsing and execution run on the control side (`RealDrone` + `MissionExecutor`).
- The simulator does not parse mission YAML directly.

## Top-Level Shape

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

## Step Fields

- `step_id` (int, required)
- `name` (string, required)
- `action` (string, required)
- `enabled` (bool, optional, default `true`)
- `advance_mode` (`time_based` or `completion_based`)
- `duration_s` (required for `time_based`)
- `timeout_s` (optional, default `60.0`)
- `on_timeout` (`abort`, `proceed`, `retry`)
- `retry_count` (used with `retry`)
- `fallback_step_id` (reserved)
- `completion_criteria` (used for `completion_based`)

## Actions

- `hover`
- `go_to_position`
- `land`
- `set_attitude`
- `change_altitude`
- `rotate_yaw`

## Completion Criteria

Supported `condition_type` values:

- `time_elapsed`
- `altitude_reached`
- `altitude_and_velocity`
- `position_reached`
- `yaw_reached`
- `attitude_reached`
- `velocity_low`
- `landed`

Current parser behavior for unknown values:

- falls back to `time_elapsed`

## Runtime Behavior

Per control tick:

1. Mission step updates control targets.
2. Controller computes actuator output.
3. Output is sent through `SimulationGateway`.
4. Simulation advances one step.

## CLI

`simulator_app [weather_config_file] [logs_dir] [listen_address]`

`drone_app [backend] [simulator_address] [steps] [dt_s] [altitude_config_file] [attitude_config_file] [mission_file] [logs_dir]`

Example Docker run pair:

```bash
docker compose run --rm dev bash -lc '/workspace/build/simulator_app config/weather.yaml docs/tutorials 0.0.0.0:50051'
docker compose run --rm dev bash -lc '/workspace/build/drone_app grpc localhost:50051 1200 0.02 config/altitude_controller.yaml config/attitude_controller.yaml config/missions/hover_and_move.yaml docs/tutorials'
```

## Logs

Mission runs emit:

- simulator telemetry: `docs/tutorials/simulation_telemetry.csv`
- simulator events: `docs/tutorials/simulation_events.log`
- control events: `docs/tutorials/control_events.log`