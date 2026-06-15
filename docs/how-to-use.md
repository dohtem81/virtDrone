# How to Use

## Supported Execution Model

Use Docker commands only.

All build, run, and test examples below execute through `docker compose`.

## Build

```bash
docker compose run --rm dev cmake -S /workspace -B /workspace/build -G Ninja -DCMAKE_BUILD_TYPE=Debug
docker compose run --rm dev cmake --build /workspace/build -j
```

## Run Split Runtime

Start simulation side (server):

```bash
docker compose run --rm dev bash -lc '/workspace/build/simulator_app config/weather.yaml docs/tutorials 0.0.0.0:50051'
```

Start control side (client):

```bash
docker compose run --rm dev bash -lc '/workspace/build/drone_app grpc localhost:50051 10000 0.01 config/altitude_controller.yaml config/attitude_controller.yaml config/missions/hover_and_move.yaml docs/tutorials'
```

## CLI Signatures

`simulator_app [weather_config_file] [logs_dir] [listen_address]`

`drone_app [backend] [simulator_address] [steps] [dt_s] [altitude_config_file] [attitude_config_file] [mission_file] [logs_dir]`

Backend values:

- `grpc` / `sim` / `simulation`: implemented
- `hardware`: reserved placeholder, not implemented yet

## Hardware Backend Activation Plan (When Implemented)

Use this checklist once `hardware` backend support is added:

1. Implement and register the hardware gateway in `CreateControlGateway(...)` for `ControlBackend::Hardware`.
2. Provide hardware-side sensor/actuator adapter configuration (device endpoint, auth, timing) via runtime config.
3. Keep `drone_app` CLI unchanged and switch backend argument to `hardware`.
4. Run a Docker smoke test with short duration first:

```bash
docker compose run --rm dev bash -lc '/workspace/build/drone_app hardware <hardware-endpoint> 300 0.01 config/altitude_controller.yaml config/attitude_controller.yaml config/missions/hover_and_move.yaml docs/tutorials'
```

5. Confirm output files are produced in `docs/tutorials/` and that control-session lifecycle events are present in `control_events.log`.

## Validation Run

End-to-end build + tests with one command:

```bash
docker compose run --rm dev bash -lc '/workspace/tools/scripts/refactor-closeout-docker.sh'
```

This command configures/builds in `/tmp/virtdrone-build` and runs `ctest --output-on-failure`.

Validation summary report:

- `reports/refactor-validation-summary.html`

## Test Commands

Run all tests:

```bash
docker compose run --rm dev bash -lc 'ctest --test-dir /workspace/build --output-on-failure'
```

Run targeted mission tests:

```bash
docker compose run --rm dev bash -lc "ctest --test-dir /workspace/build -R 'test_mission_loader$|test_mission_executor_transitions$' --output-on-failure"
```

## Logs and Artifacts

Default runtime artifact folder:

- `docs/tutorials/`

Produced files:

- `docs/tutorials/simulation_telemetry.csv`
- `docs/tutorials/simulation_events.log`
- `docs/tutorials/control_events.log`
- `reports/refactor-validation-summary.html`

Charts:

- `docs/tutorials/charts/flight_dashboard.png`
- `docs/tutorials/charts/mission_xyz_status.png`

## Chart Generation

```bash
docker compose run --rm chart
```

Chart parser tests:

```bash
docker compose run --rm chart-test
```

## Mission Configuration

Mission examples:

- `config/missions/hover_and_move.yaml`
- `config/missions/hover_and_land.yaml`

Controller config files:

- `config/altitude_controller.yaml`
- `config/attitude_controller.yaml`

Position-hold tuning keys are defined in altitude config YAML:

- `position_hold_enabled`
- `position_hold_kp_pos`
- `position_hold_kp_vel`
- `position_hold_kd_vel`
- `position_hold_max_velocity_mps`
- `position_hold_max_tilt_rad`