# How to Use

## Build and Run

### Docker (recommended)

```bash
docker compose run --rm dev cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
docker compose run --rm dev cmake --build build --target simulator_app
docker compose run --rm dev ./build/simulator_app 10000 0.01
```

### Host

```bash
cmake -S . -B build
cmake --build build --target simulator_app
./build/simulator_app 1000 0.01
```

## Configuration

Default controller configuration:

- `config/altitude_controller.yaml`

Aggressive alternative:

- `config/altitude_controller_fast.yaml`

In addition to altitude PID fields, controller YAML now supports position-hold tuning fields:

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

Default runtime behavior keeps XY hold enabled even without a mission file; the current XY is latched as the hold reference during free-flight hover.

Current controller status (important):

- Position hold is available but still not reliable enough in all scenarios (especially takeoff/hover under disturbance/noisy sensing).
- Altitude controller behavior still needs further tuning/rework for robust mission behavior.
- Use the tutorial logs/charts below to evaluate behavior on your run before relying on defaults.

Run with explicit config file:

```bash
./build/simulator_app 1000 0.01 config/altitude_controller_fast.yaml
```

Run with explicit altitude + weather config files:

```bash
./build/simulator_app 1000 0.01 config/altitude_controller.yaml config/weather.yaml
```

Run with explicit altitude + weather + mission files:

```bash
./build/simulator_app 1000 0.01 config/altitude_controller.yaml config/weather.yaml config/missions/hover_and_move.yaml
```

Run with explicit log output directory parameter:

```bash
./build/simulator_app 1000 0.01 config/altitude_controller.yaml config/weather.yaml config/missions/hover_and_move.yaml docs/tutorials
```

Docker mission run from repository root:

```bash
docker compose run --rm dev bash -lc "cd /workspace/build && ./simulator_app 1000 0.01 config/altitude_controller.yaml config/weather.yaml ../config/missions/hover_and_move.yaml"
```

Docker mission run with explicit logs directory:

```bash
docker compose run --rm dev bash -lc "cd /workspace/build && ./simulator_app 10000 0.01 ../config/altitude_controller.yaml ../config/attitude_controller.yaml ../config/weather.yaml ../config/missions/hover_and_land.yaml ../docs/tutorials/"
```

Mission examples:

- `config/missions/hover_and_move.yaml`
- `config/missions/rectangle_patrol.yaml`

## Tests

Run all tests:

```bash
ctest --test-dir build --output-on-failure
```

Run altitude-controller-related tests:

```bash
ctest --test-dir build -R alt_ctrl --output-on-failure
```

Run mission loader and mission step transition tests:

```bash
docker compose run --rm dev bash -lc "cd /workspace/build; cmake --build . --target test_mission_loader test_mission_executor_transitions -j; ctest -R 'test_mission_loader$|test_mission_executor_transitions$' --output-on-failure"
```

## Runtime logs

The simulator writes logs to files (instead of console telemetry output):

- `docs/tutorials/simulation_telemetry.csv`: timestamped per-step telemetry for charting and troubleshooting.
- `docs/tutorials/simulation_events.log`: timestamped lifecycle and mission events (start, mission status, mission step changes, stop).

Files are written to the repository tutorial folder whether `simulator_app` is launched from repository root or from `build/`.

You can override log location by passing the optional `logs_dir` argument:

`simulator_app [steps] [dt_s] [altitude_config_file] [weather_config_file] [mission_file] [logs_dir]`

Tutorial artifacts to inspect after each run:

- Logs: `docs/tutorials/simulation_telemetry.csv`, `docs/tutorials/simulation_events.log`
- Charts: `docs/tutorials/charts/flight_dashboard.png`, `docs/tutorials/charts/mission_xyz_status.png`

## Chart Generation

Recommended workflow (ensures UTF-8 simulator log for chart parser):

```bash
docker compose run --rm dev sh -lc './build/simulator_app 1000 0.01 > test.txt'
docker compose run --rm chart
```

Generate default full dashboard chart:

```bash
docker compose run --rm chart
```

Default output file:

- `docs/tutorials/charts/flight_dashboard.png`

Generate custom full dashboard:

```bash
docker compose run --rm chart sh -c "pip install --no-cache-dir pandas matplotlib && python tools/scripts/generate_chart.py --input test.txt --mode full --output docs/tutorials/charts/my_run_full.png"
```

Generate minimal chart (sensors + altitude + controller state):

```bash
docker compose run --rm chart sh -c "pip install --no-cache-dir pandas matplotlib && python tools/scripts/generate_chart.py --input test.txt --mode minimal --output docs/tutorials/charts/my_run_minimal.png"
```

Run chart parser tests:

```bash
docker compose run --rm chart-test
```

Generate mission chart (XYZ/YPR actual vs references + mission status/step + motor RPMs):

```bash
docker compose run --rm chart sh -c "pip install --no-cache-dir pandas matplotlib pyyaml && python tools/scripts/generate_mission_chart.py --telemetry docs/tutorials/simulation_telemetry.csv --events docs/tutorials/simulation_events.log --mission config/missions/hover_and_move.yaml --output docs/tutorials/charts/mission_xyz_status.png"
```

This chart overlays:

- X/Y/Z position vs mission references
- yaw/pitch/roll attitude vs mission references
- Mission step id over simulation time
- Mission sequence status (`IDLE`, `RUNNING`, `PAUSED`, `COMPLETED`, `ABORTED`, `FAILED`)
- Per-motor RPM references (`M0..M3`)

Telemetry now also appends mixer and ENU/attitude fields (`ComRPM`, `MixYPR`, `MRef`, `PosENU`, `VelENU`, `YPR`) while preserving existing chart-compatible core fields.

Telemetry now also includes:

- sensed/perfect GPS position + velocity pairs
- weather terms (`WTotAcc`, `WSteady`, `WGust`, `WTurb`)

Full dashboard currently includes a dual-axis energy/temperature panel and weather plots.
