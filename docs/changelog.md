# Changelog

This changelog tracks major architecture, runtime, and documentation updates.

## Entry policy
- Use the `YYYY-MM-DD` date format for each release/update block.
- Group changes under short headings (for example: runtime, controller, energy, tooling, docs).
- Keep entries concise and user-visible (avoid internal-only refactor noise unless it changes behavior).
- Add newest entries at the top.

## 2026-03-04

### Position hold behavior and config
- Enabled XY position hold by default outside mission mode, with auto-latched current XY reference for local hover drift suppression.
- Added mission hover-step XY hold latching behavior to keep local XY stable while tracking hover altitude/yaw references.
- Exposed and documented position-hold YAML tuning fields: `position_hold_enabled`, `position_hold_kp_pos`, `position_hold_kp_vel`, `position_hold_kd_vel`, `position_hold_max_velocity_mps`, `position_hold_max_tilt_rad`.
- Synchronized README and docs pages so architecture/usage/state summaries match the new default-hold behavior.

### Mission module consistency
- Moved mission domain module from `simulator/mission` paths to `drone/mission` paths for architecture consistency with RealDrone mission ownership.
- Updated includes, namespaces, build wiring, and mission tests to use the new drone-side mission module location.

### Logging and charting
- Added `sim_elapsed_s` to each `simulation_events.log` line (`local_timestamp,sim_elapsed_s,event_message`).
- Added `tools/scripts/generate_mission_chart.py` to plot XYZ positions vs references and mission sequence status/step timeline.
- Extended mission chart to include per-motor RPM reference traces (`desired_motor_rpm_0..3`) in an additional subplot.
- Changed simulator log output location to `docs/tutorials/simulation_telemetry.csv` and `docs/tutorials/simulation_events.log`.
- Added optional `logs_dir` CLI parameter to override simulator log output directory.
- Added usage documentation for mission chart generation from `simulation_telemetry.csv` and `simulation_events.log`.

## 2026-03-03

### Mission runtime and schema
- Added mission YAML schema support with loader, typed mission actions, and completion criteria parsing.
- Added mission execution engine with step sequencing, time-based/completion-based advancement, and timeout behavior handling.
- Integrated mission lifecycle into `RealDrone` runtime and `simulator_app` optional CLI mission argument.

### Control integration
- Added `PositionController` and integrated it into `RealDrone` as XY target tracking that outputs pitch/roll references.
- Extended runtime sensor/actuator framing to carry ENU position values needed for mission completion and position control.

### Tests
- Added force-dynamics tests covering yaw/pitch/roll force projection and XYZ movement behavior.
- Added position-controller unit tests.
- Added mission loader unit tests (valid parse, invalid schema, unknown condition fallback).
- Added mission executor integration tests (time->completion transitions, position-reached completion behavior).

### Documentation and examples
- Added mission format reference doc and mission YAML examples (`hover_and_move`, `rectangle_patrol`).
- Updated architecture, usage, current-state, roadmap, and index documentation to reflect mission and controller changes.

### Validation notes
- Targeted Docker builds/tests for new mission and control targets pass.
- Full-repo test runs still include pre-existing altitude-controller test API mismatch outside this change set.

## 2026-03-02

### Dynamics and control pipeline
- Added ENU translational force integration with yaw/pitch/roll thrust projection.
- Added common + differential (`yaw/pitch/roll`) motor mixing path and related telemetry.

### GPS and sensing
- Added perfect ENU-to-geodetic GPS propagation in simulation.
- Added GPS noise injection on the sim-to-real sensor connection path.
- Extended telemetry with sensed/perfect GPS position and velocity pairs.

### Weather and constraints
- Added configurable weather model (steady + gust + turbulence) and runtime integration.
- Added weather telemetry fields (`WTotAcc`, `WSteady`, `WGust`, `WTurb`).
- Added ground-lock behavior to prevent movement while altitude is clamped to ground.

### Tests
- Added coverage for force dynamics, mixer logic, GPS mapping, GPS noise, weather model/config, and ground-lock behavior.

### Tooling and charts
- Extended chart parser and full dashboard for optional weather telemetry fields.
- Added UTF-8/UTF-16 decoding fallback in parser.
- Updated full dashboard energy/temperature panel to dual-axis scaling.
- Added dedicated Docker Compose `chart-test` workflow for parser tests.

### Runtime and architecture
- Separated real-drone control logic from simulation plant physics.
- Moved sensor noise modeling to the sim-to-real connection path.
- Added connection adapter for noisy sensor transport.
- Kept simulation physics updates on perfect internal plant state.

### Controller and telemetry
- Added PID-related telemetry in logs (target error, P, I, D).
- Added target altitude and sensed/perfect paired telemetry fields.
- Added configurable PID activation behavior with error-band gating.

### Energy and motor behavior
- Enforced battery depletion cutoff: depleted battery drives RPM to zero.
- Coupled max achievable RPM to available battery voltage.

### Tooling and charts
- Added Docker compose chart workflow.
- Updated chart generator to parse current telemetry format.
- Added full and minimal chart modes.
- Set default chart output path under docs/tutorials/charts.

### Documentation
- Reorganized root documentation to docs/ pages.
- Added dedicated pages for architecture, usage, current state, and roadmap.
