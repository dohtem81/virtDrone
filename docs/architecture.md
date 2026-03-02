# Architecture

## Overview

virtDrone uses a split runtime model:

- **Real drone side (`drone/`)**: controller logic and config
- **Simulation side (`simulator/`)**: plant physics and environment updates

This keeps control decisions separate from physics integration.

## Runtime Flow

Each simulation step:

1. **Connection layer** returns sensor data to real-drone logic
   - Connection may inject measurement noise (hardware-like behavior)
2. **RealDrone** computes actuator command from sensed values
3. **Simulation** applies command and advances plant state

## Main Interfaces

- `drone::runtime::SensorSource`
- `drone::runtime::ActuatorSink`
- `drone::runtime::RealDrone`
- `drone::simulator::QuaroSimulation`

## Noise Model Placement

Noise is modeled on the **sim-to-real connection** (sensor transport), not in plant state updates.

- Plant state remains internally consistent for physics
- Real-drone control receives noisy measurements

## Controller Configuration

Controller config is loaded from YAML using drone config classes:

- `include/drone/config/config_base.h` (abstract base)
- `include/drone/config/altitude_controller_config.h` (altitude controller config)

Supported control parameters include:

- `control_param_p`, `control_param_i`, `control_param_d`
- `enable_i_component`, `enable_d_component`
- `activation_error_band_m`

## Logging

Telemetry output includes:

- Sensed vs perfect values (`S/P` pairs)
- Actuator/controller state (`RefRPM`, `TgtErr`, `P`, `I`, `D`)

This supports debugging both plant behavior and control behavior in one run.
