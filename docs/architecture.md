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

## Coordinate and Attitude Conventions

Dynamics are now modeled in **ENU Cartesian coordinates**:

- `x`: East [m]
- `y`: North [m]
- `z`: Up [m]

Attitude is represented as Euler angles in radians:

- `yaw_rad`
- `pitch_rad`
- `roll_rad`

Rotation used for force projection is body-to-world with Z-Y-X composition (yaw -> pitch -> roll).

## Force-Vector Integration

At each simulation step:

1. Total motor thrust is accumulated in body frame (`[0, 0, thrust]`)
2. Thrust is rotated to ENU using yaw/pitch/roll
3. Gravity is applied as `[0, 0, -m*g]`
4. Linear damping is applied opposite ENU velocity
5. Net force is integrated to ENU acceleration, velocity, and position

Altitude compatibility is preserved by mirroring `altitude_m = z` for existing altitude-focused runtime/controller paths.

## Command Decomposition and Mixing

Actuation now uses a two-part command model:

1. **Common reference** (`common_motor_rpm`) for shared lift demand
2. **Differential terms** (`yaw_control_rpm`, `pitch_control_rpm`, `roll_control_rpm`) for attitude shaping

The final per-motor setpoints are mixed from common + differential terms (X-frame convention), then saturated while preserving common reference first.

Backward compatibility is preserved by keeping scalar `desired_motor_rpm`; when per-motor references are absent, simulator falls back to equal RPM on all motors.

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
- Appended ENU/YPR state (`PosENU`, `VelENU`, `YPR`)
- Mixer state (`ComRPM`, `MixYPR`, `MRef`)

This supports debugging both plant behavior and control behavior in one run.
