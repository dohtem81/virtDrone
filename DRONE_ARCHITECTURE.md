# Drone Architecture

## Overview

The runtime is now explicitly split into two sides:

- **Real drone side (`drone/`)**: control logic and configuration
- **Simulation side (`simulator/`)**: physics and environment evolution

This separation ensures that control code does not directly execute physics internals, and simulator code does not make control decisions.

## Runtime Components

### Real Drone (`drone/runtime`)

**Location:** `include/drone/runtime/real_drone.h`

- `RealDrone`
  - owns `AltitudeController`
  - reads sensor data through `SensorSource`
  - writes motor command through `ActuatorSink`
- `SensorFrame`
  - altitude, battery, motor temperature, motor RPM
- `ActuatorFrame`
  - desired motor RPM command
- `SensorSource` / `ActuatorSink`
  - interface contracts between control and plant

### Simulation (`simulator`)

**Location:** `include/simulator/quadrosimulator.h`, `src/simulator/quadrosimulator.cpp`

- `QuaroSimulation`
  - implements `SensorSource` and `ActuatorSink`
  - stores last actuator command (`desired_rpm_`)
  - advances plant physics in `onStep()`
    - motor dynamics
    - thrust and altitude integration
    - battery drain
    - temperature and GPS updates

## Two-Drone Mental Model

- **Real drone (controller model)**: what firmware would do
  - read sensors
  - compute command
- **Simulated drone (plant model)**: what physics does
  - apply command
  - update physical state

Both are run in the same process in `simulator_app`, but responsibilities are separated by interface boundaries.

## Step Loop

Implemented in `src/simulator/main.cpp`:

1. `real_drone.update(dt, *sim, *sim)`
   - reads `SensorFrame` from simulator
   - computes control output
   - writes `ActuatorFrame` to simulator
2. `sim->step(dt)`
   - applies command and advances physics

This ordering gives a deterministic closed-loop simulation while preserving separation of concerns.

## Configuration

Controller YAML loading is in the drone module:

- `include/drone/config/config_base.h` (abstract base)
- `include/drone/config/altitude_controller_config.h` (derived altitude controller config)

The simulator consumes only configured controller outputs, not config parsing internals.

## Notes

- `Quadrocopter` no longer owns an altitude controller.
- Altitude control execution has been moved out of simulator physics and into `drone/runtime::RealDrone`.
- Existing physical models (RPM, battery, temperature, altitude) remain in `simulator/`.
