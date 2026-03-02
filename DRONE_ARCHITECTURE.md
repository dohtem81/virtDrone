# Drone Architecture Refactoring

## Overview

The drone model architecture has been refactored to separate concerns between **simulation perspective** and **drone perspective**.

## Class Hierarchy

```
DronePhysical (Sensor Data Interface)
    └── DroneBase (Simulator with Physics Calculations)
            └── Quadrocopter (Concrete Quadrocopter Implementation)
```

## Class Descriptions

### DronePhysical
**Purpose:** Represents the drone from the drone's own perspective  
**Location:** `include/drone/model/drone_physical.h`

This is the "drone sensor interface" - it provides **only** the data that actual drone sensors can measure:
- **Battery:** voltage, state of charge
- **Temperature:** sensor reading in Celsius
- **GPS:** position and altitude
- **Name/ID:** drone identifier

This class is read-only from a sensor perspective - the drone sees these endpoints and can only read sensor values through them.

### DroneBase
**Purpose:** Simulator class with full physics calculations  
**Location:** `include/drone/model/drone_base.h`

This extends `DronePhysical` and adds:
- **Motor Management:** Owns and manages all electric motors
- **Physics Calculations:** 
  - Total weight calculation (body + motors + components)
  - Component weight aggregation
- **Simulation Lifecycle:** Can update/replace components for testing
- **Public APIs:** Component setters for simulation scenarios

This is the "simulator view" - where all physical calculations happen and components are managed.

### Quadrocopter
**Purpose:** Concrete quadrocopter drone implementation  
**Location:** `include/drone/model/quadrocopter.h`

Inherits from `DroneBase` and adds:
- **Altitude Controller:** Specializes the base drone with altitude control logic
- **Factory Methods:** `createWithBatterySim()` - convenience constructor with simulated battery/GPS
- **Motor Count:** Enforces exactly 4 motors

## Data Flow

### Drone Firmware/Software Perspective
```
Drone Firmware
    ↓
DronePhysical (reads sensor data)
    ├─ getBatteryVoltageV()
    ├─ getTemperatureC()
    ├─ getAltitudeM()
    └─ getGPS()
```

### Simulator Perspective
```
Flight Simulator
    ↓
DroneBase (manages components and calculations)
    ├─ Motors (getMotors/setMotors)
    ├─ Physics (getTotalWeightKg())
    ├─ Battery Management (setBattery/getBattery)
    └─ Sensor Management (setTemperatureSensor/setGPS)
```

## Backward Compatibility

- Existing code using `DroneBase` continues to work unchanged
- `Quadrocopter` inherits all functionality from `DroneBase`
- Tests remain compatible and pass without modification

## Usage Examples

### From Simulator (Physics/Calculation)
```cpp
DroneBase drone = Quadrocopter::createWithBatterySim(...);
double total_weight = drone.getTotalWeightKg();  // Physics calculation
drone.getMotors()[0].setThrottle(0.5);  // Motor control
```

### From Drone Perspective (Sensor Readings)
```cpp
DronePhysical* sensor_view = &drone;
double altitude = sensor_view->getAltitudeM();     // GPS reading
double voltage = sensor_view->getBatteryVoltageV(); // Battery reading
double temp = sensor_view->getTemperatureC();      // Thermal reading
```

## Benefits

1. **Clear Separation of Concerns:** Simulation logic separate from sensor interface
2. **Extensibility:** Easy to add more drone types extending `DroneBase`
3. **Realism:** Drone software only sees what sensors provide
4. **Testability:** Can mock `DronePhysical` for pure firmware testing
5. **Documentation:** Class names clearly indicate their purpose
