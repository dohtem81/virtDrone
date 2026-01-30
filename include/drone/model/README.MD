# virtDrone Model Components

This README documents the key classes, structs, and enums in the `drone::model` namespace, based on unit tests. It provides a list of objects and usage examples derived from test cases.

## Objects Overview

### Classes
- **BaseSensor**: Abstract base class for sensors with analog I/O specs.
- **TemperatureSensor**: Concrete sensor for temperature readings (inherits from BaseSensor).
- **GPSModule_base**: Abstract base class for GPS modules.
- **ElecMotor**: Class for electric motors with specs and I/O.

### Structs
- **AnalogIOSpec**: Defines analog I/O specifications (direction, current range, counts range).
- **GPSSensorSpecs**: Defines GPS sensor specifications (accuracies, rates, max satellites).
- **ElecMotorSpecs**: Defines electric motor specifications (speed, voltage, current, efficiency, thermal resistance).
- **Position3D**: Represents 3D position (latitude, longitude, altitude).
- **Velocity3D**: Represents 3D velocity (north, east, down).
- **TemperatureSensorRanges**: Defines min/max temperature ranges for TemperatureSensor.

### Enums
- **SensorStatus**: Enum for sensor states (e.g., INACTIVE, ACTIVE, ERROR).
- **AnalogIOSpec::IODirection**: Enum for I/O direction (INPUT, OUTPUT).
- **AnalogIOSpec::CurrentRange**: Enum for current ranges (ZERO_TO_10V, PLUS_MINUS_10V, ZERO_TO_20mA, FOUR_TO_20mA).

## Usage Examples

### BaseSensor
Example: Creating and testing a sensor with ZERO_TO_10V range.

```cpp
#include "drone/model/sensors/base_sensor.h"

class TestSensor : public drone::model::sensors::BaseSensor {
public:
    TestSensor(const std::string& name, AnalogIOSpec type) : BaseSensor(name, type) {}
    void update() override { /* implementation */ }
};

TestSensor sensor("TestSensor", AnalogIOSpec(
    AnalogIOSpec::IODirection::INPUT,
    AnalogIOSpec::CurrentRange::ZERO_TO_10V,
    0, 1023
));

REQUIRE(sensor.getName() == "TestSensor");
REQUIRE(sensor.getStatus() == SensorStatus::INACTIVE);
REQUIRE(sensor.setLastCountsReading(512) == true);
REQUIRE(sensor.getLastCountsReading() == 512);
```

### TemperatureSensor
Example: Initializing and setting readings with FOUR_TO_20mA range.

```cpp
#include "drone/model/sensors/temperature_sensor.h"

TemperatureSensor sensor("TempSensor", AnalogIOSpec(
    AnalogIOSpec::IODirection::INPUT,
    AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
    4000, 20000
), TemperatureSensorRanges(-50.0, 150.0));

REQUIRE(sensor.getName() == "TempSensor");
REQUIRE(sensor.setLastCountsReading(4000) == true);  // Min valid
REQUIRE(sensor.getTemperature() == -50.0);
REQUIRE(sensor.setLastCountsReading(20000) == true); // Max valid
REQUIRE(sensor.getTemperature() == 150.0);
REQUIRE(sensor.setLastCountsReading(0) == false);    // Out of range
REQUIRE(sensor.getStatus() == SensorStatus::ERROR);
```

### GPSModule_base
Example: Using a test subclass to set and get GPS data.

```cpp
#include "drone/model/components/gps_module_base.h"

class GPSensor_test : public GPSModule_base {
public:
    GPSensor_test(const std::string& name, const GPSSensorSpecs& specs) : GPSModule_base(name, specs) {}
    void update() override { /* dummy */ }
    using GPSModule_base::setPosition;
    using GPSModule_base::setVelocity;
};

GPSSensorSpecs specs(5.0, 10.0, 0.5, 5, 8);
GPSensor_test gps("TestGPS", specs);

Position3D pos{45.0, -122.0, 100.0};
gps.setPosition(pos);
REQUIRE(gps.getPosition().latitude_deg == 45.0);

Velocity3D vel{10.0, 5.0, -2.0};
gps.setVelocity(vel);
REQUIRE(gps.getVelocity().north_mps == 10.0);

gps.setSatelliteCount(6);
REQUIRE(gps.getSatelliteCount() == 6);
```

### ElecMotor
Example: Initializing and checking motor properties.

```cpp
#include "drone/model/components/elect_motor.h"

ElecMotorSpecs specs(15000.0, 14.8, 20.0, 0.9, 0.4);
AnalogIOSpec io_spec(AnalogIOSpec::IODirection::OUTPUT, AnalogIOSpec::CurrentRange::ZERO_TO_10V, 0, 10000);

ElecMotor motor("TestMotor", io_spec, specs);

REQUIRE(motor.getSpeedRPM() == 0.0);
REQUIRE(motor.getCurrentA() == 0.0);
REQUIRE(motor.getTemperatureC() == 25.0);
REQUIRE(motor.getEfficiency() == 0.9);
```