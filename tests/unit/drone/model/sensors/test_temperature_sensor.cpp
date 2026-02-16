#include <catch2/catch_test_macros.hpp>
#include "drone/model/sensors/temperature_sensor.h"

using namespace drone::model::sensors;

TemperatureSensor sensor("TempSensor", AnalogIOSpec(
    AnalogIOSpec::IODirection::INPUT,
    AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
        4000,
        20000), 
    TemperatureSensorRanges(-50.0, 150.0),
    0.02);

// Test constructor for FOUR_TO_20mA
TEST_CASE("TemperatureSensor Constructor with FOUR_TO_20mA", "[temperature_sensor]") {
    REQUIRE(sensor.getName() == "TempSensor");
    REQUIRE(sensor.getStatus() == SensorStatus::INACTIVE);
    REQUIRE(sensor.getType().direction == AnalogIOSpec::IODirection::INPUT);
    REQUIRE(sensor.getType().current_range == AnalogIOSpec::CurrentRange::FOUR_TO_20mA);
    REQUIRE(sensor.getType().counts_range.min == 4000);
    REQUIRE(sensor.getType().counts_range.max == 20000);
    REQUIRE(sensor.getRanges()->min_temperature == -50.0);
    REQUIRE(sensor.getRanges()->max_temperature == 150.0);
}

// Test setLastCountsReading with out-of-range values (below min)
TEST_CASE("TemperatureSensor setLastCountsReading below min range", "[temperature_sensor]") {
    bool result = sensor.setLastCountsReading(0);
    REQUIRE(result == false);
    REQUIRE(sensor.getLastCountsReading() == 0);
    REQUIRE(sensor.getStatus() == SensorStatus::ERROR);
    REQUIRE(sensor.getTemperature() == -50.0); // Should clamp to min temperature
}

// Test setLastCountsReading with out-of-range values (above max)
TEST_CASE("TemperatureSensor setLastCountsReading above max range", "[temperature_sensor]") {
    bool result = sensor.setLastCountsReading(22000);
    REQUIRE(result == false);
    REQUIRE(sensor.getLastCountsReading() == 22000);
    REQUIRE(sensor.getStatus() == SensorStatus::ERROR);
    REQUIRE(sensor.getTemperature() == 150.0); // Should clamp to max temperature
}

// Test setLastCountsReading with valid min value
TEST_CASE("TemperatureSensor setLastCountsReading at min valid range", "[temperature_sensor]") {
    bool result = sensor.setLastCountsReading(4000);
    REQUIRE(result == true);
    REQUIRE(sensor.getLastCountsReading() == 4000);
    REQUIRE(sensor.getStatus() == SensorStatus::ACTIVE);
    REQUIRE(sensor.getTemperature() == -50.0); // Should map to min temperature
}

// Test setLastCountsReading with valid mid value
TEST_CASE("TemperatureSensor setLastCountsReading at mid valid range", "[temperature_sensor]") {
    bool result = sensor.setLastCountsReading(10000);
    REQUIRE(result == true);
    REQUIRE(sensor.getLastCountsReading() == 10000);
    REQUIRE(sensor.getStatus() == SensorStatus::ACTIVE);
    REQUIRE(sensor.getTemperature() >= -50.0);
    REQUIRE(sensor.getTemperature() <= 150.0);
}

// Test setLastCountsReading with valid max value
TEST_CASE("TemperatureSensor setLastCountsReading at max valid range", "[temperature_sensor]") {
    bool result = sensor.setLastCountsReading(20000);
    REQUIRE(result == true);
    REQUIRE(sensor.getLastCountsReading() == 20000);
    REQUIRE(sensor.getStatus() == SensorStatus::ACTIVE);
    REQUIRE(sensor.getTemperature() == 150.0); // Should map to max temperature
}

