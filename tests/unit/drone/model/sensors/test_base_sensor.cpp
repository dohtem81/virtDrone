#include <catch2/catch_test_macros.hpp>
#include "drone/model/sensors/base_sensor.h"

using namespace drone::model::sensors;

// Test subclass to implement the pure virtual method
class TestSensor : public drone::model::sensors::BaseSensor {
public:
    TestSensor(const std::string& name, AnalogIOSpec type)
        : BaseSensor(name, type) {}

    void update() override {
        // Dummy implementation for testing
    }
};

// Test constructor for ZERO_TO_10V
TEST_CASE("BaseSensor Constructor with ZERO_TO_10V", "[base_sensor]") {
    TestSensor sensor("TestSensor", AnalogIOSpec(
        AnalogIOSpec::IODirection::INPUT,
        AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0,
        1023
    ));
    REQUIRE(sensor.getName() == "TestSensor");
    REQUIRE(sensor.getStatus() == drone::model::sensors::SensorStatus::INACTIVE);
    REQUIRE(sensor.getType().direction == drone::model::sensors::AnalogIOSpec::IODirection::INPUT);  // Changed to reference access
    REQUIRE(sensor.getType().current_range == drone::model::sensors::AnalogIOSpec::CurrentRange::ZERO_TO_10V);  // Changed to reference access
    REQUIRE(sensor.getType().counts_range.min == 0);
    REQUIRE(sensor.getType().counts_range.max == 1023);
}

// Test constructor for PLUS_MINUS_10V
TEST_CASE("BaseSensor Constructor with PLUS_MINUS_10V", "[base_sensor]") {
    TestSensor sensor("TestSensor", AnalogIOSpec(
        AnalogIOSpec::IODirection::INPUT,
        AnalogIOSpec::CurrentRange::PLUS_MINUS_10V,
        0,
        1023
    ));
    REQUIRE(sensor.getName() == "TestSensor");
    REQUIRE(sensor.getStatus() == drone::model::sensors::SensorStatus::INACTIVE);
    REQUIRE(sensor.getType().direction == drone::model::sensors::AnalogIOSpec::IODirection::INPUT);  // Changed to reference access
    REQUIRE(sensor.getType().current_range == drone::model::sensors::AnalogIOSpec::CurrentRange::PLUS_MINUS_10V);  // Changed to reference access
    REQUIRE(sensor.getType().counts_range.min == 0);
    REQUIRE(sensor.getType().counts_range.max == 1023);
}

// Test constructor for ZERO_TO_20mA
TEST_CASE("BaseSensor Constructor with ZERO_TO_20mA", "[base_sensor]") {
    TestSensor sensor("TestSensor", AnalogIOSpec(
        AnalogIOSpec::IODirection::INPUT,
        AnalogIOSpec::CurrentRange::ZERO_TO_20mA,
        0,
        65000
    ));
    REQUIRE(sensor.getName() == "TestSensor");
    REQUIRE(sensor.getStatus() == drone::model::sensors::SensorStatus::INACTIVE);
    REQUIRE(sensor.getType().direction == drone::model::sensors::AnalogIOSpec::IODirection::INPUT);  // Changed to reference access
    REQUIRE(sensor.getType().current_range == drone::model::sensors::AnalogIOSpec::CurrentRange::ZERO_TO_20mA);  // Changed to reference access
    REQUIRE(sensor.getType().counts_range.min == 0);
    REQUIRE(sensor.getType().counts_range.max == 65000);
}

// Test constructor for FOUR_TO_20mA
TEST_CASE("BaseSensor Constructor with FOUR_TO_20mA", "[base_sensor]") {
    TestSensor sensor("TestSensor", AnalogIOSpec(
        AnalogIOSpec::IODirection::INPUT,
        AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
        4000,
        20000
    ));
    REQUIRE(sensor.getName() == "TestSensor");
    REQUIRE(sensor.getStatus() == drone::model::sensors::SensorStatus::INACTIVE);
    REQUIRE(sensor.getType().direction == drone::model::sensors::AnalogIOSpec::IODirection::INPUT);  // Changed to reference access
    REQUIRE(sensor.getType().current_range == drone::model::sensors::AnalogIOSpec::CurrentRange::FOUR_TO_20mA);  // Changed to reference access
    REQUIRE(sensor.getType().counts_range.min == 4000);
    REQUIRE(sensor.getType().counts_range.max == 20000);
}

// Test setLastCountsReading with valid and invalid inputs
TEST_CASE("BaseSensor setLastCountsReading", "[base_sensor]") {
    TestSensor sensor("TestSensor", AnalogIOSpec(
        drone::model::sensors::AnalogIOSpec::IODirection::INPUT,
        drone::model::sensors::AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0,
        1023
    ));

    // Valid reading
    bool result = sensor.setLastCountsReading(512);
    REQUIRE(result == true);
    REQUIRE(sensor.getLastCountsReading() == 512);
    REQUIRE(sensor.getStatus() == drone::model::sensors::SensorStatus::ACTIVE);

    // Invalid reading (too high)
    result = sensor.setLastCountsReading(2000);
    REQUIRE(result == false);
    REQUIRE(sensor.getStatus() == drone::model::sensors::SensorStatus::ERROR);
    REQUIRE(sensor.getLastCountsReading() == 2000); // Should max out

    // Invalid reading (too low, using max value for uint64_t as approximation)
    result = sensor.setLastCountsReading(0);
    REQUIRE(result == true);
    REQUIRE(sensor.getStatus() == drone::model::sensors::SensorStatus::ACTIVE);
    REQUIRE(sensor.getLastCountsReading() == 0); // Should min out
}
