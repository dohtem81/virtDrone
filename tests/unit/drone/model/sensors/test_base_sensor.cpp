#include <catch2/catch_test_macros.hpp>
#include "drone/model/sensors/base_sensor.h"

// Test subclass to implement the pure virtual method
class TestSensor : public BaseSensor {
public:
    TestSensor(const std::string& name, SensorType type)
        : BaseSensor(name, type) {}

    void update() override {
        // Dummy implementation for testing
    }
};

// Test constructor
TEST_CASE("BaseSensor Constructor sets name and type", "[base_sensor]") {
    TestSensor sensor("TestSensor", SensorType::SENSING);
    REQUIRE(sensor.getName() == "TestSensor");
    REQUIRE(sensor.getType() == SensorType::SENSING);
    REQUIRE(sensor.getStatus() == SensorStatus::INACTIVE);  // Default status
}

// Test getters
TEST_CASE("BaseSensor Getters return correct values", "[base_sensor]") {
    TestSensor sensor("TestSensor", SensorType::SENSING);
    REQUIRE(sensor.getName() == "TestSensor");
    REQUIRE(sensor.getType() == SensorType::SENSING);
    REQUIRE(sensor.getStatus() == SensorStatus::INACTIVE);
}

// Test setter
TEST_CASE("BaseSensor SetStatus updates status", "[base_sensor]") {
    TestSensor sensor("TestSensor", SensorType::SENSING);
    sensor.setStatus(SensorStatus::ACTIVE);
    REQUIRE(sensor.getStatus() == SensorStatus::ACTIVE);

    sensor.setStatus(SensorStatus::ERROR);
    REQUIRE(sensor.getStatus() == SensorStatus::ERROR);
}

// Test actuator type
TEST_CASE("BaseSensor Actuator type", "[base_sensor]") {
    TestSensor actuatorSensor("ActuatorSensor", SensorType::ACTUATOR);
    REQUIRE(actuatorSensor.getType() == SensorType::ACTUATOR);
}

// Test sensing type
TEST_CASE("BaseSensor Sensing type", "[base_sensor]") {
    TestSensor sensingSensor("SensingSensor", SensorType::SENSING);
    REQUIRE(sensingSensor.getType() == SensorType::SENSING);
}