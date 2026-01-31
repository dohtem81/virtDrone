#include <catch2/catch_test_macros.hpp>
#include "simulator/physics/motor_physics.h"
// #include "simulator/physics/physics.h"
#include "drone/model/components/elect_motor.h"

using namespace drone::model::components;
using namespace drone::model::sensors;
using namespace drone::simulator::physics;

ElecMotorSpecs specs(
    15000.0,    // max_speed_rpm
    14.8,       // nominal_voltage_v
    20.0,       // max_current_a
     0.9,        // efficiency
     0.4         // thermal_resistance
);
AnalogIOSpec io_spec(
    AnalogIOSpec::IODirection::OUTPUT,
    AnalogIOSpec::CurrentRange::ZERO_TO_10V,
    0, 10000
);

// Test MotorPhysics simulation logic
TEST_CASE("MotorPhysics updates state correctly", "[MotorPhysics]") {
    ElecMotor motor("TestMotor", io_spec, specs);
    // Simulate multiple updates to allow speed ramping
    REQUIRE(motor.getSpeedRPM() == 0.0);
    REQUIRE(motor.getCurrentA() == 0.0);
    REQUIRE(motor.getTemperatureC() == 25.0);
    motor.setDesiredSpeedRPM(5000.0); // Set desired speed
    MotorPhysics::updateMotorPhysics(motor, static_cast<uint64_t>(1000));
    REQUIRE(motor.getSpeedRPM() == 1000.0);
    REQUIRE(motor.getCurrentA() > 0.0);
    REQUIRE(motor.getTemperatureC() >= 25.0); // Temperature should not decrease
    MotorPhysics::updateMotorPhysics(motor, static_cast<uint64_t>(4000));
    REQUIRE(motor.getSpeedRPM() == 5000.0);
    REQUIRE(motor.getCurrentA() > 0.0);
    REQUIRE(motor.getTemperatureC() >= 25.0); // Temperature should not decrease
}

// Test temperature update over time using internal temperature
TEST_CASE("MotorPhysics temperature updates over time", "[MotorPhysics]") {
    ElecMotor motor("TestMotor", io_spec, specs);
    motor.setDesiredSpeedRPM(4000.0); // Set high desired speed to generate heat
    MotorPhysics::updateMotorPhysics(motor, static_cast<uint64_t>(30000)); // Update for 30 seconds
    double temp_after_30s = motor.getTemperatureC();
    REQUIRE(temp_after_30s > 25.0);
    REQUIRE(temp_after_30s > 28.0); // Should not exceed reasonable limits
}

// Test temperature update over time using internal temperature sensor reading
TEST_CASE("MotorPhysics internal temperature sensor reading", "[MotorPhysics]") {
    ElecMotor motor("TestMotor", io_spec, specs);
    motor.setDesiredSpeedRPM(4000.0); // Set high desired speed to generate heat
    MotorPhysics::updateMotorPhysics(motor, static_cast<uint64_t>(30000)); // Update for 30 seconds
    TemperatureSensorReading temp_reading = motor.getTemperatureReading();
    REQUIRE(temp_reading.temperature > 25.0);
    REQUIRE(temp_reading.status == SensorStatus::ACTIVE);
}