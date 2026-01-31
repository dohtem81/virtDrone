#include <catch2/catch_test_macros.hpp>
#include "drone/model/components/elect_motor.h"

using namespace drone::model::components;
using namespace drone::model::sensors;

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

// test cases for ElecMotor class
TEST_CASE("ElecMotor initializes correctly", "[ElecMotor]") {
    ElecMotor motor("TestMotor", io_spec, specs);

    REQUIRE(motor.getSpeedRPM() == 0.0);
    REQUIRE(motor.getCurrentA() == 0.0);
    REQUIRE(motor.getTemperatureC() == 25.0); // Initial temperature
    REQUIRE(motor.getEfficiency() == specs.efficiency);
}