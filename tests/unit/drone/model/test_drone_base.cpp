#include <catch2/catch_test_macros.hpp>

#include "drone/model/drone_base.h"
#include "drone/model/components/elect_motor.h"
#include "drone/model/sensors/temperature_sensor.h"
#include "simulator/physics/battery_sim.h"

using namespace drone::model;
using namespace drone::model::components;
using namespace drone::model::sensors;
using namespace drone::simulator::physics;

TEST_CASE("DroneBase initializes components", "[DroneBase]") {
    ElecMotorSpecs motor_specs(15000.0, 14.8, 20.0, 0.9, 0.4);
    AnalogIOSpec motor_io_spec(
        AnalogIOSpec::IODirection::OUTPUT,
        AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0, 10000
    );

    std::vector<ElecMotor> motors;
    motors.emplace_back("Motor1", motor_io_spec, motor_specs);
    motors.emplace_back("Motor2", motor_io_spec, motor_specs);

    CellSpecs cell_specs(1500.0, 4.2);
    auto battery = std::make_unique<BatterySim>("TestBattery", BatterySpecs(4, cell_specs));

    auto temp_sensor = std::make_unique<TemperatureSensor>(
        "BodyTemp",
        AnalogIOSpec(
            AnalogIOSpec::IODirection::INPUT,
            AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
            4000, 20000
        ),
        TemperatureSensorRanges(-50.0, 150.0)
    );

    DroneBase drone("TestDrone", std::move(motors), std::move(battery), std::move(temp_sensor));

    REQUIRE(drone.getName() == "TestDrone");
    REQUIRE(drone.getMotors().size() == 2);
    REQUIRE(drone.getBattery() != nullptr);
    REQUIRE(drone.getTemperatureSensor()->getName() == "BodyTemp");
}

TEST_CASE("DroneBase allows component updates", "[DroneBase]") {
    ElecMotorSpecs motor_specs(10000.0, 12.0, 10.0, 0.85, 0.5);
    AnalogIOSpec motor_io_spec(
        AnalogIOSpec::IODirection::OUTPUT,
        AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0, 10000
    );

    std::vector<ElecMotor> motors;
    motors.emplace_back("MotorA", motor_io_spec, motor_specs);

    CellSpecs cell_specs(1200.0, 3.7);
    auto battery = std::make_unique<BatterySim>("BatteryA", BatterySpecs(1, cell_specs));

    auto temp_sensor = std::make_unique<TemperatureSensor>(
        "TempA",
        AnalogIOSpec(
            AnalogIOSpec::IODirection::INPUT,
            AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
            4000, 20000
        ),
        TemperatureSensorRanges(-20.0, 80.0)
    );

    DroneBase drone("DroneA", std::move(motors), std::move(battery), std::move(temp_sensor));

    std::vector<ElecMotor> new_motors;
    new_motors.emplace_back("MotorB", motor_io_spec, motor_specs);
    new_motors.emplace_back("MotorC", motor_io_spec, motor_specs);
    drone.setMotors(std::move(new_motors));

    auto new_battery = std::make_unique<BatterySim>("BatteryB", BatterySpecs(2, cell_specs));
    drone.setBattery(std::move(new_battery));

    auto new_temp_sensor = std::make_unique<TemperatureSensor>(
        "TempB",
        AnalogIOSpec(
            AnalogIOSpec::IODirection::INPUT,
            AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
            4000, 20000
        ),
        TemperatureSensorRanges(-10.0, 90.0)
    );
    drone.setTemperatureSensor(std::move(new_temp_sensor));

    REQUIRE(drone.getMotors().size() == 2);
    REQUIRE(drone.getBattery() != nullptr);
    REQUIRE(drone.getTemperatureSensor()->getName() == "TempB");
}
