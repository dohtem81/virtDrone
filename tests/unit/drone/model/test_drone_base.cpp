#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "drone/model/drone_base.h"
#include "drone/model/components/elect_motor.h"
#include "drone/model/components/gps_module_base.h"
#include "drone/model/sensors/temperature_sensor.h"
#include "simulator/physics/battery_sim.h"

class GPSensor_test : public drone::model::components::GPSModule_base {
public:
    GPSensor_test(const std::string& name, const drone::model::components::GPSSensorSpecs& specs)
        : GPSModule_base(name, specs) {}

    void update() override {}
};

using namespace drone::model;
using namespace drone::model::components;
using namespace drone::model::sensors;
using namespace drone::simulator::physics;
using Catch::Approx;

TEST_CASE("DroneBase initializes components", "[DroneBase]") {
    ElecMotorSpecs motor_specs(15000.0, 14.8, 20.0, 0.9, 0.4, 0.12);
    AnalogIOSpec motor_io_spec(
        AnalogIOSpec::IODirection::OUTPUT,
        AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0, 10000
    );

    std::vector<ElecMotor> motors;
    motors.emplace_back("Motor1", motor_io_spec, motor_specs);
    motors.emplace_back("Motor2", motor_io_spec, motor_specs);

    CellSpecs cell_specs(1500.0, 4.2);
    auto battery = std::make_unique<BatterySim>("TestBattery", BatterySpecs(4, cell_specs, 0.35));

    auto temp_sensor = std::make_unique<TemperatureSensor>(
        "BodyTemp",
        AnalogIOSpec(
            AnalogIOSpec::IODirection::INPUT,
            AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
            4000, 20000
        ),
        TemperatureSensorRanges(-50.0, 150.0),
        0.02
    );

    auto gps = std::make_unique<GPSensor_test>("GPS", GPSSensorSpecs());

    DroneBase drone("TestDrone", std::move(motors), std::move(battery), std::move(temp_sensor), std::move(gps), 1.2);

    REQUIRE(drone.getName() == "TestDrone");
    REQUIRE(drone.getMotors().size() == 2);
    REQUIRE(drone.getBattery() != nullptr);
    REQUIRE(drone.getTemperatureSensor()->getName() == "BodyTemp");
    REQUIRE(drone.getTotalWeightKg() == Approx(1.2 + 0.35 + 0.02 + 0.05 + (2 * 0.12)));
}

TEST_CASE("DroneBase allows component updates", "[DroneBase]") {
    ElecMotorSpecs motor_specs(10000.0, 12.0, 10.0, 0.85, 0.5, 0.1);
    AnalogIOSpec motor_io_spec(
        AnalogIOSpec::IODirection::OUTPUT,
        AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0, 10000
    );

    std::vector<ElecMotor> motors;
    motors.emplace_back("MotorA", motor_io_spec, motor_specs);

    CellSpecs cell_specs(1200.0, 3.7);
    auto battery = std::make_unique<BatterySim>("BatteryA", BatterySpecs(1, cell_specs, 0.25));

    auto temp_sensor = std::make_unique<TemperatureSensor>(
        "TempA",
        AnalogIOSpec(
            AnalogIOSpec::IODirection::INPUT,
            AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
            4000, 20000
        ),
        TemperatureSensorRanges(-20.0, 80.0),
        0.01
    );

    auto gps = std::make_unique<GPSensor_test>("GPS", GPSSensorSpecs());

    DroneBase drone("DroneA", std::move(motors), std::move(battery), std::move(temp_sensor), std::move(gps), 0.9);

    std::vector<ElecMotor> new_motors;
    new_motors.emplace_back("MotorB", motor_io_spec, motor_specs);
    new_motors.emplace_back("MotorC", motor_io_spec, motor_specs);
    drone.setMotors(std::move(new_motors));

    auto new_battery = std::make_unique<BatterySim>("BatteryB", BatterySpecs(2, cell_specs, 0.3));
    drone.setBattery(std::move(new_battery));

    auto new_temp_sensor = std::make_unique<TemperatureSensor>(
        "TempB",
        AnalogIOSpec(
            AnalogIOSpec::IODirection::INPUT,
            AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
            4000, 20000
        ),
        TemperatureSensorRanges(-10.0, 90.0),
        0.015
    );
    drone.setTemperatureSensor(std::move(new_temp_sensor));

    auto new_gps = std::make_unique<GPSensor_test>("GPS2", GPSSensorSpecs());
    drone.setGPS(std::move(new_gps));

    REQUIRE(drone.getMotors().size() == 2);
    REQUIRE(drone.getBattery() != nullptr);
    REQUIRE(drone.getTemperatureSensor()->getName() == "TempB");
    REQUIRE(drone.getGPS() != nullptr);
}
