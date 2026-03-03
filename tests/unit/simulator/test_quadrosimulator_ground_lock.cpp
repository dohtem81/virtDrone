#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "simulator/quadrosimulator.h"

namespace {

drone::model::components::ElecMotorSpecs makeMotorSpecs() {
    return drone::model::components::ElecMotorSpecs(15000.0, 14.8, 20.0, 0.9, 0.4, 0.12);
}

drone::model::sensors::AnalogIOSpec makeMotorIOSpec() {
    return drone::model::sensors::AnalogIOSpec(
        drone::model::sensors::AnalogIOSpec::IODirection::OUTPUT,
        drone::model::sensors::AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0,
        10000);
}

drone::model::components::BatterySpecs makeBatterySpecs() {
    const drone::model::components::CellSpecs cell_specs(1500.0, 4.2);
    return drone::model::components::BatterySpecs(4, cell_specs, 0.35);
}

drone::model::sensors::AnalogIOSpec makeTempIOSpec() {
    return drone::model::sensors::AnalogIOSpec(
        drone::model::sensors::AnalogIOSpec::IODirection::INPUT,
        drone::model::sensors::AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
        4000,
        20000);
}

drone::model::sensors::TemperatureSensorRanges makeTempRanges() {
    return drone::model::sensors::TemperatureSensorRanges(-50.0, 150.0);
}

std::shared_ptr<drone::simulator::QuaroSimulation> makeSimulation() {
    return drone::simulator::QuadroSimulationFactory(
        "QuadTest",
        makeMotorSpecs(),
        makeMotorIOSpec(),
        makeBatterySpecs(),
        makeTempIOSpec(),
        makeTempRanges(),
        0.02,
        drone::model::components::GPSSensorSpecs(),
        1.2,
        0.3,
        1.0,
        100,
        0.01);
}

}  // namespace

TEST_CASE("QuaroSimulation does not move while grounded under weather disturbance", "[QuaroSimulation][GroundLock]") {
    auto sim = makeSimulation();

    drone::simulator::config::WeatherConfig weather_config;
    weather_config.enabled = true;
    weather_config.steady_accel_enu_ms2 = drone::Vector3(10.0, -5.0, 0.0);
    weather_config.gust_amplitude_enu_ms2 = drone::Vector3(2.0, 2.0, 0.0);
    weather_config.turbulence_std_enu_ms2 = drone::Vector3(0.5, 0.5, 0.0);
    weather_config.random_seed = 123;

    sim->setWeatherConfig(weather_config);
    sim->start();

    for (int i = 0; i < 120; ++i) {
        sim->step(0.01);
    }

    const auto sensors = sim->readSensors();
    sim->stop();

    REQUIRE(sensors.altitude_m == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sensors.gps_altitude_m == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sensors.gps_latitude_deg == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sensors.gps_longitude_deg == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sensors.gps_velocity_east_mps == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sensors.gps_velocity_north_mps == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sensors.gps_velocity_down_mps == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("QuaroSimulation can move after liftoff while weather is enabled", "[QuaroSimulation][GroundLock]") {
    auto sim = makeSimulation();

    drone::simulator::config::WeatherConfig weather_config;
    weather_config.enabled = true;
    weather_config.steady_accel_enu_ms2 = drone::Vector3(1.5, 0.0, 0.0);
    weather_config.gust_amplitude_enu_ms2 = drone::Vector3(0.0, 0.0, 0.0);
    weather_config.turbulence_std_enu_ms2 = drone::Vector3(0.0, 0.0, 0.0);

    sim->setWeatherConfig(weather_config);

    drone::runtime::ActuatorFrame actuators;
    actuators.desired_motor_rpm = 16000.0;
    actuators.common_motor_rpm = 16000.0;
    actuators.desired_motor_rpm_each = {16000.0, 16000.0, 16000.0, 16000.0};

    sim->start();

    bool reached_airborne_state = false;
    for (int i = 0; i < 2500; ++i) {
        sim->applyActuators(actuators);
        sim->step(0.01);
        if (sim->readSensors().altitude_m > 0.5) {
            reached_airborne_state = true;
            break;
        }
    }

    REQUIRE(reached_airborne_state);

    for (int i = 0; i < 200; ++i) {
        sim->applyActuators(actuators);
        sim->step(0.01);
    }

    const auto sensors = sim->readSensors();
    sim->stop();

    REQUIRE(sensors.altitude_m > 0.5);
    REQUIRE(sensors.gps_velocity_east_mps > 0.1);
}
