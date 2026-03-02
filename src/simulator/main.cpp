#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "drone/model/quadrocopter.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/gps_sim.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/quadrosimulator.h"

namespace {

struct AltitudeControllerConfig {
    double target_altitude_m = 50.0;
    double altitude_param_p = 2.0;
    double max_altitude_delta_mps = 5.0;
    double control_param_p = 100.0;
    double control_param_i = 10.0;
    double neutral_rpm = 11400.0;
};

bool loadAltitudeControllerConfig(const std::string& config_file, AltitudeControllerConfig& config) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(config_file);
        
        if (yaml_config["altitude_controller"]) {
            auto alt_ctrl = yaml_config["altitude_controller"];
            
            if (alt_ctrl["target_altitude_m"]) {
                config.target_altitude_m = alt_ctrl["target_altitude_m"].as<double>();
            }
            if (alt_ctrl["altitude_param_p"]) {
                config.altitude_param_p = alt_ctrl["altitude_param_p"].as<double>();
            }
            if (alt_ctrl["max_altitude_delta_mps"]) {
                config.max_altitude_delta_mps = alt_ctrl["max_altitude_delta_mps"].as<double>();
            }
            if (alt_ctrl["control_param_p"]) {
                config.control_param_p = alt_ctrl["control_param_p"].as<double>();
            }
            if (alt_ctrl["control_param_i"]) {
                config.control_param_i = alt_ctrl["control_param_i"].as<double>();
            }
            if (alt_ctrl["neutral_rpm"]) {
                config.neutral_rpm = alt_ctrl["neutral_rpm"].as<double>();
            }
        }
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading config file: " << e.what() << std::endl;
        return false;
    }
}

bool parseArgs(int argc, char** argv, uint64_t& steps, double& dt_s, std::string& config_file) {
    if (argc >= 2) {
        try {
            steps = static_cast<uint64_t>(std::stoull(argv[1]));
        } catch (...) {
            return false;
        }
    }
    if (argc >= 3) {
        try {
            dt_s = std::stod(argv[2]);
        } catch (...) {
            return false;
        }
    }
    if (argc >= 4) {
        config_file = argv[3];
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    uint64_t steps = 10;
    double dt_s = 0.01;
    std::string config_file = "config/altitude_controller.yaml";

    if (!parseArgs(argc, argv, steps, dt_s, config_file)) {
        std::cerr << "Usage: " << argv[0] << " [steps] [dt_s] [config_file]" << std::endl;
        std::cerr << "  steps: number of simulation steps (default: 10)" << std::endl;
        std::cerr << "  dt_s: time step in seconds (default: 0.01)" << std::endl;
        std::cerr << "  config_file: YAML config file path (default: config/altitude_controller.yaml)" << std::endl;
        return 1;
    }

    // Load altitude controller configuration
    AltitudeControllerConfig alt_config;
    if (!loadAltitudeControllerConfig(config_file, alt_config)) {
        std::cerr << "Warning: Could not load config file '" << config_file << "', using defaults" << std::endl;
    } else {
        std::cout << "Loaded altitude controller config from: " << config_file << std::endl;
        std::cout << "  Target altitude: " << alt_config.target_altitude_m << " m" << std::endl;
        std::cout << "  Neutral RPM: " << alt_config.neutral_rpm << std::endl;
    }

    // Create default motor specs
    drone::model::components::ElecMotorSpecs motor_specs(15000.0, 14.8, 20.0, 0.9, 0.4, 0.12);
    
    // Create I/O specs for motors
    drone::model::sensors::AnalogIOSpec motor_io_spec(
        drone::model::sensors::AnalogIOSpec::IODirection::OUTPUT,
        drone::model::sensors::AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0, 10000
    );

    // Create battery specs
    drone::model::components::CellSpecs cell_specs(1500.0, 4.2);
    drone::model::components::BatterySpecs battery_specs(4, cell_specs, 0.35);

    // Create temperature sensor specs
    drone::model::sensors::AnalogIOSpec temp_io_spec(
        drone::model::sensors::AnalogIOSpec::IODirection::INPUT,
        drone::model::sensors::AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
        4000, 20000
    );
    drone::model::sensors::TemperatureSensorRanges temp_ranges(-50.0, 150.0);

    // Create GPS specs
    drone::model::components::GPSSensorSpecs gps_specs;

    // Create altitude controller with parameters from config file
    drone::model::components::AltitudeController alt_ctrl(
        alt_config.altitude_param_p,
        alt_config.max_altitude_delta_mps,
        alt_config.control_param_p,
        alt_config.control_param_i,
        alt_config.neutral_rpm
    );
    alt_ctrl.setTargetAltitude(alt_config.target_altitude_m);

    // Create simulation using factory
    auto sim = drone::simulator::QuadroSimulationFactory(
        "QuadTest",
        motor_specs,
        motor_io_spec,
        battery_specs,
        temp_io_spec,
        temp_ranges,
        0.02,  // temp_sensor_weight_kg
        gps_specs,
        1.2,   // body_weight_kg
        0.3,   // blade_diameter_m
        1.0,   // blade_shape_coeff
        alt_ctrl,
        steps,
        dt_s
    );

    sim->start();
    sim->runForSteps(steps, dt_s);
    sim->stop();

    return 0;
}
