#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "drone/config/altitude_controller_config.h"
#include "drone/model/quadrocopter.h"
#include "drone/runtime/real_drone.h"
#include "simulator/config/weather_config.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/gps_sim.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/quadrosimulator.h"
#include "simulator/runtime/noisy_sensor_source.h"

namespace {

std::string localTimestampNow() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif
    std::ostringstream out;
    out << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
    return out.str();
}

std::string missionStatusToString(drone::mission::MissionStatus status) {
    using drone::mission::MissionStatus;
    switch (status) {
        case MissionStatus::IDLE:
            return "IDLE";
        case MissionStatus::RUNNING:
            return "RUNNING";
        case MissionStatus::PAUSED:
            return "PAUSED";
        case MissionStatus::COMPLETED:
            return "COMPLETED";
        case MissionStatus::ABORTED:
            return "ABORTED";
        case MissionStatus::FAILED:
            return "FAILED";
    }
    return "UNKNOWN";
}

std::filesystem::path resolveLogsDir(const std::string& logs_dir_arg) {
    namespace fs = std::filesystem;

    if (!logs_dir_arg.empty()) {
        const fs::path requested_dir = fs::path(logs_dir_arg);
        fs::create_directories(requested_dir);
        return requested_dir;
    }

    const fs::path cwd_tutorial = fs::path("docs") / "tutorials";
    const fs::path parent_tutorial = fs::path("..") / "docs" / "tutorials";

    if (fs::exists(cwd_tutorial) && fs::is_directory(cwd_tutorial)) {
        return cwd_tutorial;
    }
    if (fs::exists(parent_tutorial) && fs::is_directory(parent_tutorial)) {
        return parent_tutorial;
    }

    fs::create_directories(cwd_tutorial);
    return cwd_tutorial;
}

void logEvent(std::ofstream& events_log, double sim_elapsed_s, const std::string& message) {
    if (!events_log.is_open()) {
        return;
    }
    events_log << std::fixed << std::setprecision(6)
               << localTimestampNow() << ","
               << sim_elapsed_s << ","
               << message << "\n";
}

bool parseArgs(
    int argc,
    char** argv,
    uint64_t& steps,
    double& dt_s,
    std::string& altitude_config_file,
    std::string& weather_config_file,
    std::string& mission_file,
    std::string& logs_dir) {
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
        altitude_config_file = argv[3];
    }
    if (argc >= 5) {
        weather_config_file = argv[4];
    }
    if (argc >= 6) {
        mission_file = argv[5];
    }
    if (argc >= 7) {
        logs_dir = argv[6];
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    uint64_t steps = 10;
    double dt_s = 0.01;
    std::string altitude_config_file = "config/altitude_controller.yaml";
    std::string weather_config_file = "config/weather.yaml";
    std::string mission_file;
    std::string logs_dir;
    double sim_elapsed_s = 0.0;

    if (!parseArgs(argc, argv, steps, dt_s, altitude_config_file, weather_config_file, mission_file, logs_dir)) {
        std::cerr << "Usage: " << argv[0] << " [steps] [dt_s] [altitude_config_file] [weather_config_file] [mission_file] [logs_dir]" << std::endl;
        std::cerr << "  steps: number of simulation steps (default: 10)" << std::endl;
        std::cerr << "  dt_s: time step in seconds (default: 0.01)" << std::endl;
        std::cerr << "  altitude_config_file: YAML config file path (default: config/altitude_controller.yaml)" << std::endl;
        std::cerr << "  weather_config_file: YAML config file path (default: config/weather.yaml)" << std::endl;
        std::cerr << "  mission_file: YAML mission file path (optional)" << std::endl;
        std::cerr << "  logs_dir: output directory for simulation_telemetry.csv and simulation_events.log (optional, default: docs/tutorials)" << std::endl;
        return 1;
    }

    const std::filesystem::path output_logs_dir = resolveLogsDir(logs_dir);
    const std::string telemetry_log_file = (output_logs_dir / "simulation_telemetry.csv").string();
    const std::string events_log_file = (output_logs_dir / "simulation_events.log").string();

    std::ofstream events_log(events_log_file, std::ios::out | std::ios::trunc);
    if (!events_log.is_open()) {
        std::cerr << "Failed to open events log file: " << events_log_file << std::endl;
        return 1;
    }

    logEvent(events_log, sim_elapsed_s,
             "SIMULATION_START steps=" + std::to_string(steps) +
             " dt_s=" + std::to_string(dt_s) +
             " altitude_config='" + altitude_config_file + "'" +
             " weather_config='" + weather_config_file + "'" +
             " mission_file='" + mission_file + "'" +
             " logs_dir='" + output_logs_dir.string() + "'" +
             " telemetry_csv='" + telemetry_log_file + "'");

    // Load altitude controller configuration
    drone::config::AltitudeControllerConfig alt_config;
    if (!alt_config.loadFromFile(altitude_config_file)) {
        logEvent(events_log, sim_elapsed_s,
                 "WARN altitude config load failed: '" + altitude_config_file + "' using defaults");
    } else {
        logEvent(events_log, sim_elapsed_s, "Loaded altitude config: '" + altitude_config_file + "'");
    }

    drone::simulator::config::WeatherConfig weather_config;
    if (!weather_config.loadFromFile(weather_config_file)) {
        logEvent(events_log, sim_elapsed_s,
                 "WARN weather config load failed: '" + weather_config_file + "' using defaults");
    } else {
        logEvent(events_log, sim_elapsed_s, "Loaded weather config: '" + weather_config_file + "'");
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
        alt_config.neutral_rpm,
        alt_config.control_param_d,
        alt_config.enable_i_component,
        alt_config.enable_d_component,
        alt_config.activation_error_band_m
    );

    drone::runtime::RealDrone real_drone(alt_ctrl);
    real_drone.setTargetAltitude(alt_config.target_altitude_m);
    real_drone.setPositionControlEnabled(alt_config.position_hold_enabled);
    real_drone.setPositionGain(alt_config.position_hold_kp_pos);
    real_drone.setVelocityGains(alt_config.position_hold_kp_vel, alt_config.position_hold_kd_vel);
    real_drone.setMaxVelocity(alt_config.position_hold_max_velocity_mps);
    real_drone.setMaxTilt(alt_config.position_hold_max_tilt_rad);

    {
        std::ostringstream params;
        params << "Altitude controller params"
               << " target_altitude_m=" << alt_config.target_altitude_m
               << " altitude_param_p=" << alt_config.altitude_param_p
               << " max_altitude_delta_mps=" << alt_config.max_altitude_delta_mps
               << " control_param_p=" << alt_config.control_param_p
               << " control_param_i=" << alt_config.control_param_i
               << " control_param_d=" << alt_config.control_param_d
               << " enable_i_component=" << (alt_config.enable_i_component ? "true" : "false")
               << " enable_d_component=" << (alt_config.enable_d_component ? "true" : "false")
               << " activation_error_band_m=" << alt_config.activation_error_band_m
               << " neutral_rpm=" << alt_config.neutral_rpm
               << " position_hold_enabled=" << (alt_config.position_hold_enabled ? "true" : "false")
               << " position_hold_kp_pos=" << alt_config.position_hold_kp_pos
               << " position_hold_kp_vel=" << alt_config.position_hold_kp_vel
               << " position_hold_kd_vel=" << alt_config.position_hold_kd_vel
               << " position_hold_max_velocity_mps=" << alt_config.position_hold_max_velocity_mps
               << " position_hold_max_tilt_rad=" << alt_config.position_hold_max_tilt_rad;
        logEvent(events_log, sim_elapsed_s, params.str());
    }

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
        steps,
        dt_s
    );

    sim->setWeatherConfig(weather_config);
    if (!sim->setTelemetryLogFile(telemetry_log_file)) {
        logEvent(events_log, sim_elapsed_s, "ERROR failed to open telemetry csv: '" + telemetry_log_file + "'");
        return 1;
    }
    logEvent(events_log, sim_elapsed_s, "Telemetry CSV initialized: '" + telemetry_log_file + "'");

    if (!mission_file.empty()) {
        std::string mission_error;
        if (!real_drone.loadMissionFromFile(mission_file, &mission_error)) {
            logEvent(events_log, sim_elapsed_s,
                     "ERROR mission load failed: '" + mission_file + "' reason='" + mission_error + "'");
            return 1;
        }
        real_drone.startMission();
        logEvent(events_log, sim_elapsed_s, "Loaded mission: '" + mission_file + "'");
    }

    sim->start();
    logEvent(events_log, sim_elapsed_s, "Simulation runtime started");

    drone::simulator::runtime::NoisySensorSource noisy_sensor_source(*sim);
    drone::mission::MissionStatus last_mission_status = drone::mission::MissionStatus::IDLE;
    int last_mission_step_id = -1;

    for (uint64_t i = 0; i < steps; ++i) {
        if (real_drone.hasMissionLoaded()) {
            real_drone.updateMission(sim->readSensors(), dt_s);

            const auto mission_status = real_drone.getMissionStatus();
            const int mission_step_id = real_drone.getCurrentMissionStepId();
            const std::string mission_step_name = real_drone.getCurrentMissionStepName();

            if (mission_status != last_mission_status) {
                logEvent(events_log, sim_elapsed_s,
                         "MISSION_STATUS status=" + missionStatusToString(mission_status));
                last_mission_status = mission_status;
            }

            if (mission_step_id != last_mission_step_id) {
                logEvent(events_log, sim_elapsed_s,
                         "MISSION_STEP step_id=" + std::to_string(mission_step_id) +
                             " name='" + mission_step_name + "'");
                last_mission_step_id = mission_step_id;
            }
        }

        real_drone.update(dt_s, noisy_sensor_source, *sim);
        sim->step(dt_s);
        sim_elapsed_s += dt_s;

        if (real_drone.hasMissionLoaded()) {
            const auto status = real_drone.getMissionStatus();
            if (status == drone::mission::MissionStatus::COMPLETED ||
                status == drone::mission::MissionStatus::ABORTED ||
                status == drone::mission::MissionStatus::FAILED) {
                logEvent(events_log, sim_elapsed_s,
                         "MISSION_TERMINATED status=" + missionStatusToString(status));
                break;
            }
        }
    }
    sim->stop();
    logEvent(events_log, sim_elapsed_s, "SIMULATION_STOP");
    events_log.close();

    return 0;
}
