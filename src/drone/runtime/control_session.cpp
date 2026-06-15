#include "drone/runtime/control_session.h"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "drone/config/altitude_controller_config.h"
#include "drone/config/attitude_controller_config.h"
#include "drone/model/components/altitude_controler.h"
#include "drone/model/quadrocopter.h"
#include "drone/runtime/real_drone.h"

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

void logEvent(std::ofstream& events_log, double sim_elapsed_s, const std::string& message) {
    if (!events_log.is_open()) {
        return;
    }
    events_log << std::fixed << std::setprecision(6)
               << localTimestampNow() << ","
               << sim_elapsed_s << ","
               << message << "\n";
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

}  // namespace

namespace drone::runtime {

int RunControlSession(
    SimulationGateway& gateway,
    const std::string& altitude_config_file,
    const std::string& attitude_config_file,
    const std::string& mission_file,
    uint64_t steps,
    double dt_s,
    const std::filesystem::path& logs_dir) {
    const std::string events_log_file = (logs_dir / "control_events.log").string();
    std::ofstream events_log(events_log_file, std::ios::out | std::ios::trunc);
    if (!events_log.is_open()) {
        std::cerr << "Failed to open control log file: " << events_log_file << std::endl;
        return 1;
    }

    double sim_elapsed_s = 0.0;
    logEvent(events_log, sim_elapsed_s,
             "CONTROL_START steps=" + std::to_string(steps) +
             " dt_s=" + std::to_string(dt_s));

    drone::config::AltitudeControllerConfig alt_config;
    if (!alt_config.loadFromFile(altitude_config_file)) {
        logEvent(events_log, sim_elapsed_s,
                 "WARN altitude config load failed: '" + altitude_config_file + "' using defaults");
    }

    drone::config::AttitudeControllerConfig att_config;
    if (!att_config.loadFromFile(attitude_config_file)) {
        logEvent(events_log, sim_elapsed_s,
                 "WARN attitude config load failed: '" + attitude_config_file + "' using defaults");
    }

    drone::model::components::AltitudeController alt_ctrl(
        alt_config.altitude_param_p,
        alt_config.max_altitude_delta_mps,
        alt_config.control_param_p,
        alt_config.control_param_i,
        alt_config.neutral_rpm,
        alt_config.control_param_d,
        alt_config.enable_i_component,
        alt_config.enable_d_component,
        alt_config.activation_error_band_m);

    RealDrone real_drone(alt_ctrl);
    real_drone.setTargetAltitude(alt_config.target_altitude_m);
    real_drone.setPositionControlEnabled(alt_config.position_hold_enabled);
    real_drone.setPositionGain(alt_config.position_hold_kp_pos);
    real_drone.setVelocityGains(alt_config.position_hold_kp_vel, alt_config.position_hold_kd_vel);
    real_drone.setMaxVelocity(alt_config.position_hold_max_velocity_mps);
    real_drone.setMaxTilt(alt_config.position_hold_max_tilt_rad);
    real_drone.setAttitudeGains(
        att_config.yaw_p_gain_rpm_per_rad,
        att_config.yaw_d_gain_rpm_per_rad_s,
        att_config.pitch_p_gain_rpm_per_rad,
        att_config.pitch_d_gain_rpm_per_rad_s,
        att_config.roll_p_gain_rpm_per_rad,
        att_config.roll_d_gain_rpm_per_rad_s);

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

    MissionStatus last_mission_status = MissionStatus::IDLE;
    int last_mission_step_id = -1;

    for (uint64_t i = 0; i < steps; ++i) {
        if (real_drone.hasMissionLoaded()) {
            real_drone.updateMission(gateway.readSensors(), dt_s);

            const auto mission_status = real_drone.getMissionStatus();
            const int mission_step_id = real_drone.getCurrentMissionStepId();
            const std::string mission_step_name = real_drone.getCurrentMissionStepName();
            const std::string mission_step_target = real_drone.getCurrentMissionStepTargetDescription();

            if (mission_status != last_mission_status) {
                logEvent(events_log, sim_elapsed_s,
                         "MISSION_STATUS status=" + missionStatusToString(mission_status));
                last_mission_status = mission_status;
            }

            if (mission_step_id != last_mission_step_id) {
                logEvent(events_log, sim_elapsed_s,
                         "MISSION_STEP step_id=" + std::to_string(mission_step_id) +
                             " name='" + mission_step_name + "'" +
                             " target='" + mission_step_target + "'");
                last_mission_step_id = mission_step_id;
            }
        }

        runSimulationStep(real_drone, gateway, gateway, gateway, dt_s);
        sim_elapsed_s += dt_s;

        if (real_drone.hasMissionLoaded()) {
            const auto status = real_drone.getMissionStatus();
            if (status == MissionStatus::COMPLETED ||
                status == MissionStatus::ABORTED ||
                status == MissionStatus::FAILED) {
                logEvent(events_log, sim_elapsed_s,
                         "MISSION_TERMINATED status=" + missionStatusToString(status));
                break;
            }
        }
    }

    logEvent(events_log, sim_elapsed_s, "CONTROL_STOP");
    events_log.close();
    return 0;
}

}  // namespace drone::runtime