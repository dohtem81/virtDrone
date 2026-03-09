#pragma once

#include <string>

namespace drone::config {

struct AttitudeControllerConfig {
    // Yaw control parameters
    double yaw_p_gain_rpm_per_rad = 200.0;
    double yaw_d_gain_rpm_per_rad_s = 50.0;

    // Pitch control parameters
    double pitch_p_gain_rpm_per_rad = 250.0;
    double pitch_d_gain_rpm_per_rad_s = 60.0;

    // Roll control parameters
    double roll_p_gain_rpm_per_rad = 250.0;
    double roll_d_gain_rpm_per_rad_s = 60.0;

    /**
     * @brief Load attitude controller configuration from YAML file.
     * @param filepath Path to YAML config file.
     * @return true if loaded successfully, false otherwise (uses defaults).
     */
    bool loadFromFile(const std::string& filepath);
};

}  // namespace drone::config
