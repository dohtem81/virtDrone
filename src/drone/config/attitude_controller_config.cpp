#include "drone/config/attitude_controller_config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace drone::config {

bool AttitudeControllerConfig::loadFromFile(const std::string& filepath) {
    try {
        const YAML::Node config = YAML::LoadFile(filepath);
        
        if (config["yaw_p_gain_rpm_per_rad"]) {
            yaw_p_gain_rpm_per_rad = config["yaw_p_gain_rpm_per_rad"].as<double>();
        }
        if (config["yaw_d_gain_rpm_per_rad_s"]) {
            yaw_d_gain_rpm_per_rad_s = config["yaw_d_gain_rpm_per_rad_s"].as<double>();
        }
        
        if (config["pitch_p_gain_rpm_per_rad"]) {
            pitch_p_gain_rpm_per_rad = config["pitch_p_gain_rpm_per_rad"].as<double>();
        }
        if (config["pitch_d_gain_rpm_per_rad_s"]) {
            pitch_d_gain_rpm_per_rad_s = config["pitch_d_gain_rpm_per_rad_s"].as<double>();
        }
        
        if (config["roll_p_gain_rpm_per_rad"]) {
            roll_p_gain_rpm_per_rad = config["roll_p_gain_rpm_per_rad"].as<double>();
        }
        if (config["roll_d_gain_rpm_per_rad_s"]) {
            roll_d_gain_rpm_per_rad_s = config["roll_d_gain_rpm_per_rad_s"].as<double>();
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load attitude controller config from '" << filepath 
                  << "': " << e.what() << std::endl;
        return false;
    }
}

}  // namespace drone::config
