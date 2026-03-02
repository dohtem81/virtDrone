#ifndef DRONE_CONFIG_ALTITUDE_CONTROLLER_CONFIG_H
#define DRONE_CONFIG_ALTITUDE_CONTROLLER_CONFIG_H

#include "drone/config/config_base.h"

namespace drone::config {

class AltitudeControllerConfig final : public ConfigBase {
public:
    double target_altitude_m = 50.0;
    double altitude_param_p = 2.0;
    double max_altitude_delta_mps = 5.0;
    double control_param_p = 100000.0;
    double control_param_i = 50.0;
    double control_param_d = 0.0;
    bool enable_i_component = true;
    bool enable_d_component = false;
    double activation_error_band_m = 0.5;
    double neutral_rpm = 11400.0;

protected:
    bool loadFromYaml(const YAML::Node& yaml_config) override {
        if (!yaml_config["altitude_controller"]) {
            return true;
        }

        const auto altitude_controller = yaml_config["altitude_controller"];
        readIfPresent(altitude_controller, "target_altitude_m", target_altitude_m);
        readIfPresent(altitude_controller, "altitude_param_p", altitude_param_p);
        readIfPresent(altitude_controller, "max_altitude_delta_mps", max_altitude_delta_mps);
        readIfPresent(altitude_controller, "control_param_p", control_param_p);
        readIfPresent(altitude_controller, "control_param_i", control_param_i);
        readIfPresent(altitude_controller, "control_param_d", control_param_d);
        readIfPresent(altitude_controller, "enable_i_component", enable_i_component);
        readIfPresent(altitude_controller, "enable_d_component", enable_d_component);
        readIfPresent(altitude_controller, "activation_error_band_m", activation_error_band_m);
        readIfPresent(altitude_controller, "neutral_rpm", neutral_rpm);
        return true;
    }
};

}  // namespace drone::config

#endif  // DRONE_CONFIG_ALTITUDE_CONTROLLER_CONFIG_H
