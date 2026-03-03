#ifndef SIMULATOR_CONFIG_WEATHER_CONFIG_H
#define SIMULATOR_CONFIG_WEATHER_CONFIG_H

#include <string>

#include <yaml-cpp/yaml.h>

#include "drone/drone_data_types.h"

namespace drone::simulator::config {

class WeatherConfig {
public:
    bool enabled = false;
    drone::Vector3 steady_accel_enu_ms2{0.0, 0.0, 0.0};
    drone::Vector3 gust_amplitude_enu_ms2{0.0, 0.0, 0.0};
    double gust_frequency_hz = 0.2;
    drone::Vector3 turbulence_std_enu_ms2{0.0, 0.0, 0.0};
    uint32_t random_seed = 42;

    bool loadFromFile(const std::string& config_file) {
        try {
            const YAML::Node yaml_config = YAML::LoadFile(config_file);
            return loadFromYaml(yaml_config);
        } catch (const YAML::Exception&) {
            return false;
        }
    }

private:
    bool loadFromYaml(const YAML::Node& yaml_config) {
        if (!yaml_config["weather"]) {
            return true;
        }

        const auto weather = yaml_config["weather"];
        readIfPresent(weather, "enabled", enabled);
        readVector3IfPresent(weather, "steady_accel_enu_ms2", steady_accel_enu_ms2);
        readVector3IfPresent(weather, "gust_amplitude_enu_ms2", gust_amplitude_enu_ms2);
        readIfPresent(weather, "gust_frequency_hz", gust_frequency_hz);
        readVector3IfPresent(weather, "turbulence_std_enu_ms2", turbulence_std_enu_ms2);
        readIfPresent(weather, "random_seed", random_seed);
        return true;
    }

    template <typename T>
    static void readIfPresent(const YAML::Node& node, const char* key, T& value) {
        if (node[key]) {
            value = node[key].as<T>();
        }
    }

    static void readVector3IfPresent(const YAML::Node& node, const char* key, drone::Vector3& value) {
        if (!node[key]) {
            return;
        }
        const auto vector_node = node[key];
        if (!vector_node.IsSequence() || vector_node.size() != 3) {
            return;
        }
        value.x = vector_node[0].as<double>();
        value.y = vector_node[1].as<double>();
        value.z = vector_node[2].as<double>();
    }
};

}  // namespace drone::simulator::config

#endif  // SIMULATOR_CONFIG_WEATHER_CONFIG_H
