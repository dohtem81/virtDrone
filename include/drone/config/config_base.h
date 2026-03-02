#ifndef DRONE_CONFIG_CONFIG_BASE_H
#define DRONE_CONFIG_CONFIG_BASE_H

#include <string>

#include <yaml-cpp/yaml.h>

namespace drone::config {

class ConfigBase {
public:
    virtual ~ConfigBase() = default;

    bool loadFromFile(const std::string& config_file) {
        try {
            YAML::Node yaml_config = YAML::LoadFile(config_file);
            return loadFromYaml(yaml_config);
        } catch (const YAML::Exception&) {
            return false;
        }
    }

protected:
    virtual bool loadFromYaml(const YAML::Node& yaml_config) = 0;

    template <typename T>
    static void readIfPresent(const YAML::Node& node, const char* key, T& value) {
        if (node[key]) {
            value = node[key].as<T>();
        }
    }
};

}  // namespace drone::config

#endif  // DRONE_CONFIG_CONFIG_BASE_H
