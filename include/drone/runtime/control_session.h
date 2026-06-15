#ifndef DRONE_RUNTIME_CONTROL_SESSION_H
#define DRONE_RUNTIME_CONTROL_SESSION_H

#include <cstdint>
#include <filesystem>
#include <string>

#include "drone/runtime/simulation_gateway.h"

namespace drone::runtime {

int RunControlSession(
    SimulationGateway& gateway,
    const std::string& altitude_config_file,
    const std::string& attitude_config_file,
    const std::string& mission_file,
    uint64_t steps,
    double dt_s,
    const std::filesystem::path& logs_dir);

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_CONTROL_SESSION_H