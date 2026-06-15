#ifndef DRONE_RUNTIME_SIMULATION_GATEWAY_FACTORY_H
#define DRONE_RUNTIME_SIMULATION_GATEWAY_FACTORY_H

#include <memory>
#include <string>

#include "drone/runtime/simulation_gateway.h"

namespace drone::runtime {

enum class ControlBackend {
    GrpcSimulation,
    Hardware,
};

std::unique_ptr<SimulationGateway> CreateSimulationGateway(const std::string& endpoint);
std::unique_ptr<SimulationGateway> CreateControlGateway(ControlBackend backend, const std::string& endpoint);

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_SIMULATION_GATEWAY_FACTORY_H