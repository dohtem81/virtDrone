#ifndef DRONE_RUNTIME_SIMULATION_GATEWAY_H
#define DRONE_RUNTIME_SIMULATION_GATEWAY_H

#include "drone/runtime/real_drone.h"

namespace drone::runtime {

// Process boundary contract for split-control/split-simulator deployments.
// A concrete transport layer (for example gRPC) can implement this interface.
class SimulationGateway : public SensorSource, public ActuatorSink {
public:
    ~SimulationGateway() override = default;
    virtual void step(double dt_s) = 0;
};

// Convenience helper for the current runtime loop shape.
inline void runSimulationStep(
    RealDrone& real_drone,
    SensorSource& sensor_source,
    ActuatorSink& actuator_sink,
    SimulationGateway& gateway,
    double dt_s) {
    real_drone.update(dt_s, sensor_source, actuator_sink);
    gateway.step(dt_s);
}

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_SIMULATION_GATEWAY_H