#ifndef DRONE_RUNTIME_SIMULATION_GATEWAY_RPC_H
#define DRONE_RUNTIME_SIMULATION_GATEWAY_RPC_H

#include "drone/runtime/simulation_gateway.h"

namespace drone::runtime {

class SimulationGatewayRpcService {
public:
    virtual ~SimulationGatewayRpcService() = default;
    virtual SensorFrame ReadSensors() const = 0;
    virtual void ApplyActuators(const ActuatorFrame& actuator_frame) = 0;
    virtual void Step(double dt_s) = 0;
};

class SimulationGatewayRpcStub final : public SimulationGateway {
public:
    explicit SimulationGatewayRpcStub(SimulationGatewayRpcService& service)
        : service_(service) {}

    SensorFrame readSensors() const override {
        return service_.ReadSensors();
    }

    void applyActuators(const ActuatorFrame& actuator_frame) override {
        service_.ApplyActuators(actuator_frame);
    }

    void step(double dt_s) override {
        service_.Step(dt_s);
    }

private:
    SimulationGatewayRpcService& service_;
};

class LocalSimulationGatewayRpcService final : public SimulationGatewayRpcService {
public:
    explicit LocalSimulationGatewayRpcService(SimulationGateway& gateway)
        : gateway_(gateway) {}

    SensorFrame ReadSensors() const override {
        return gateway_.readSensors();
    }

    void ApplyActuators(const ActuatorFrame& actuator_frame) override {
        gateway_.applyActuators(actuator_frame);
    }

    void Step(double dt_s) override {
        gateway_.step(dt_s);
    }

private:
    SimulationGateway& gateway_;
};

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_SIMULATION_GATEWAY_RPC_H