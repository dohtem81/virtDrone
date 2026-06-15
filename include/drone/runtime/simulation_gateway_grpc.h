#ifndef DRONE_RUNTIME_SIMULATION_GATEWAY_GRPC_H
#define DRONE_RUNTIME_SIMULATION_GATEWAY_GRPC_H

#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>

#include "drone/runtime/simulation_gateway.h"
#include "simulation_gateway.pb.h"
#include "simulation_gateway.grpc.pb.h"

namespace drone::runtime {

virtDrone::runtime::SensorFrame ToProto(const SensorFrame& frame);
SensorFrame FromProto(const virtDrone::runtime::SensorFrame& frame);

virtDrone::runtime::ActuatorFrame ToProto(const ActuatorFrame& frame);
ActuatorFrame FromProto(const virtDrone::runtime::ActuatorFrame& frame);

class GrpcSimulationGatewayClient final : public SimulationGateway {
public:
    explicit GrpcSimulationGatewayClient(std::shared_ptr<::grpc::Channel> channel);

    SensorFrame readSensors() const override;
    void applyActuators(const ActuatorFrame& actuator_frame) override;
    void step(double dt_s) override;

private:
    std::unique_ptr<virtDrone::runtime::SimulationGateway::Stub> stub_;
};

class GrpcSimulationGatewayService final : public virtDrone::runtime::SimulationGateway::Service {
public:
    explicit GrpcSimulationGatewayService(SimulationGateway& gateway);

    ::grpc::Status ReadSensors(
        ::grpc::ServerContext* context,
        const virtDrone::runtime::Empty* request,
        virtDrone::runtime::SensorFrame* response) override;

    ::grpc::Status ApplyActuators(
        ::grpc::ServerContext* context,
        const virtDrone::runtime::ActuatorFrame* request,
        virtDrone::runtime::Empty* response) override;

    ::grpc::Status Step(
        ::grpc::ServerContext* context,
        const virtDrone::runtime::StepRequest* request,
        virtDrone::runtime::Empty* response) override;

private:
    SimulationGateway& gateway_;
};

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_SIMULATION_GATEWAY_GRPC_H