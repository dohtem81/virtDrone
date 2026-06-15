#include "drone/runtime/simulation_gateway_grpc.h"
#include "drone/runtime/simulation_gateway_factory.h"

namespace drone::runtime {

virtDrone::runtime::SensorFrame ToProto(const SensorFrame& frame) {
    virtDrone::runtime::SensorFrame proto;
    proto.set_altitude_m(frame.altitude_m);
    proto.set_position_enu_x_m(frame.position_enu_x_m);
    proto.set_position_enu_y_m(frame.position_enu_y_m);
    proto.set_position_enu_z_m(frame.position_enu_z_m);
    proto.set_gps_latitude_deg(frame.gps_latitude_deg);
    proto.set_gps_longitude_deg(frame.gps_longitude_deg);
    proto.set_gps_altitude_m(frame.gps_altitude_m);
    proto.set_gps_velocity_north_mps(frame.gps_velocity_north_mps);
    proto.set_gps_velocity_east_mps(frame.gps_velocity_east_mps);
    proto.set_gps_velocity_down_mps(frame.gps_velocity_down_mps);
    proto.set_battery_voltage_v(frame.battery_voltage_v);
    proto.set_battery_soc_percent(frame.battery_soc_percent);
    proto.set_motor_temperature_c(frame.motor_temperature_c);
    proto.set_motor_rpm(frame.motor_rpm);
    for (double rpm : frame.motor_rpm_each) {
        proto.add_motor_rpm_each(rpm);
    }
    for (double temp : frame.motor_temperature_c_each) {
        proto.add_motor_temperature_c_each(temp);
    }
    proto.set_yaw_rad(frame.yaw_rad);
    proto.set_pitch_rad(frame.pitch_rad);
    proto.set_roll_rad(frame.roll_rad);
    return proto;
}

SensorFrame FromProto(const virtDrone::runtime::SensorFrame& frame) {
    SensorFrame out;
    out.altitude_m = frame.altitude_m();
    out.position_enu_x_m = frame.position_enu_x_m();
    out.position_enu_y_m = frame.position_enu_y_m();
    out.position_enu_z_m = frame.position_enu_z_m();
    out.gps_latitude_deg = frame.gps_latitude_deg();
    out.gps_longitude_deg = frame.gps_longitude_deg();
    out.gps_altitude_m = frame.gps_altitude_m();
    out.gps_velocity_north_mps = frame.gps_velocity_north_mps();
    out.gps_velocity_east_mps = frame.gps_velocity_east_mps();
    out.gps_velocity_down_mps = frame.gps_velocity_down_mps();
    out.battery_voltage_v = frame.battery_voltage_v();
    out.battery_soc_percent = frame.battery_soc_percent();
    out.motor_temperature_c = frame.motor_temperature_c();
    out.motor_rpm = frame.motor_rpm();
    for (int i = 0; i < frame.motor_rpm_each_size() && i < static_cast<int>(kMotorCount); ++i) {
        out.motor_rpm_each[static_cast<std::size_t>(i)] = frame.motor_rpm_each(i);
    }
    for (int i = 0; i < frame.motor_temperature_c_each_size() && i < static_cast<int>(kMotorCount); ++i) {
        out.motor_temperature_c_each[static_cast<std::size_t>(i)] = frame.motor_temperature_c_each(i);
    }
    out.yaw_rad = frame.yaw_rad();
    out.pitch_rad = frame.pitch_rad();
    out.roll_rad = frame.roll_rad();
    return out;
}

virtDrone::runtime::ActuatorFrame ToProto(const ActuatorFrame& frame) {
    virtDrone::runtime::ActuatorFrame proto;
    proto.set_desired_motor_rpm(frame.desired_motor_rpm);
    proto.set_common_motor_rpm(frame.common_motor_rpm);
    for (double rpm : frame.desired_motor_rpm_each) {
        proto.add_desired_motor_rpm_each(rpm);
    }
    proto.set_yaw_control_rpm(frame.yaw_control_rpm);
    proto.set_pitch_control_rpm(frame.pitch_control_rpm);
    proto.set_roll_control_rpm(frame.roll_control_rpm);
    proto.set_desired_yaw_rad(frame.desired_yaw_rad);
    proto.set_desired_pitch_rad(frame.desired_pitch_rad);
    proto.set_desired_roll_rad(frame.desired_roll_rad);
    proto.set_target_altitude_m(frame.target_altitude_m);
    proto.set_target_error_m(frame.target_error_m);
    proto.set_p_component_rpm(frame.p_component_rpm);
    proto.set_i_component_rpm(frame.i_component_rpm);
    proto.set_d_component_rpm(frame.d_component_rpm);
    proto.set_sensed_altitude_m(frame.sensed_altitude_m);
    proto.set_sensed_position_enu_x_m(frame.sensed_position_enu_x_m);
    proto.set_sensed_position_enu_y_m(frame.sensed_position_enu_y_m);
    proto.set_sensed_position_enu_z_m(frame.sensed_position_enu_z_m);
    proto.set_sensed_gps_latitude_deg(frame.sensed_gps_latitude_deg);
    proto.set_sensed_gps_longitude_deg(frame.sensed_gps_longitude_deg);
    proto.set_sensed_gps_altitude_m(frame.sensed_gps_altitude_m);
    proto.set_sensed_gps_velocity_north_mps(frame.sensed_gps_velocity_north_mps);
    proto.set_sensed_gps_velocity_east_mps(frame.sensed_gps_velocity_east_mps);
    proto.set_sensed_gps_velocity_down_mps(frame.sensed_gps_velocity_down_mps);
    proto.set_sensed_battery_voltage_v(frame.sensed_battery_voltage_v);
    proto.set_sensed_battery_soc_percent(frame.sensed_battery_soc_percent);
    proto.set_sensed_motor_temperature_c(frame.sensed_motor_temperature_c);
    proto.set_sensed_motor_rpm(frame.sensed_motor_rpm);
    proto.set_sensed_yaw_rad(frame.sensed_yaw_rad);
    proto.set_sensed_pitch_rad(frame.sensed_pitch_rad);
    proto.set_sensed_roll_rad(frame.sensed_roll_rad);
    return proto;
}

ActuatorFrame FromProto(const virtDrone::runtime::ActuatorFrame& frame) {
    ActuatorFrame out;
    out.desired_motor_rpm = frame.desired_motor_rpm();
    out.common_motor_rpm = frame.common_motor_rpm();
    for (int i = 0; i < frame.desired_motor_rpm_each_size() && i < static_cast<int>(kMotorCount); ++i) {
        out.desired_motor_rpm_each[static_cast<std::size_t>(i)] = frame.desired_motor_rpm_each(i);
    }
    out.yaw_control_rpm = frame.yaw_control_rpm();
    out.pitch_control_rpm = frame.pitch_control_rpm();
    out.roll_control_rpm = frame.roll_control_rpm();
    out.desired_yaw_rad = frame.desired_yaw_rad();
    out.desired_pitch_rad = frame.desired_pitch_rad();
    out.desired_roll_rad = frame.desired_roll_rad();
    out.target_altitude_m = frame.target_altitude_m();
    out.target_error_m = frame.target_error_m();
    out.p_component_rpm = frame.p_component_rpm();
    out.i_component_rpm = frame.i_component_rpm();
    out.d_component_rpm = frame.d_component_rpm();
    out.sensed_altitude_m = frame.sensed_altitude_m();
    out.sensed_position_enu_x_m = frame.sensed_position_enu_x_m();
    out.sensed_position_enu_y_m = frame.sensed_position_enu_y_m();
    out.sensed_position_enu_z_m = frame.sensed_position_enu_z_m();
    out.sensed_gps_latitude_deg = frame.sensed_gps_latitude_deg();
    out.sensed_gps_longitude_deg = frame.sensed_gps_longitude_deg();
    out.sensed_gps_altitude_m = frame.sensed_gps_altitude_m();
    out.sensed_gps_velocity_north_mps = frame.sensed_gps_velocity_north_mps();
    out.sensed_gps_velocity_east_mps = frame.sensed_gps_velocity_east_mps();
    out.sensed_gps_velocity_down_mps = frame.sensed_gps_velocity_down_mps();
    out.sensed_battery_voltage_v = frame.sensed_battery_voltage_v();
    out.sensed_battery_soc_percent = frame.sensed_battery_soc_percent();
    out.sensed_motor_temperature_c = frame.sensed_motor_temperature_c();
    out.sensed_motor_rpm = frame.sensed_motor_rpm();
    out.sensed_yaw_rad = frame.sensed_yaw_rad();
    out.sensed_pitch_rad = frame.sensed_pitch_rad();
    out.sensed_roll_rad = frame.sensed_roll_rad();
    return out;
}

GrpcSimulationGatewayClient::GrpcSimulationGatewayClient(std::shared_ptr<::grpc::Channel> channel)
    : stub_(virtDrone::runtime::SimulationGateway::NewStub(channel)) {}

SensorFrame GrpcSimulationGatewayClient::readSensors() const {
    ::grpc::ClientContext context;
    virtDrone::runtime::Empty request;
    virtDrone::runtime::SensorFrame response;
    stub_->ReadSensors(&context, request, &response);
    return FromProto(response);
}

void GrpcSimulationGatewayClient::applyActuators(const ActuatorFrame& actuator_frame) {
    ::grpc::ClientContext context;
    virtDrone::runtime::ActuatorFrame request = ToProto(actuator_frame);
    virtDrone::runtime::Empty response;
    stub_->ApplyActuators(&context, request, &response);
}

void GrpcSimulationGatewayClient::step(double dt_s) {
    ::grpc::ClientContext context;
    virtDrone::runtime::StepRequest request;
    request.set_dt_s(dt_s);
    virtDrone::runtime::Empty response;
    stub_->Step(&context, request, &response);
}

GrpcSimulationGatewayService::GrpcSimulationGatewayService(SimulationGateway& gateway)
    : gateway_(gateway) {}

std::unique_ptr<SimulationGateway> CreateSimulationGateway(const std::string& endpoint) {
    return std::make_unique<GrpcSimulationGatewayClient>(
        ::grpc::CreateChannel(endpoint, ::grpc::InsecureChannelCredentials()));
}

std::unique_ptr<SimulationGateway> CreateControlGateway(ControlBackend backend, const std::string& endpoint) {
    switch (backend) {
        case ControlBackend::GrpcSimulation:
            return CreateSimulationGateway(endpoint);
        case ControlBackend::Hardware:
            return nullptr;
    }
    return nullptr;
}

::grpc::Status GrpcSimulationGatewayService::ReadSensors(
    ::grpc::ServerContext*,
    const virtDrone::runtime::Empty*,
    virtDrone::runtime::SensorFrame* response) {
    *response = ToProto(gateway_.readSensors());
    return ::grpc::Status::OK;
}

::grpc::Status GrpcSimulationGatewayService::ApplyActuators(
    ::grpc::ServerContext*,
    const virtDrone::runtime::ActuatorFrame* request,
    virtDrone::runtime::Empty*) {
    gateway_.applyActuators(FromProto(*request));
    return ::grpc::Status::OK;
}

::grpc::Status GrpcSimulationGatewayService::Step(
    ::grpc::ServerContext*,
    const virtDrone::runtime::StepRequest* request,
    virtDrone::runtime::Empty*) {
    gateway_.step(request->dt_s());
    return ::grpc::Status::OK;
}

}  // namespace drone::runtime