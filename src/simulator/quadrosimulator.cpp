#include "simulator/quadrosimulator.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/thrust_model.h"
#include "simulator/physics/gps_sim.h"
#include "simulator/physics/force_dynamics.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

std::string localTimestampNow() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif
    std::ostringstream out;
    out << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
    return out.str();
}

}  // namespace

namespace drone::simulator {

drone::runtime::SensorFrame QuaroSimulation::readSensors() const {
    drone::runtime::SensorFrame sensor_frame;
    if (!quad_) {
        return sensor_frame;
    }

    sensor_frame.altitude_m = quad_->getAltitudeM();
    sensor_frame.position_enu_x_m = position_enu_m_.x;
    sensor_frame.position_enu_y_m = position_enu_m_.y;
    sensor_frame.position_enu_z_m = position_enu_m_.z;
    if (quad_->getGPS()) {
        const auto gps_position = quad_->getGPS()->getPosition();
        const auto gps_velocity = quad_->getGPS()->getVelocity();
        sensor_frame.gps_latitude_deg = gps_position.latitude_deg;
        sensor_frame.gps_longitude_deg = gps_position.longitude_deg;
        sensor_frame.gps_altitude_m = gps_position.altitude_m;
        sensor_frame.gps_velocity_north_mps = gps_velocity.north_mps;
        sensor_frame.gps_velocity_east_mps = gps_velocity.east_mps;
        sensor_frame.gps_velocity_down_mps = gps_velocity.down_mps;
    }
    sensor_frame.battery_voltage_v = quad_->getBatteryVoltageV();
    sensor_frame.battery_soc_percent = quad_->getBatterySOC();
    sensor_frame.motor_temperature_c = quad_->getTemperatureC();
    sensor_frame.yaw_rad = attitude_ypr_rad_.yaw_rad;
    sensor_frame.pitch_rad = attitude_ypr_rad_.pitch_rad;
    sensor_frame.roll_rad = attitude_ypr_rad_.roll_rad;

    const auto& motors = quad_->getMotors();
    sensor_frame.motor_rpm = motors.empty() ? 0.0 : motors[0].getSpeedRPM();
    for (std::size_t i = 0; i < motors.size() && i < sensor_frame.motor_rpm_each.size(); ++i) {
        sensor_frame.motor_rpm_each[i] = motors[i].getSpeedRPM();
        sensor_frame.motor_temperature_c_each[i] = motors[i].getTemperatureC();
    }
    return sensor_frame;
}

void QuaroSimulation::applyActuators(const drone::runtime::ActuatorFrame& actuator_frame) {
    desired_rpm_ = actuator_frame.desired_motor_rpm;
    common_motor_rpm_ = actuator_frame.common_motor_rpm;
    desired_motor_rpm_each_ = actuator_frame.desired_motor_rpm_each;
    yaw_control_rpm_ = actuator_frame.yaw_control_rpm;
    pitch_control_rpm_ = actuator_frame.pitch_control_rpm;
    roll_control_rpm_ = actuator_frame.roll_control_rpm;
    attitude_ypr_rad_.yaw_rad = actuator_frame.desired_yaw_rad;
    attitude_ypr_rad_.pitch_rad = actuator_frame.desired_pitch_rad;
    attitude_ypr_rad_.roll_rad = actuator_frame.desired_roll_rad;
    target_altitude_m_ = actuator_frame.target_altitude_m;
    target_error_m_ = actuator_frame.target_error_m;
    p_component_rpm_ = actuator_frame.p_component_rpm;
    i_component_rpm_ = actuator_frame.i_component_rpm;
    d_component_rpm_ = actuator_frame.d_component_rpm;
    sensed_altitude_m_ = actuator_frame.sensed_altitude_m;
    sensed_position_enu_x_m_ = actuator_frame.sensed_position_enu_x_m;
    sensed_position_enu_y_m_ = actuator_frame.sensed_position_enu_y_m;
    sensed_position_enu_z_m_ = actuator_frame.sensed_position_enu_z_m;
    sensed_gps_latitude_deg_ = actuator_frame.sensed_gps_latitude_deg;
    sensed_gps_longitude_deg_ = actuator_frame.sensed_gps_longitude_deg;
    sensed_gps_altitude_m_ = actuator_frame.sensed_gps_altitude_m;
    sensed_gps_velocity_north_mps_ = actuator_frame.sensed_gps_velocity_north_mps;
    sensed_gps_velocity_east_mps_ = actuator_frame.sensed_gps_velocity_east_mps;
    sensed_gps_velocity_down_mps_ = actuator_frame.sensed_gps_velocity_down_mps;
    sensed_battery_voltage_v_ = actuator_frame.sensed_battery_voltage_v;
    sensed_battery_soc_percent_ = actuator_frame.sensed_battery_soc_percent;
    sensed_motor_temperature_c_ = actuator_frame.sensed_motor_temperature_c;
    sensed_motor_rpm_ = actuator_frame.sensed_motor_rpm;
}

void QuaroSimulation::setWeatherConfig(const drone::simulator::config::WeatherConfig& weather_config) {
    weather_model_.setConfig(weather_config);
}

bool QuaroSimulation::setTelemetryLogFile(const std::string& telemetry_log_file) {
    telemetry_log_file_ = telemetry_log_file;
    if (telemetry_log_stream_.is_open()) {
        telemetry_log_stream_.close();
    }

    telemetry_log_stream_.open(telemetry_log_file_, std::ios::out | std::ios::trunc);
    telemetry_csv_header_written_ = false;
    return telemetry_log_stream_.is_open();
}

void QuaroSimulation::onStart() {
    if (!is_running_) {
        is_running_ = true;
        elapsed_s_ = 0.0;
        position_enu_m_ = drone::Vector3(0.0, 0.0, altitude_m_);
        velocity_enu_mps_ = drone::Vector3(0.0, 0.0, vertical_speed_mps_);
        acceleration_enu_ms2_ = drone::Vector3();

        if (!telemetry_log_stream_.is_open()) {
            setTelemetryLogFile(telemetry_log_file_);
        }
        if (telemetry_log_stream_.is_open() && !telemetry_csv_header_written_) {
            telemetry_log_stream_
                << "local_timestamp,sim_elapsed_s,sim_is_running,ground_locked,"
                << "sensed_altitude_m,sensed_position_enu_x_m,sensed_position_enu_y_m,sensed_position_enu_z_m,"
                << "sensed_gps_latitude_deg,sensed_gps_longitude_deg,sensed_gps_altitude_m,"
                << "sensed_gps_velocity_north_mps,sensed_gps_velocity_east_mps,sensed_gps_velocity_down_mps,"
                << "sensed_battery_voltage_v,sensed_battery_soc_percent,sensed_motor_temperature_c,sensed_motor_rpm,"
                << "altitude_m,position_enu_x_m,position_enu_y_m,position_enu_z_m,"
                << "velocity_enu_x_mps,velocity_enu_y_mps,velocity_enu_z_mps,"
                << "yaw_rad,pitch_rad,roll_rad,"
                << "target_altitude_m,target_error_m,p_component_rpm,i_component_rpm,d_component_rpm,"
                << "desired_rpm,common_motor_rpm,yaw_control_rpm,pitch_control_rpm,roll_control_rpm,"
                << "desired_motor_rpm_0,desired_motor_rpm_1,desired_motor_rpm_2,desired_motor_rpm_3,"
                << "battery_voltage_v,battery_soc_percent,motor_temperature_c,motor_rpm,motor_current_a,battery_capacity_mah,"
                << "gps_latitude_deg,gps_longitude_deg,gps_altitude_m,"
                << "gps_velocity_north_mps,gps_velocity_east_mps,gps_velocity_down_mps,"
                << "weather_total_ax,weather_total_ay,weather_total_az,"
                << "weather_steady_ax,weather_steady_ay,weather_steady_az,"
                << "weather_gust_ax,weather_gust_ay,weather_gust_az,"
                << "weather_turb_ax,weather_turb_ay,weather_turb_az\n";
            telemetry_csv_header_written_ = true;
        }
    }
}

void QuaroSimulation::onStop() {
    if (is_running_) {
        is_running_ = false;
        if (telemetry_log_stream_.is_open()) {
            telemetry_log_stream_.flush();
            telemetry_log_stream_.close();
        }
    }
}

void QuaroSimulation::onStep(double delta_time_s) {
    if (is_running_ && quad_) {
        elapsed_s_ += delta_time_s;
        uint64_t delta_ms = static_cast<uint64_t>(delta_time_s * 1000.0);

        // SIMULATION SIDE: Apply desired RPM to motors and compute physics
        auto& motors = quad_->getMotors();
        double battery_voltage = quad_->getBattery() ? quad_->getBattery()->getVoltageV() : 0.0;
        auto* battery = quad_->getBattery();

        bool has_per_motor_refs = false;
        for (double rpm_ref : desired_motor_rpm_each_) {
            if (rpm_ref > 0.0) {
                has_per_motor_refs = true;
                break;
            }
        }
        
        for (std::size_t i = 0; i < motors.size(); ++i) {
            auto& motor = motors[i];
            const double motor_rpm_ref = (has_per_motor_refs && i < desired_motor_rpm_each_.size())
                ? desired_motor_rpm_each_[i]
                : desired_rpm_;

            motor.setDesiredSpeedRPM(motor_rpm_ref);
            // Use battery-aware physics engine to update motor (includes depletion cutoff)
            if (battery) {
                drone::simulator::physics::MotorPhysics::updateMotorPhysics(motor, delta_ms, battery);
            } else {
                drone::simulator::physics::MotorPhysics::updateMotorPhysics(motor, delta_ms, battery_voltage);
            }
        }
        
        // Calculate total current draw and update battery
        double total_current = 0.0;
        for (const auto& motor : motors) {
            total_current += motor.getCurrentA();
        }
        
        // Update battery with total current draw
        if (quad_->getBattery()) {
            auto* battery_sim = dynamic_cast<drone::simulator::physics::BatterySim*>(quad_->getBattery());
            if (battery_sim) {
                battery_sim->setCurrentA(total_current);
                battery_sim->update(delta_ms);
            }
        }
        
        // Calculate thrust from all motors and update altitude
        double total_thrust_n = 0.0;
        drone::simulator::physics::ThrustModelParams thrust_params;
        thrust_params.kT = 1.5e-5;  // Thrust coefficient for small quadcopter props
        thrust_params.kQ = 1.5e-6;  // Torque coefficient (typically kQ = kT / 10)
        thrust_params.diameter_m = 0.3;  // Will be overridden by motor specs
        thrust_params.shape_coeff = 1.0;  // Will be overridden by motor specs
        
        for (const auto& motor : motors) {
            total_thrust_n += drone::simulator::physics::ThrustModel::computeThrustN(&motor, thrust_params);
        }
        
        // Calculate net force and acceleration in ENU coordinates
        const double GRAVITY_MS2 = 9.81;
        const double DAMPING_N_PER_MPS = 1.2;
        double total_weight_kg = quad_->getTotalWeightKg();
        const drone::Vector3 thrust_body_n(0.0, 0.0, total_thrust_n);
        const drone::Vector3 net_force_enu_n = drone::simulator::physics::computeNetForceEnu(
            thrust_body_n,
            attitude_ypr_rad_,
            total_weight_kg,
            velocity_enu_mps_,
            DAMPING_N_PER_MPS,
            GRAVITY_MS2);

        weather_sample_ = weather_model_.sample(elapsed_s_);
        const drone::Vector3 weather_force_enu_n = weather_sample_.total_accel_enu_ms2 * total_weight_kg;
        const drone::Vector3 net_force_with_weather_enu_n = net_force_enu_n + weather_force_enu_n;

        acceleration_enu_ms2_ = net_force_with_weather_enu_n * (1.0 / total_weight_kg);

        // Integrate translational dynamics
        const drone::Vector3 prev_position_enu_m = position_enu_m_;
        velocity_enu_mps_ += acceleration_enu_ms2_ * delta_time_s;
        position_enu_m_ += velocity_enu_mps_ * delta_time_s;

        // Ground clamp and lock (z=0): no movement allowed while grounded
        if (position_enu_m_.z <= 0.0) {
            position_enu_m_.x = prev_position_enu_m.x;
            position_enu_m_.y = prev_position_enu_m.y;
            position_enu_m_.z = 0.0;
            velocity_enu_mps_ = drone::Vector3(0.0, 0.0, 0.0);
            acceleration_enu_ms2_ = drone::Vector3(0.0, 0.0, 0.0);
        }

        // Legacy compatibility mirrors
        altitude_m_ = position_enu_m_.z;
        vertical_speed_mps_ = velocity_enu_mps_.z;
        
        // Update GPS from perfect simulator state
        if (quad_->getGPS()) {
            auto* gps_sim = dynamic_cast<drone::simulator::physics::GPSSim*>(quad_->getGPS());
            if (gps_sim) {
                gps_sim->setPerfectEnuState(position_enu_m_, velocity_enu_mps_);
            }
        }
        
        // Update temperature sensor
        if (quad_->getTemperatureSensor()) {
            quad_->getTemperatureSensor()->update();
        }
        
        // Update GPS
        if (quad_->getGPS()) {
            quad_->getGPS()->update();
        }
        
        // Get telemetry data
        double motor_temp = 0.0;
        double motor_rpm = 0.0;
        double motor_current = 0.0;
        
        if (!motors.empty()) {
            motor_temp = motors[0].getTemperatureC();
            motor_rpm = motors[0].getSpeedRPM();
            motor_current = motors[0].getCurrentA();
        }
        
        // Get battery data
        double battery_capacity = quad_->getBattery() ? quad_->getBattery()->getRemainingCapacityMah() : 0.0;
        double battery_soc = quad_->getBattery() ? quad_->getBattery()->getStateOfChargePercent() : 0.0;
        // battery_voltage already declared above
        
        // Get altitude and velocity from GPS
        double altitude_m = quad_->getAltitudeM();
        drone::Position3D gps_position;
        drone::Velocity3D gps_velocity;
        if (quad_->getGPS()) {
            gps_position = quad_->getGPS()->getPosition();
            gps_velocity = quad_->getGPS()->getVelocity();
        }
        
        if (telemetry_log_stream_.is_open()) {
            const bool ground_locked = position_enu_m_.z <= 0.0;
            telemetry_log_stream_ << std::fixed << std::setprecision(6)
                << localTimestampNow() << ","
                << elapsed_s_ << ","
                << (is_running_ ? 1 : 0) << ","
                << (ground_locked ? 1 : 0) << ","
                << sensed_altitude_m_ << ","
                << sensed_position_enu_x_m_ << ","
                << sensed_position_enu_y_m_ << ","
                << sensed_position_enu_z_m_ << ","
                << sensed_gps_latitude_deg_ << ","
                << sensed_gps_longitude_deg_ << ","
                << sensed_gps_altitude_m_ << ","
                << sensed_gps_velocity_north_mps_ << ","
                << sensed_gps_velocity_east_mps_ << ","
                << sensed_gps_velocity_down_mps_ << ","
                << sensed_battery_voltage_v_ << ","
                << sensed_battery_soc_percent_ << ","
                << sensed_motor_temperature_c_ << ","
                << sensed_motor_rpm_ << ","
                << altitude_m << ","
                << position_enu_m_.x << ","
                << position_enu_m_.y << ","
                << position_enu_m_.z << ","
                << velocity_enu_mps_.x << ","
                << velocity_enu_mps_.y << ","
                << velocity_enu_mps_.z << ","
                << attitude_ypr_rad_.yaw_rad << ","
                << attitude_ypr_rad_.pitch_rad << ","
                << attitude_ypr_rad_.roll_rad << ","
                << target_altitude_m_ << ","
                << target_error_m_ << ","
                << p_component_rpm_ << ","
                << i_component_rpm_ << ","
                << d_component_rpm_ << ","
                << desired_rpm_ << ","
                << common_motor_rpm_ << ","
                << yaw_control_rpm_ << ","
                << pitch_control_rpm_ << ","
                << roll_control_rpm_ << ","
                << desired_motor_rpm_each_[0] << ","
                << desired_motor_rpm_each_[1] << ","
                << desired_motor_rpm_each_[2] << ","
                << desired_motor_rpm_each_[3] << ","
                << battery_voltage << ","
                << battery_soc << ","
                << motor_temp << ","
                << motor_rpm << ","
                << motor_current << ","
                << battery_capacity << ","
                << gps_position.latitude_deg << ","
                << gps_position.longitude_deg << ","
                << gps_position.altitude_m << ","
                << gps_velocity.north_mps << ","
                << gps_velocity.east_mps << ","
                << gps_velocity.down_mps << ","
                << weather_sample_.total_accel_enu_ms2.x << ","
                << weather_sample_.total_accel_enu_ms2.y << ","
                << weather_sample_.total_accel_enu_ms2.z << ","
                << weather_sample_.steady_accel_enu_ms2.x << ","
                << weather_sample_.steady_accel_enu_ms2.y << ","
                << weather_sample_.steady_accel_enu_ms2.z << ","
                << weather_sample_.gust_accel_enu_ms2.x << ","
                << weather_sample_.gust_accel_enu_ms2.y << ","
                << weather_sample_.gust_accel_enu_ms2.z << ","
                << weather_sample_.turbulence_accel_enu_ms2.x << ","
                << weather_sample_.turbulence_accel_enu_ms2.y << ","
                << weather_sample_.turbulence_accel_enu_ms2.z << "\n";
        }
    }
}

std::shared_ptr<QuaroSimulation> QuadroSimulationFactory(
    std::string name,
    drone::model::components::ElecMotorSpecs emSpecs,
    drone::model::sensors::AnalogIOSpec aIOSpec,
    drone::model::components::BatterySpecs batterySpecs,
    drone::model::sensors::AnalogIOSpec tempIOSpec,
    drone::model::sensors::TemperatureSensorRanges tempSensorRanges,
    double temp_sensor_weight_kg,
    drone::model::components::GPSSensorSpecs gpsSpecs,
    double body_weight_kg,
    double blade_diameter_m,
    double blade_shape_coefficient,
    uint64_t steps,
    double dt_s) {
    (void)steps;
    (void)dt_s;
    
    // Create simulation object using new (since make_shared can't access protected constructor)
    auto sim = std::shared_ptr<QuaroSimulation>(new QuaroSimulation());
    
    // Create quadrocopter with simulated battery and GPS modules
    sim->quad_ = std::make_unique<drone::model::Quadrocopter>(
        drone::model::Quadrocopter::createWithBatterySim(
            name, emSpecs, aIOSpec, batterySpecs, tempIOSpec, tempSensorRanges, 
            temp_sensor_weight_kg, gpsSpecs, body_weight_kg, blade_diameter_m, 
            blade_shape_coefficient));

    if (sim->quad_ && sim->quad_->getGPS()) {
        auto* gps_sim = dynamic_cast<drone::simulator::physics::GPSSim*>(sim->quad_->getGPS());
        if (gps_sim) {
            gps_sim->setReferenceGeodetic(drone::Position3D(0.0, 0.0, 0.0));
        }
    }

    return sim;
}

}  // namespace drone::simulator;
