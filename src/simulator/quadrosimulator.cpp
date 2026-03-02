#include "simulator/quadrosimulator.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/thrust_model.h"
#include "simulator/physics/gps_sim.h"

#include <algorithm>
#include <iomanip>
#include <iostream>

namespace drone::simulator {

drone::runtime::SensorFrame QuaroSimulation::readSensors() const {
    drone::runtime::SensorFrame sensor_frame;
    if (!quad_) {
        return sensor_frame;
    }

    sensor_frame.altitude_m = quad_->getAltitudeM();
    sensor_frame.battery_voltage_v = quad_->getBatteryVoltageV();
    sensor_frame.battery_soc_percent = quad_->getBatterySOC();
    sensor_frame.motor_temperature_c = quad_->getTemperatureC();

    const auto& motors = quad_->getMotors();
    sensor_frame.motor_rpm = motors.empty() ? 0.0 : motors[0].getSpeedRPM();
    return sensor_frame;
}

void QuaroSimulation::applyActuators(const drone::runtime::ActuatorFrame& actuator_frame) {
    desired_rpm_ = actuator_frame.desired_motor_rpm;
    target_altitude_m_ = actuator_frame.target_altitude_m;
    target_error_m_ = actuator_frame.target_error_m;
    p_component_rpm_ = actuator_frame.p_component_rpm;
    i_component_rpm_ = actuator_frame.i_component_rpm;
    d_component_rpm_ = actuator_frame.d_component_rpm;
    sensed_altitude_m_ = actuator_frame.sensed_altitude_m;
    sensed_battery_voltage_v_ = actuator_frame.sensed_battery_voltage_v;
    sensed_battery_soc_percent_ = actuator_frame.sensed_battery_soc_percent;
    sensed_motor_temperature_c_ = actuator_frame.sensed_motor_temperature_c;
    sensed_motor_rpm_ = actuator_frame.sensed_motor_rpm;
}

void QuaroSimulation::onStart() {
    if (!is_running_) {
        is_running_ = true;
        elapsed_s_ = 0.0;
    }
}

void QuaroSimulation::onStop() {
    if (is_running_) {
        is_running_ = false;
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
        
        for (auto& motor : motors) {
            motor.setDesiredSpeedRPM(desired_rpm_);
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
        
        // Calculate net force and acceleration
        const double GRAVITY_MS2 = 9.81;
        const double VERTICAL_DAMPING_N_PER_MPS = 1.2;
        double total_weight_kg = quad_->getTotalWeightKg();
        double weight_n = total_weight_kg * GRAVITY_MS2;
        double damping_force_n = VERTICAL_DAMPING_N_PER_MPS * vertical_speed_mps_;
        double net_force_n = total_thrust_n - weight_n - damping_force_n;
        double acceleration_ms2 = net_force_n / total_weight_kg;
        
        // Update vertical velocity and altitude
        vertical_speed_mps_ += acceleration_ms2 * delta_time_s;
        altitude_m_ += vertical_speed_mps_ * delta_time_s;
        
        // Prevent going below ground
        if (altitude_m_ < 0.0) {
            altitude_m_ = 0.0;
            vertical_speed_mps_ = 0.0;
        }
        
        // Update GPS altitude
        if (quad_->getGPS()) {
            auto* gps_sim = dynamic_cast<drone::simulator::physics::GPSSim*>(quad_->getGPS());
            if (gps_sim) {
                gps_sim->setAltitudeM(altitude_m_);
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
        
        // Get altitude from GPS
        double altitude_m = quad_->getAltitudeM();
        
        // Print telemetry with all data
        std::cout << std::fixed << std::setprecision(2)
                  << "T: " << std::setw(7) << elapsed_s_ << "s"
              << " | S/P Alt: " << std::setw(8) << sensed_altitude_m_ << " / " << std::setw(8) << altitude_m << "m"
              << " | S/P RPM: " << std::setw(8) << sensed_motor_rpm_ << " / " << std::setw(8) << motor_rpm
              << " | S/P SOC: " << std::setw(6) << sensed_battery_soc_percent_ << " / " << std::setw(6) << battery_soc << "%"
              << " | S/P V: " << std::setw(5) << sensed_battery_voltage_v_ << " / " << std::setw(5) << battery_voltage << "V"
              << " | S/P T: " << std::setw(6) << sensed_motor_temperature_c_ << " / " << std::setw(6) << motor_temp << "C"
              << " | Curr: " << std::setw(6) << motor_current << "A"
              << " | Batt: " << std::setw(7) << battery_capacity << "mAh"
              << " | TgtAlt: " << std::setw(8) << target_altitude_m_ << "m"
              << " | RefRPM: " << std::setw(8) << desired_rpm_
                  << " | TgtErr: " << std::setw(8) << target_error_m_ << "m"
                  << " | P: " << std::setw(8) << p_component_rpm_
                  << " | I: " << std::setw(8) << i_component_rpm_
                  << " | D: " << std::setw(8) << d_component_rpm_
                  << std::endl;
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

    return sim;
}

}  // namespace drone::simulator;
