#include "simulator/quadrosimulator.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/thrust_model.h"
#include "simulator/physics/gps_sim.h"

#include <algorithm>
#include <iomanip>
#include <iostream>

namespace drone::simulator {

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
        
        // PHYSICAL DRONE SIDE: Altitude controller computes desired RPM based on sensor readings
        double desired_rpm = 0.0;
        auto* alt_ctrl = quad_->getAltitudeController();
        if (alt_ctrl) {
            // Read current altitude from GPS sensor (physical drone perspective)
            double current_altitude_m = quad_->getAltitudeM();
            
            // Read current motor RPM
            auto& motors = quad_->getMotors();
            double current_rpm = motors.empty() ? 0.0 : motors[0].getSpeedRPM();
            
            // Controller computes RPM reference to maintain target altitude
            alt_ctrl->update(current_altitude_m, alt_ctrl->getAltitudeRefInUse(), 
                           current_rpm, desired_rpm, delta_time_s);
        } else {
            // Fallback: Ramp motors from 0% to 100% over 2 seconds (no controller)
            double ramp_time_s = 2.0;
            double throttle = std::min(1.0, elapsed_s_ / ramp_time_s);
            desired_rpm = throttle * 15000.0;  // Max RPM from motor specs
        }
        
        // SIMULATION SIDE: Apply desired RPM to motors and compute physics
        auto& motors = quad_->getMotors();
        double battery_voltage = quad_->getBattery() ? quad_->getBattery()->getVoltageV() : 0.0;
        
        for (auto& motor : motors) {
            motor.setDesiredSpeedRPM(desired_rpm);
            // Use physics engine to update motor
            drone::simulator::physics::MotorPhysics::updateMotorPhysics(motor, delta_ms, battery_voltage);
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
        double total_weight_kg = quad_->getTotalWeightKg();
        double weight_n = total_weight_kg * GRAVITY_MS2;
        double net_force_n = total_thrust_n - weight_n;
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
                  << " | Alt: " << std::setw(8) << altitude_m << "m"
                  << " | RefRPM: " << std::setw(8) << desired_rpm
                  << " | RPM: " << std::setw(8) << motor_rpm
                  << " | Curr: " << std::setw(6) << motor_current << "A"
                  << " | Batt: " << std::setw(7) << battery_capacity << "mAh"
                  << " | SOC: " << std::setw(6) << battery_soc << "%"
                  << " | V: " << std::setw(5) << battery_voltage << "V"
                  << " | T: " << std::setw(6) << motor_temp << "C"
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
    drone::model::components::AltitudeController altitudeController,
    uint64_t steps,
    double dt_s) {
    
    // Create simulation object using new (since make_shared can't access protected constructor)
    auto sim = std::shared_ptr<QuaroSimulation>(new QuaroSimulation());
    
    // Create quadrocopter with simulated battery and GPS modules
    sim->quad_ = std::make_unique<drone::model::Quadrocopter>(
        drone::model::Quadrocopter::createWithBatterySim(
            name, emSpecs, aIOSpec, batterySpecs, tempIOSpec, tempSensorRanges, 
            temp_sensor_weight_kg, gpsSpecs, body_weight_kg, blade_diameter_m, 
            blade_shape_coefficient, altitudeController));

    return sim;
}

}  // namespace drone::simulator;
