#include "simulator/physics/motor_physics.h"
#include <algorithm>  // For std::clamp
#include "drone/model/utils.h"

namespace drone::simulator::physics {

void MotorPhysics::updateSpeed(drone::model::components::ElecMotor& motor, uint64_t delta_ms) {
    double ramp_rate = 1000.0;  // RPM per second, adjustable
    double delta_s = delta_ms / 1000.0;
    double desired = motor.getDesiredSpeedRPM();
    double current = motor.getSpeedRPM();
    if (current < desired) {
        current += ramp_rate * delta_s;
        if (current > desired) current = desired;
    } else if (current > desired) {
        current -= ramp_rate * delta_s;
        if (current < desired) current = desired;
    }
    motor.setSpeedRPM(current);
}

void MotorPhysics::calculateCurrent(drone::model::components::ElecMotor& motor) {
    // Simple linear relationship: current increases with desired speed, adjusted by efficiency
    // Current = (desired_speed / max_speed) * max_current / efficiency
    // Clamp to ensure it doesn't exceed max_current
    double normalized_speed = motor.getDesiredSpeedRPM() / motor.getSpecs().max_speed_rpm;
    double current = std::clamp(normalized_speed * motor.getSpecs().max_current_a / motor.getSpecs().efficiency, 0.0, motor.getSpecs().max_current_a);
    motor.setCurrentA(current);
}

void MotorPhysics::calculateLosses(drone::model::components::ElecMotor& motor) {
    // Losses = Input power - Output power
    // Input power = Voltage * Current
    // Output power = Input power * Efficiency
    // So Losses = Input power * (1 - Efficiency)
    double input_power = motor.getVoltageV() * motor.getCurrentA();
    double losses = input_power * (1.0 - motor.getSpecs().efficiency);
    motor.setLossesW(losses);
}

void MotorPhysics::updateTemperature(drone::model::components::ElecMotor& motor, uint64_t delta_ms) {
    // Simple time-based temperature calculation: integrate temperature change over time
    // dT/dt = (losses * thermal_resistance - (T - ambient)) / thermal_time_constant
    // For simplicity, assume thermal_time_constant = 10.0 s, and delta_time = 1.0 s per update
    // Approximation: T_new = T + (target_T - T) * (delta_time / tau)
    double target_temp = motor.getAmbientTempC() + motor.getLossesW() * motor.getSpecs().thermal_resistance;
    double tau = 10.0;  // Thermal time constant in seconds (can be added to specs if needed)
    double delta_s = delta_ms / 1000.0;       // Convert to seconds for calculation
    double new_temp = motor.getTemperatureC() + (target_temp - motor.getTemperatureC()) * (delta_s / tau);
    motor.setTemperatureC(new_temp);

    // Update internal temperature sensor reading
    // Map temperature to counts for the sensor
    motor.getTempSensor().setLastCountsReading(
        static_cast<uint64_t>(Utils::mapRange(
            new_temp,
            motor.getTempSensor().getRanges()->min_temperature,
            motor.getTempSensor().getRanges()->max_temperature,
            motor.getTempSensor().getType().counts_range.min,
            motor.getTempSensor().getType().counts_range.max
        ))
    );
}

double MotorPhysics::calculateBatteryDrain(const drone::model::components::ElecMotor& motor, double time_s) {
    // Energy = Power * time = (Voltage * Current) * time
    return motor.getVoltageV() * motor.getCurrentA() * time_s;
}

void MotorPhysics::updateMotorPhysics(drone::model::components::ElecMotor& motor, uint64_t delta_ms) {
    MotorPhysics::calculateCurrent(motor);
    MotorPhysics::calculateLosses(motor);
    MotorPhysics::updateTemperature(motor, delta_ms);
    MotorPhysics::updateSpeed(motor, delta_ms);
}

}  // namespace drone::simulator::physics