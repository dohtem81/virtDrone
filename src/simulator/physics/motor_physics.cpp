#include "simulator/physics/motor_physics.h"
#include <algorithm>  // For std::clamp
#include "drone/model/utils.h"

namespace drone::simulator::physics {

void MotorPhysics::updateSpeed(drone::model::components::ElecMotor& motor, uint64_t delta_ms, double currentBattVoltageV) {
    double delta_s = delta_ms / 1000.0;
    double desiredRPM = motor.getDesiredSpeedRPM();
    double battDrainFactor = currentBattVoltageV / motor.getSpecs().nominal_voltage_v;
    double max_rpm = motor.getSpecs().max_speed_rpm * battDrainFactor;  // Scale max RPM by battery voltage factor
    desiredRPM = std::min(desiredRPM, max_rpm);  // Ensure desired RPM does not exceed scaled max RPM
    double currentRPM = motor.getSpeedRPM();

    // calculate delta RPMs
    double delta_rpm = desiredRPM - currentRPM;
    // delta RPM change based on max ramp rate and time, scaled by battery voltage factor
    double delta_rpm_allowed = motor.getMaxRampRateRPMPerS() * delta_s;

    double currentRPMChange = std::clamp(delta_rpm, -delta_rpm_allowed, delta_rpm_allowed);
    currentRPM += currentRPMChange;
    motor.setSpeedRPM(currentRPM);
}

void MotorPhysics::calculateCurrent(drone::model::components::ElecMotor& motor) {
    // Simple linear relationship: current increases with desired speed, adjusted by efficiency
    double normalized_speed = motor.getDesiredSpeedRPM() / motor.getSpecs().max_speed_rpm;
    double current_speed = motor.getSpeedRPM();
    double calculated_current = (current_speed / motor.getSpecs().max_speed_rpm) * motor.getSpecs().max_current_a / motor.getSpecs().efficiency;
    double current = std::clamp(
        calculated_current,
        0.0,
        motor.getSpecs().max_current_a
    );
    motor.setCurrentA(current);
}

void MotorPhysics::calculateCurrent(drone::model::components::ElecMotor& motor, drone::model::components::Battery_base* battery) {
    if (battery->getStateOfChargePercent() < 1.0) {
        motor.setCurrentA(0.0);
    } else {
        MotorPhysics::calculateCurrent(motor);
    }
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

void MotorPhysics::updateMotorPhysics(
        drone::model::components::ElecMotor& motor, 
        uint64_t delta_ms, 
        double currentBattVoltageV) {

    MotorPhysics::calculateCurrent(motor);
    MotorPhysics::calculateLosses(motor);
    MotorPhysics::updateTemperature(motor, delta_ms);
    MotorPhysics::updateSpeed(motor, delta_ms, currentBattVoltageV);
}

void MotorPhysics::updateMotorPhysics(
        drone::model::components::ElecMotor& motor, 
        uint64_t delta_ms, 
        drone::model::components::Battery_base* battery) {

    // if (battery->getStateOfChargePercent() < 1.0) {
    //     motor.setCurrentA(0.0);
    //     motor.setSpeedRPM(0.0);
    //     motor.setLossesW(0.0);
    // }
    MotorPhysics::updateSpeed(motor, delta_ms, battery->getStateOfChargePercent() < 1.0 ? 0.0 : battery->getVoltageV());
    MotorPhysics::calculateCurrent(motor, battery);
    MotorPhysics::calculateLosses(motor);
    MotorPhysics::updateTemperature(motor, delta_ms);
}

}  // namespace drone::simulator::physics