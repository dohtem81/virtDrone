#include "drone/model/components/elect_motor.h"
#include <algorithm>  // For std::clamp
#include "drone/model/utils.h"

ElecMotor::ElecMotor(const std::string& name, const AnalogIOSpec& spec, const ElecMotorSpecs& motor_specs)
    : BaseSensor(name, spec),      // Initialize base class
      specs_(motor_specs),         // Initialize motor specs
      speed_rpm_(0.0),             // Initial speed is 0
      desired_speed_rpm_(0.0),     // Initial desired speed is 0
      current_a_(0.0),             // Initial current is 0
      voltage_v_(specs_.nominal_voltage_v),  // Set voltage to nominal
      temperature_c_(25.0),        // Assume ambient temperature is 25°C
      losses_w_(0.0),              // Initial losses are 0
      ambient_temp_c_(25.0),       // Ambient temperature
      last_update_time_(std::chrono::steady_clock::now()),  // Initialize last update time
      temp_sensor_(name + "_TempSensor",
                   AnalogIOSpec(AnalogIOSpec::IODirection::INPUT,
                                 AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
                                 4000, 20000),
                   TemperatureSensorRanges(0.0, 50.0))
{
    // No additional initialization needed
}

void ElecMotor::calculateCurrent() {
    // Simple linear relationship: current increases with desired speed, adjusted by efficiency
    // Current = (desired_speed / max_speed) * max_current / efficiency
    // Clamp to ensure it doesn't exceed max_current
    double normalized_speed = desired_speed_rpm_ / specs_.max_speed_rpm;
    current_a_ = std::clamp(normalized_speed * specs_.max_current_a / specs_.efficiency, 0.0, specs_.max_current_a);
}

void ElecMotor::calculateLosses() {
    // Losses = Input power - Output power
    // Input power = Voltage * Current
    // Output power = Input power * Efficiency
    // So Losses = Input power * (1 - Efficiency)
    double input_power = voltage_v_ * current_a_;
    losses_w_ = input_power * (1.0 - specs_.efficiency);
}

void ElecMotor::updateTemperature(uint64_t delta_ms) {
    // Simple time-based temperature calculation: integrate temperature change over time
    // dT/dt = (losses * thermal_resistance - (T - ambient)) / thermal_time_constant
    // For simplicity, assume thermal_time_constant = 10.0 s, and delta_time = 1.0 s per update
    // Approximation: T_new = T + (target_T - T) * (delta_time / tau)
    double target_temp = ambient_temp_c_ + losses_w_ * specs_.thermal_resistance;
    double tau = 10.0;  // Thermal time constant in seconds (can be added to specs if needed)
    double delta_s = delta_ms / 1000.0;       // Convert to seconds for calculation
    temperature_c_ += (target_temp - temperature_c_) * (delta_s / tau);

    // Update internal temperature sensor reading
    // Map temperature to counts for the sensor
    temp_sensor_.setLastCountsReading(
        static_cast<uint64_t>(Utils::mapRange(
            temperature_c_,
            temp_sensor_.getRanges()->min_temperature,
            temp_sensor_.getRanges()->max_temperature,
            temp_sensor_.getType().counts_range.min,
            temp_sensor_.getType().counts_range.max
        ))
    );
}

void ElecMotor::update() {
    // Calculate real-time delta since last update using the system clock
    auto now = std::chrono::steady_clock::now();
    auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_);
    last_update_time_ = now;

    // Call the parameterized update with the calculated delta in milliseconds
    update(delta_time.count());
}

void ElecMotor::update(uint64_t delta_time) {
    // Advance the last update time by the provided delta_time (for simulation/manual stepping)
    last_update_time_ += std::chrono::milliseconds(delta_time);

    // Update speed to approach desired speed (simple ramp)
    // For simulation, speed ramps towards desired_speed
    double ramp_rate = 1000.0;  // RPM per second, adjustable
    if (speed_rpm_ < desired_speed_rpm_) {
        speed_rpm_ += ramp_rate * (delta_time / 1000.0);
        if (speed_rpm_ > desired_speed_rpm_) speed_rpm_ = desired_speed_rpm_;
    } else if (speed_rpm_ > desired_speed_rpm_) {
        speed_rpm_ -= ramp_rate * (delta_time / 1000.0);
        if (speed_rpm_ < desired_speed_rpm_) speed_rpm_ = desired_speed_rpm_;
    }

    calculateCurrent();
    calculateLosses();
    updateTemperature(delta_time);
}

void ElecMotor::setDesiredSpeedRPM(double speed_rpm) {
    desired_speed_rpm_ = std::clamp(speed_rpm, 0.0, specs_.max_speed_rpm);
}

double ElecMotor::calculateBatteryDrain(double time_s) const {
    // Energy = Power * time = (Voltage * Current) * time
    return voltage_v_ * current_a_ * time_s;
}