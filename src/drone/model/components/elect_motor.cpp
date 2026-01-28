#include "drone/model/components/elect_motor.h"
#include <algorithm>  // For std::clamp
#include "drone/model/utils.h"

namespace drone::model::components {

ElecMotor::ElecMotor(const std::string& name, const drone::model::sensors::AnalogIOSpec& spec, const ElecMotorSpecs& motor_specs)
    : drone::model::sensors::BaseSensor(name, spec),      // Initialize base class
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
                   drone::model::sensors::AnalogIOSpec(drone::model::sensors::AnalogIOSpec::IODirection::INPUT,
                                 drone::model::sensors::AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
                                 4000, 20000),
                   drone::model::sensors::TemperatureSensorRanges(0.0, 50.0))
{
    // No additional initialization needed
}

void ElecMotor::update() {
    // Update last update time
    last_update_time_ = std::chrono::steady_clock::now();
    // Simulation logic moved to MotorPhysics
}

void ElecMotor::setDesiredSpeedRPM(double speed_rpm) {
    desired_speed_rpm_ = std::clamp(speed_rpm, 0.0, specs_.max_speed_rpm);
}

double ElecMotor::calculateBatteryDrain(double time_s) const {
    // Energy = Power * time = (Voltage * Current) * time
    return voltage_v_ * current_a_ * time_s;
}

}  // namespace drone::model::components