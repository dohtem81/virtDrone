#include "drone/model/quadrocopter.h"

#include <utility>
#include <vector>

#include "simulator/physics/battery_sim.h"

namespace drone::model {

Quadrocopter::Quadrocopter(const std::string& name,
                           const components::ElecMotorSpecs& motor_specs,
                           const sensors::AnalogIOSpec& motor_io_spec,
                           std::unique_ptr<components::Battery_base> battery,
                           std::unique_ptr<sensors::TemperatureSensor> temperature_sensor)
    : DroneBase(
          name,
          [&]() {
              std::vector<components::ElecMotor> motors;
              motors.reserve(kMotorCount);
              motors.emplace_back(name + "_M1", motor_io_spec, motor_specs);
              motors.emplace_back(name + "_M2", motor_io_spec, motor_specs);
              motors.emplace_back(name + "_M3", motor_io_spec, motor_specs);
              motors.emplace_back(name + "_M4", motor_io_spec, motor_specs);
              return motors;
          }(),
          std::move(battery),
          std::move(temperature_sensor)) {}

Quadrocopter Quadrocopter::createWithBatterySim(const std::string& name,
                                                const components::ElecMotorSpecs& motor_specs,
                                                const sensors::AnalogIOSpec& motor_io_spec,
                                                const components::BatterySpecs& battery_specs,
                                                const sensors::AnalogIOSpec& temp_io_spec,
                                                const sensors::TemperatureSensorRanges& temp_ranges) {
    auto battery = std::make_unique<drone::simulator::physics::BatterySim>(name + "_Battery", battery_specs);
    auto temp_sensor = std::make_unique<sensors::TemperatureSensor>(name + "_Temp", temp_io_spec, temp_ranges);

    return Quadrocopter(name, motor_specs, motor_io_spec, std::move(battery), std::move(temp_sensor));
}

}  // namespace drone::model
