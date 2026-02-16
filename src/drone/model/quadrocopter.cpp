#include "drone/model/quadrocopter.h"

#include <utility>
#include <vector>

#include "simulator/physics/battery_sim.h"
#include "simulator/physics/gps_sim.h"

namespace drone::model {

Quadrocopter::Quadrocopter(const std::string& name,
                           const components::ElecMotorSpecs& motor_specs,
                           const sensors::AnalogIOSpec& motor_io_spec,
                           std::unique_ptr<components::Battery_base> battery,
                           std::unique_ptr<sensors::TemperatureSensor> temperature_sensor,
                           std::unique_ptr<components::GPSModule_base> gps,
                           double body_weight_kg)
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
          std::move(temperature_sensor),
          std::move(gps),
          body_weight_kg) {}

Quadrocopter Quadrocopter::createWithBatterySim(const std::string& name,
                                                const components::ElecMotorSpecs& motor_specs,
                                                const sensors::AnalogIOSpec& motor_io_spec,
                                                const components::BatterySpecs& battery_specs,
                                                const sensors::AnalogIOSpec& temp_io_spec,
                                                const sensors::TemperatureSensorRanges& temp_ranges,
                                                double temp_sensor_weight_kg,
                                                const components::GPSSensorSpecs& gps_specs,
                                                double body_weight_kg) {
    auto battery = std::make_unique<drone::simulator::physics::BatterySim>(name + "_Battery", battery_specs);
    auto temp_sensor = std::make_unique<sensors::TemperatureSensor>(name + "_Temp", temp_io_spec, temp_ranges, temp_sensor_weight_kg);
    auto gps = std::make_unique<drone::simulator::physics::GPSSim>(name + "_GPS", gps_specs);

    return Quadrocopter(name, motor_specs, motor_io_spec, std::move(battery), std::move(temp_sensor), std::move(gps), body_weight_kg);
}

}  // namespace drone::model
