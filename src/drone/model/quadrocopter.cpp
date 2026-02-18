#include "drone/model/quadrocopter.h"

#include <string>
#include <utility>
#include <vector>

#include "simulator/physics/battery_sim.h"
#include "simulator/physics/gps_sim.h"

namespace drone::model {

namespace {
std::vector<components::ElecMotor> buildMotors(const std::string& name,
                                               const components::ElecMotorSpecs& motor_specs,
                                               const sensors::AnalogIOSpec& motor_io_spec,
                                               double blade_diameter_m,
                                               double blade_shape_coeff) {
    std::vector<components::ElecMotor> motors;
    motors.reserve(4);

    components::ElecMotorSpecs specs = motor_specs;
    specs.blade_diameter_m = blade_diameter_m;
    specs.blade_shape_coeff = blade_shape_coeff;

    for (int i = 0; i < 4; ++i) {
        motors.emplace_back(name + "_M" + std::to_string(i + 1), motor_io_spec, specs);
    }
    return motors;
}
}  // namespace

Quadrocopter::Quadrocopter(const std::string& name,
                           const components::ElecMotorSpecs& motor_specs,
                           const sensors::AnalogIOSpec& motor_io_spec,
                           std::unique_ptr<components::Battery_base> battery,
                           std::unique_ptr<sensors::TemperatureSensor> temperature_sensor,
                           std::unique_ptr<components::GPSModule_base> gps,
                           double body_weight_kg,
                           double blade_diameter_m,
                           double blade_shape_coeff,
                           std::unique_ptr<components::AltitudeController> alt_ctrl)
    : DroneBase(name,
                buildMotors(name, motor_specs, motor_io_spec, blade_diameter_m, blade_shape_coeff),
                std::move(battery),
                std::move(temperature_sensor),
                std::move(gps),
                body_weight_kg) {
                    alt_ctrl_ = std::move(alt_ctrl);
                }

Quadrocopter Quadrocopter::createWithBatterySim(const std::string& name,
                                                const components::ElecMotorSpecs& motor_specs,
                                                const sensors::AnalogIOSpec& motor_io_spec,
                                                const components::BatterySpecs& battery_specs,
                                                const sensors::AnalogIOSpec& temp_io_spec,
                                                const sensors::TemperatureSensorRanges& temp_ranges,
                                                double temp_sensor_weight_kg,
                                                const components::GPSSensorSpecs& gps_specs,
                                                double body_weight_kg,
                                                double blade_diameter_m,
                                                double blade_shape_coeff,
                                                const components::AltitudeController& alt_ctrl) {
    auto battery = std::make_unique<simulator::physics::BatterySim>(name + "_Battery", battery_specs);
    auto temperature_sensor = std::make_unique<sensors::TemperatureSensor>(
        name + "_TempSensor", temp_io_spec, temp_ranges, temp_sensor_weight_kg);
    auto gps = std::make_unique<simulator::physics::GPSSim>(name + "_GPS", gps_specs);
    auto altitude_controller = std::make_unique<components::AltitudeController>(alt_ctrl);

    return Quadrocopter(name,
                        motor_specs,
                        motor_io_spec,
                        std::move(battery),
                        std::move(temperature_sensor),
                        std::move(gps),
                        body_weight_kg,
                        blade_diameter_m,
                        blade_shape_coeff,
                        std::move(altitude_controller));
}

}  // namespace drone::model
