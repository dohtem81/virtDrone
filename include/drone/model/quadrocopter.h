#ifndef QUADROCOPTER_H
#define QUADROCOPTER_H

#include <memory>
#include <string>

#include "drone/model/drone_base.h"

namespace drone::model {

/**
 * @brief Quadrocopter model with four motors.
 */
class Quadrocopter final : public DroneBase {
public:
    Quadrocopter(const std::string& name,
                const components::ElecMotorSpecs& motor_specs,
                const sensors::AnalogIOSpec& motor_io_spec,
                std::unique_ptr<components::Battery_base> battery,
                std::unique_ptr<sensors::TemperatureSensor> temperature_sensor,
                std::unique_ptr<components::GPSModule_base> gps,
                double body_weight_kg,
                double blade_diameter_m,
                double blade_shape_coeff);

    static Quadrocopter createWithBatterySim(const std::string& name,
                                            const components::ElecMotorSpecs& motor_specs,
                                            const sensors::AnalogIOSpec& motor_io_spec,
                                            const components::BatterySpecs& battery_specs,
                                            const sensors::AnalogIOSpec& temp_io_spec,
                                            const sensors::TemperatureSensorRanges& temp_ranges,
                                            double temp_sensor_weight_kg = 0.02,
                                            const components::GPSSensorSpecs& gps_specs = components::GPSSensorSpecs(),
                                            double body_weight_kg = 0.0,
                                            double blade_diameter_m = 0.3,
                                            double blade_shape_coeff = 1.0);

    static constexpr size_t kMotorCount = 4;
};

}  // namespace drone::model

#endif  // QUADROCOPTER_H
