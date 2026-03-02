#ifndef SIMULATOR_RUNTIME_NOISY_SENSOR_SOURCE_H
#define SIMULATOR_RUNTIME_NOISY_SENSOR_SOURCE_H

#include <algorithm>
#include <random>

#include "drone/runtime/real_drone.h"

namespace drone::simulator::runtime {

class NoisySensorSource final : public drone::runtime::SensorSource {
public:
    explicit NoisySensorSource(const drone::runtime::SensorSource& source)
        : source_(source) {}

    drone::runtime::SensorFrame readSensors() const override {
        drone::runtime::SensorFrame sensor_frame = source_.readSensors();

        sensor_frame.altitude_m += altitude_noise_m_(rng_);
        sensor_frame.battery_voltage_v += battery_voltage_noise_v_(rng_);
        sensor_frame.motor_temperature_c += motor_temp_noise_c_(rng_);

        sensor_frame.altitude_m = std::max(0.0, sensor_frame.altitude_m);
        sensor_frame.battery_voltage_v = std::max(0.0, sensor_frame.battery_voltage_v);
        sensor_frame.battery_soc_percent = std::clamp(sensor_frame.battery_soc_percent, 0.0, 100.0);
        sensor_frame.motor_rpm = std::max(0.0, sensor_frame.motor_rpm);
        return sensor_frame;
    }

private:
    const drone::runtime::SensorSource& source_;

    mutable std::mt19937 rng_{std::random_device{}()};
    mutable std::normal_distribution<double> altitude_noise_m_{0.0, 0.15};
    mutable std::normal_distribution<double> battery_voltage_noise_v_{0.0, 0.03};
    mutable std::normal_distribution<double> motor_temp_noise_c_{0.0, 0.2};
};

}  // namespace drone::simulator::runtime

#endif  // SIMULATOR_RUNTIME_NOISY_SENSOR_SOURCE_H
