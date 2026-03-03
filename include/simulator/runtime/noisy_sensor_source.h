#ifndef SIMULATOR_RUNTIME_NOISY_SENSOR_SOURCE_H
#define SIMULATOR_RUNTIME_NOISY_SENSOR_SOURCE_H

#include <algorithm>
#include <cmath>
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
        sensor_frame.gps_altitude_m += gps_vertical_noise_m_(rng_);
        sensor_frame.gps_latitude_deg += metersToLatitudeDeg(gps_horizontal_noise_m_(rng_));
        sensor_frame.gps_longitude_deg += metersToLongitudeDeg(
            gps_horizontal_noise_m_(rng_),
            sensor_frame.gps_latitude_deg);
        sensor_frame.gps_velocity_north_mps += gps_velocity_noise_mps_(rng_);
        sensor_frame.gps_velocity_east_mps += gps_velocity_noise_mps_(rng_);
        sensor_frame.gps_velocity_down_mps += gps_velocity_noise_mps_(rng_);
        sensor_frame.battery_voltage_v += battery_voltage_noise_v_(rng_);
        sensor_frame.motor_temperature_c += motor_temp_noise_c_(rng_);

        sensor_frame.altitude_m = std::max(0.0, sensor_frame.altitude_m);
        sensor_frame.gps_altitude_m = std::max(0.0, sensor_frame.gps_altitude_m);
        sensor_frame.battery_voltage_v = std::max(0.0, sensor_frame.battery_voltage_v);
        sensor_frame.battery_soc_percent = std::clamp(sensor_frame.battery_soc_percent, 0.0, 100.0);
        sensor_frame.motor_rpm = std::max(0.0, sensor_frame.motor_rpm);
        return sensor_frame;
    }

private:
    const drone::runtime::SensorSource& source_;

    static double metersToLatitudeDeg(double meters) {
        constexpr double kMetersPerDegLat = 111320.0;
        return meters / kMetersPerDegLat;
    }

    static double metersToLongitudeDeg(double meters, double latitude_deg) {
        constexpr double kDegToRad = 0.017453292519943295;
        constexpr double kMetersPerDegEquator = 111320.0;
        constexpr double kMinScale = 1.0e-6;
        const double scale = std::max(kMinScale, std::abs(std::cos(latitude_deg * kDegToRad)));
        return meters / (kMetersPerDegEquator * scale);
    }

    mutable std::mt19937 rng_{std::random_device{}()};
    mutable std::normal_distribution<double> altitude_noise_m_{0.0, 0.15};
    mutable std::normal_distribution<double> gps_horizontal_noise_m_{0.0, 1.5};
    mutable std::normal_distribution<double> gps_vertical_noise_m_{0.0, 2.5};
    mutable std::normal_distribution<double> gps_velocity_noise_mps_{0.0, 0.3};
    mutable std::normal_distribution<double> battery_voltage_noise_v_{0.0, 0.03};
    mutable std::normal_distribution<double> motor_temp_noise_c_{0.0, 0.2};
};

}  // namespace drone::simulator::runtime

#endif  // SIMULATOR_RUNTIME_NOISY_SENSOR_SOURCE_H
