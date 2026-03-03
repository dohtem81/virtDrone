#ifndef SIMULATOR_ENVIRONMENT_WEATHER_MODEL_H
#define SIMULATOR_ENVIRONMENT_WEATHER_MODEL_H

#include <random>

#include "drone/drone_data_types.h"
#include "simulator/config/weather_config.h"

namespace drone::simulator::environment {

struct WeatherSample {
    drone::Vector3 steady_accel_enu_ms2{};
    drone::Vector3 gust_accel_enu_ms2{};
    drone::Vector3 turbulence_accel_enu_ms2{};
    drone::Vector3 total_accel_enu_ms2{};
};

class WeatherModel {
public:
    WeatherModel();

    void setConfig(const drone::simulator::config::WeatherConfig& weather_config);
    WeatherSample sample(double elapsed_s);

private:
    drone::simulator::config::WeatherConfig config_{};
    std::mt19937 rng_;
    std::normal_distribution<double> turbulence_dist_x_{0.0, 0.0};
    std::normal_distribution<double> turbulence_dist_y_{0.0, 0.0};
    std::normal_distribution<double> turbulence_dist_z_{0.0, 0.0};
};

}  // namespace drone::simulator::environment

#endif  // SIMULATOR_ENVIRONMENT_WEATHER_MODEL_H
