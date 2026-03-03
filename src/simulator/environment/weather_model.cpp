#include "simulator/environment/weather_model.h"

#include <cmath>

namespace drone::simulator::environment {

namespace {
constexpr double kTwoPi = 6.283185307179586;
}  // namespace

WeatherModel::WeatherModel()
    : rng_(42) {}

void WeatherModel::setConfig(const drone::simulator::config::WeatherConfig& weather_config) {
    config_ = weather_config;
    rng_.seed(config_.random_seed);
    turbulence_dist_x_ = std::normal_distribution<double>(0.0, config_.turbulence_std_enu_ms2.x);
    turbulence_dist_y_ = std::normal_distribution<double>(0.0, config_.turbulence_std_enu_ms2.y);
    turbulence_dist_z_ = std::normal_distribution<double>(0.0, config_.turbulence_std_enu_ms2.z);
}

WeatherSample WeatherModel::sample(double elapsed_s) {
    WeatherSample sample;
    if (!config_.enabled) {
        return sample;
    }

    sample.steady_accel_enu_ms2 = config_.steady_accel_enu_ms2;

    const double omega = kTwoPi * config_.gust_frequency_hz;
    sample.gust_accel_enu_ms2 = drone::Vector3(
        config_.gust_amplitude_enu_ms2.x * std::sin(omega * elapsed_s),
        config_.gust_amplitude_enu_ms2.y * std::sin(omega * elapsed_s + kTwoPi / 3.0),
        config_.gust_amplitude_enu_ms2.z * std::sin(omega * elapsed_s + 2.0 * kTwoPi / 3.0));

    sample.turbulence_accel_enu_ms2 = drone::Vector3(
        turbulence_dist_x_(rng_),
        turbulence_dist_y_(rng_),
        turbulence_dist_z_(rng_));

    sample.total_accel_enu_ms2 =
        sample.steady_accel_enu_ms2 + sample.gust_accel_enu_ms2 + sample.turbulence_accel_enu_ms2;

    return sample;
}

}  // namespace drone::simulator::environment
