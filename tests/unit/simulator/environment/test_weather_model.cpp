#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "simulator/environment/weather_model.h"

TEST_CASE("WeatherModel returns zero sample when disabled", "[WeatherModel]") {
    drone::simulator::config::WeatherConfig config;
    config.enabled = false;

    drone::simulator::environment::WeatherModel model;
    model.setConfig(config);

    const auto sample = model.sample(1.0);

    REQUIRE(sample.total_accel_enu_ms2.x == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sample.total_accel_enu_ms2.y == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sample.total_accel_enu_ms2.z == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("WeatherModel combines steady and gust components deterministically without turbulence", "[WeatherModel]") {
    drone::simulator::config::WeatherConfig config;
    config.enabled = true;
    config.steady_accel_enu_ms2 = drone::Vector3(1.0, 2.0, 3.0);
    config.gust_amplitude_enu_ms2 = drone::Vector3(0.0, 0.0, 0.0);
    config.gust_frequency_hz = 0.5;
    config.turbulence_std_enu_ms2 = drone::Vector3(0.0, 0.0, 0.0);

    drone::simulator::environment::WeatherModel model;
    model.setConfig(config);

    const auto sample = model.sample(2.0);

    REQUIRE(sample.steady_accel_enu_ms2.x == Catch::Approx(1.0).margin(1e-12));
    REQUIRE(sample.steady_accel_enu_ms2.y == Catch::Approx(2.0).margin(1e-12));
    REQUIRE(sample.steady_accel_enu_ms2.z == Catch::Approx(3.0).margin(1e-12));
    REQUIRE(sample.gust_accel_enu_ms2.x == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sample.gust_accel_enu_ms2.y == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sample.gust_accel_enu_ms2.z == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sample.turbulence_accel_enu_ms2.x == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sample.turbulence_accel_enu_ms2.y == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sample.turbulence_accel_enu_ms2.z == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(sample.total_accel_enu_ms2.x == Catch::Approx(1.0).margin(1e-12));
    REQUIRE(sample.total_accel_enu_ms2.y == Catch::Approx(2.0).margin(1e-12));
    REQUIRE(sample.total_accel_enu_ms2.z == Catch::Approx(3.0).margin(1e-12));
}

TEST_CASE("WeatherModel turbulence is repeatable for same seed", "[WeatherModel]") {
    drone::simulator::config::WeatherConfig config;
    config.enabled = true;
    config.steady_accel_enu_ms2 = drone::Vector3(0.0, 0.0, 0.0);
    config.gust_amplitude_enu_ms2 = drone::Vector3(0.0, 0.0, 0.0);
    config.turbulence_std_enu_ms2 = drone::Vector3(0.2, 0.2, 0.2);
    config.random_seed = 123;

    drone::simulator::environment::WeatherModel model_a;
    drone::simulator::environment::WeatherModel model_b;
    model_a.setConfig(config);
    model_b.setConfig(config);

    const auto sample_a = model_a.sample(0.1);
    const auto sample_b = model_b.sample(0.1);

    REQUIRE(sample_a.turbulence_accel_enu_ms2.x == Catch::Approx(sample_b.turbulence_accel_enu_ms2.x).margin(1e-12));
    REQUIRE(sample_a.turbulence_accel_enu_ms2.y == Catch::Approx(sample_b.turbulence_accel_enu_ms2.y).margin(1e-12));
    REQUIRE(sample_a.turbulence_accel_enu_ms2.z == Catch::Approx(sample_b.turbulence_accel_enu_ms2.z).margin(1e-12));
}
