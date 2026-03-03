#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <string>

#include "simulator/config/weather_config.h"

TEST_CASE("WeatherConfig loads weather block from YAML file", "[WeatherConfig]") {
    const std::filesystem::path temp_file = std::filesystem::temp_directory_path() / "virtDrone_weather_test.yaml";

    {
        std::ofstream out(temp_file);
        out << "weather:\n";
        out << "  enabled: true\n";
        out << "  steady_accel_enu_ms2: [1.1, -2.2, 3.3]\n";
        out << "  gust_amplitude_enu_ms2: [0.4, 0.5, 0.6]\n";
        out << "  gust_frequency_hz: 0.7\n";
        out << "  turbulence_std_enu_ms2: [0.01, 0.02, 0.03]\n";
        out << "  random_seed: 99\n";
    }

    drone::simulator::config::WeatherConfig config;
    const bool loaded = config.loadFromFile(temp_file.string());

    std::filesystem::remove(temp_file);

    REQUIRE(loaded);
    REQUIRE(config.enabled);
    REQUIRE(config.steady_accel_enu_ms2.x == Catch::Approx(1.1));
    REQUIRE(config.steady_accel_enu_ms2.y == Catch::Approx(-2.2));
    REQUIRE(config.steady_accel_enu_ms2.z == Catch::Approx(3.3));
    REQUIRE(config.gust_amplitude_enu_ms2.x == Catch::Approx(0.4));
    REQUIRE(config.gust_amplitude_enu_ms2.y == Catch::Approx(0.5));
    REQUIRE(config.gust_amplitude_enu_ms2.z == Catch::Approx(0.6));
    REQUIRE(config.gust_frequency_hz == Catch::Approx(0.7));
    REQUIRE(config.turbulence_std_enu_ms2.x == Catch::Approx(0.01));
    REQUIRE(config.turbulence_std_enu_ms2.y == Catch::Approx(0.02));
    REQUIRE(config.turbulence_std_enu_ms2.z == Catch::Approx(0.03));
    REQUIRE(config.random_seed == 99);
}

TEST_CASE("WeatherConfig keeps defaults when weather block is missing", "[WeatherConfig]") {
    const std::filesystem::path temp_file = std::filesystem::temp_directory_path() / "virtDrone_weather_default_test.yaml";

    {
        std::ofstream out(temp_file);
        out << "dummy: true\n";
    }

    drone::simulator::config::WeatherConfig config;
    const bool loaded = config.loadFromFile(temp_file.string());

    std::filesystem::remove(temp_file);

    REQUIRE(loaded);
    REQUIRE_FALSE(config.enabled);
    REQUIRE(config.steady_accel_enu_ms2.x == Catch::Approx(0.0));
    REQUIRE(config.steady_accel_enu_ms2.y == Catch::Approx(0.0));
    REQUIRE(config.steady_accel_enu_ms2.z == Catch::Approx(0.0));
    REQUIRE(config.random_seed == 42);
}
