#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "drone/runtime/real_drone.h"
#include "simulator/runtime/noisy_sensor_source.h"

namespace {

class ConstantSensorSource final : public drone::runtime::SensorSource {
public:
    explicit ConstantSensorSource(const drone::runtime::SensorFrame& frame)
        : frame_(frame) {}

    drone::runtime::SensorFrame readSensors() const override {
        return frame_;
    }

private:
    drone::runtime::SensorFrame frame_;
};

struct RunningStats {
    void add(double value) {
        ++count;
        const double delta = value - mean;
        mean += delta / static_cast<double>(count);
        const double delta2 = value - mean;
        m2 += delta * delta2;
    }

    double variance() const {
        if (count < 2) {
            return 0.0;
        }
        return m2 / static_cast<double>(count - 1);
    }

    int count = 0;
    double mean = 0.0;
    double m2 = 0.0;
};

}  // namespace

TEST_CASE("NoisySensorSource GPS noise has near-zero bias and non-zero variance", "[NoisySensorSource][GPS]") {
    drone::runtime::SensorFrame base_frame;
    base_frame.gps_latitude_deg = 52.2297;
    base_frame.gps_longitude_deg = 21.0122;
    base_frame.gps_altitude_m = 120.0;
    base_frame.gps_velocity_north_mps = 3.0;
    base_frame.gps_velocity_east_mps = -1.5;
    base_frame.gps_velocity_down_mps = 0.2;

    ConstantSensorSource perfect_source(base_frame);
    drone::simulator::runtime::NoisySensorSource noisy_source(perfect_source);

    constexpr int kSamples = 2000;
    constexpr double kMetersPerDegLat = 111320.0;
    constexpr double kDegToRad = 0.017453292519943295;
    const double meters_per_deg_lon = kMetersPerDegLat * std::cos(base_frame.gps_latitude_deg * kDegToRad);

    RunningStats lat_err_m;
    RunningStats lon_err_m;
    RunningStats alt_err_m;
    RunningStats vel_n_err_mps;
    RunningStats vel_e_err_mps;
    RunningStats vel_d_err_mps;

    for (int i = 0; i < kSamples; ++i) {
        const auto sample = noisy_source.readSensors();

        lat_err_m.add((sample.gps_latitude_deg - base_frame.gps_latitude_deg) * kMetersPerDegLat);
        lon_err_m.add((sample.gps_longitude_deg - base_frame.gps_longitude_deg) * meters_per_deg_lon);
        alt_err_m.add(sample.gps_altitude_m - base_frame.gps_altitude_m);
        vel_n_err_mps.add(sample.gps_velocity_north_mps - base_frame.gps_velocity_north_mps);
        vel_e_err_mps.add(sample.gps_velocity_east_mps - base_frame.gps_velocity_east_mps);
        vel_d_err_mps.add(sample.gps_velocity_down_mps - base_frame.gps_velocity_down_mps);
    }

    REQUIRE(std::abs(lat_err_m.mean) < 0.25);
    REQUIRE(std::abs(lon_err_m.mean) < 0.25);
    REQUIRE(std::abs(alt_err_m.mean) < 0.40);
    REQUIRE(std::abs(vel_n_err_mps.mean) < 0.05);
    REQUIRE(std::abs(vel_e_err_mps.mean) < 0.05);
    REQUIRE(std::abs(vel_d_err_mps.mean) < 0.05);

    REQUIRE(lat_err_m.variance() > 0.25);
    REQUIRE(lon_err_m.variance() > 0.25);
    REQUIRE(alt_err_m.variance() > 2.0);
    REQUIRE(vel_n_err_mps.variance() > 0.02);
    REQUIRE(vel_e_err_mps.variance() > 0.02);
    REQUIRE(vel_d_err_mps.variance() > 0.02);

    const double lat_std_m = std::sqrt(lat_err_m.variance());
    const double lon_std_m = std::sqrt(lon_err_m.variance());
    const double alt_std_m = std::sqrt(alt_err_m.variance());
    const double vel_n_std_mps = std::sqrt(vel_n_err_mps.variance());
    const double vel_e_std_mps = std::sqrt(vel_e_err_mps.variance());
    const double vel_d_std_mps = std::sqrt(vel_d_err_mps.variance());

    REQUIRE(lat_std_m > 0.8);
    REQUIRE(lat_std_m < 2.4);
    REQUIRE(lon_std_m > 0.8);
    REQUIRE(lon_std_m < 2.4);
    REQUIRE(alt_std_m > 1.6);
    REQUIRE(alt_std_m < 3.6);
    REQUIRE(vel_n_std_mps > 0.15);
    REQUIRE(vel_n_std_mps < 0.55);
    REQUIRE(vel_e_std_mps > 0.15);
    REQUIRE(vel_e_std_mps < 0.55);
    REQUIRE(vel_d_std_mps > 0.15);
    REQUIRE(vel_d_std_mps < 0.55);
}
