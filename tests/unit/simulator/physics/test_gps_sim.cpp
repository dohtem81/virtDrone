#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "simulator/physics/gps_sim.h"

namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kRadToDeg = 57.29577951308232;

}  // namespace

TEST_CASE("GPSSim converts ENU position to geodetic using reference", "[GPSSim]") {
    const drone::model::components::GPSSensorSpecs specs;
    drone::simulator::physics::GPSSim gps("sim-gps", specs);

    const drone::Position3D reference(10.0, 20.0, 100.0);
    gps.setReferenceGeodetic(reference);

    const drone::Vector3 position_enu_m(10.0, 20.0, 30.0);
    const drone::Vector3 velocity_enu_mps(1.0, 2.0, 3.0);
    gps.setPerfectEnuState(position_enu_m, velocity_enu_mps);

    const auto position = gps.getPosition();
    const auto velocity = gps.getVelocity();

    const double expected_lat_deg = reference.latitude_deg + (20.0 / kEarthRadiusM) * kRadToDeg;
    const double expected_lon_deg = reference.longitude_deg + (10.0 / (kEarthRadiusM * std::cos(reference.latitude_deg * 0.017453292519943295))) * kRadToDeg;

    REQUIRE(position.latitude_deg == Catch::Approx(expected_lat_deg).margin(1e-9));
    REQUIRE(position.longitude_deg == Catch::Approx(expected_lon_deg).margin(1e-9));
    REQUIRE(position.altitude_m == Catch::Approx(130.0).margin(1e-9));

    REQUIRE(velocity.north_mps == Catch::Approx(2.0).margin(1e-9));
    REQUIRE(velocity.east_mps == Catch::Approx(1.0).margin(1e-9));
    REQUIRE(velocity.down_mps == Catch::Approx(-3.0).margin(1e-9));
}

TEST_CASE("GPSSim setAltitudeM keeps latitude/longitude unchanged", "[GPSSim]") {
    const drone::model::components::GPSSensorSpecs specs;
    drone::simulator::physics::GPSSim gps("sim-gps", specs);

    gps.setReferenceGeodetic(drone::Position3D(10.0, 20.0, 0.0));
    gps.setPerfectEnuState(drone::Vector3(5.0, 8.0, 2.0), drone::Vector3());

    const auto before = gps.getPosition();
    gps.setAltitudeM(42.0);
    const auto after = gps.getPosition();

    REQUIRE(after.latitude_deg == Catch::Approx(before.latitude_deg).margin(1e-12));
    REQUIRE(after.longitude_deg == Catch::Approx(before.longitude_deg).margin(1e-12));
    REQUIRE(after.altitude_m == Catch::Approx(42.0).margin(1e-12));
}
