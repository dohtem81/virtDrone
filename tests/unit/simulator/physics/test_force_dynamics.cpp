#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "drone/drone_data_types.h"
#include "simulator/physics/force_dynamics.h"

#include <cmath>

TEST_CASE("Body Z thrust projects to ENU Z at zero attitude", "[ForceDynamics]") {
    const drone::Vector3 thrust_body_n(0.0, 0.0, 12.0);
    const drone::AttitudeYPR attitude(0.0, 0.0, 0.0);

    const drone::Vector3 thrust_enu_n = drone::simulator::physics::rotateBodyToEnu(thrust_body_n, attitude);

    REQUIRE(thrust_enu_n.x == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(thrust_enu_n.y == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(thrust_enu_n.z == Catch::Approx(12.0).margin(1e-9));
}

TEST_CASE("Pitch rotates body Z thrust into ENU X", "[ForceDynamics]") {
    const drone::Vector3 thrust_body_n(0.0, 0.0, 8.0);
    constexpr double kHalfPiRad = 1.5707963267948966;
    const drone::AttitudeYPR attitude(0.0, kHalfPiRad, 0.0);

    const drone::Vector3 thrust_enu_n = drone::simulator::physics::rotateBodyToEnu(thrust_body_n, attitude);

    REQUIRE(thrust_enu_n.x == Catch::Approx(8.0).margin(1e-9));
    REQUIRE(thrust_enu_n.y == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(thrust_enu_n.z == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("Net force is near zero at hover with zero velocity", "[ForceDynamics]") {
    const double mass_kg = 2.0;
    const double gravity_ms2 = 9.81;
    const double damping_n_per_mps = 1.2;

    const drone::Vector3 thrust_body_n(0.0, 0.0, mass_kg * gravity_ms2);
    const drone::AttitudeYPR attitude(0.0, 0.0, 0.0);
    const drone::Vector3 velocity_enu_mps(0.0, 0.0, 0.0);

    const drone::Vector3 net_force_enu_n = drone::simulator::physics::computeNetForceEnu(
        thrust_body_n,
        attitude,
        mass_kg,
        velocity_enu_mps,
        damping_n_per_mps,
        gravity_ms2);

    REQUIRE(net_force_enu_n.x == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(net_force_enu_n.y == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(net_force_enu_n.z == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("Damping opposes velocity in ENU net force", "[ForceDynamics]") {
    const double mass_kg = 1.0;
    const double gravity_ms2 = 9.81;
    const double damping_n_per_mps = 1.2;

    const drone::Vector3 thrust_body_n(0.0, 0.0, 0.0);
    const drone::AttitudeYPR attitude(0.0, 0.0, 0.0);
    const drone::Vector3 velocity_enu_mps(1.0, -2.0, 3.0);

    const drone::Vector3 net_force_enu_n = drone::simulator::physics::computeNetForceEnu(
        thrust_body_n,
        attitude,
        mass_kg,
        velocity_enu_mps,
        damping_n_per_mps,
        gravity_ms2);

    REQUIRE(net_force_enu_n.x == Catch::Approx(-1.2).margin(1e-9));
    REQUIRE(net_force_enu_n.y == Catch::Approx(2.4).margin(1e-9));
    REQUIRE(net_force_enu_n.z == Catch::Approx(-13.41).margin(1e-9));
}
