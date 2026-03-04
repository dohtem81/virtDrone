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

TEST_CASE("Roll rotates body Z thrust into negative ENU Y", "[ForceDynamics]") {
    const drone::Vector3 thrust_body_n(0.0, 0.0, 10.0);
    constexpr double kHalfPiRad = 1.5707963267948966;
    const drone::AttitudeYPR attitude(0.0, 0.0, kHalfPiRad);

    const drone::Vector3 thrust_enu_n = drone::simulator::physics::rotateBodyToEnu(thrust_body_n, attitude);

    REQUIRE(thrust_enu_n.x == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(thrust_enu_n.y == Catch::Approx(-10.0).margin(1e-9));
    REQUIRE(thrust_enu_n.z == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("Yaw with pitch keeps thrust magnitude and creates XY components", "[ForceDynamics]") {
    const drone::Vector3 thrust_body_n(0.0, 0.0, 12.0);
    constexpr double kHalfPiRad = 1.5707963267948966;
    constexpr double kQuarterPiRad = 0.7853981633974483;
    const drone::AttitudeYPR attitude(kHalfPiRad, kQuarterPiRad, 0.0);

    const drone::Vector3 thrust_enu_n = drone::simulator::physics::rotateBodyToEnu(thrust_body_n, attitude);

    REQUIRE(thrust_enu_n.x == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(thrust_enu_n.y == Catch::Approx(12.0 * std::sin(kQuarterPiRad)).margin(1e-9));
    REQUIRE(thrust_enu_n.z == Catch::Approx(12.0 * std::cos(kQuarterPiRad)).margin(1e-9));

    const double thrust_mag = std::sqrt(
        thrust_enu_n.x * thrust_enu_n.x + thrust_enu_n.y * thrust_enu_n.y + thrust_enu_n.z * thrust_enu_n.z);
    REQUIRE(thrust_mag == Catch::Approx(12.0).margin(1e-9));
}

TEST_CASE("Tilt yaw roll produce net-force-driven XYZ movement", "[ForceDynamics]") {
    const double mass_kg = 2.0;
    const double gravity_ms2 = 9.81;
    const double damping_n_per_mps = 0.0;
    const double dt_s = 0.2;

    const drone::Vector3 thrust_body_n(0.0, 0.0, 30.0);
    constexpr double kYawRad = 0.5235987755982988;
    constexpr double kPitchRad = 0.3490658503988659;
    constexpr double kRollRad = 0.17453292519943295;
    const drone::AttitudeYPR attitude(kYawRad, kPitchRad, kRollRad);

    const drone::Vector3 velocity_enu_mps(0.0, 0.0, 0.0);
    const drone::Vector3 net_force_enu_n = drone::simulator::physics::computeNetForceEnu(
        thrust_body_n,
        attitude,
        mass_kg,
        velocity_enu_mps,
        damping_n_per_mps,
        gravity_ms2);

    const drone::Vector3 accel_enu_mps2 = net_force_enu_n * (1.0 / mass_kg);
    const drone::Vector3 velocity_after_dt = accel_enu_mps2 * dt_s;
    const drone::Vector3 position_delta_m = accel_enu_mps2 * (0.5 * dt_s * dt_s);

    REQUIRE(std::abs(net_force_enu_n.x) > 1e-6);
    REQUIRE(std::abs(net_force_enu_n.y) > 1e-6);
    REQUIRE(net_force_enu_n.z > 0.0);

    REQUIRE(velocity_after_dt.x == Catch::Approx(accel_enu_mps2.x * dt_s).margin(1e-9));
    REQUIRE(velocity_after_dt.y == Catch::Approx(accel_enu_mps2.y * dt_s).margin(1e-9));
    REQUIRE(velocity_after_dt.z == Catch::Approx(accel_enu_mps2.z * dt_s).margin(1e-9));

    REQUIRE(position_delta_m.x == Catch::Approx(0.5 * accel_enu_mps2.x * dt_s * dt_s).margin(1e-9));
    REQUIRE(position_delta_m.y == Catch::Approx(0.5 * accel_enu_mps2.y * dt_s * dt_s).margin(1e-9));
    REQUIRE(position_delta_m.z == Catch::Approx(0.5 * accel_enu_mps2.z * dt_s * dt_s).margin(1e-9));

    REQUIRE(std::abs(position_delta_m.x) > 1e-6);
    REQUIRE(std::abs(position_delta_m.y) > 1e-6);
    REQUIRE(position_delta_m.z > 0.0);
}
