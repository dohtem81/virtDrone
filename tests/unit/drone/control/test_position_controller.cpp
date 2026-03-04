#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "drone/control/position_controller.h"

TEST_CASE("Position controller outputs zero when disabled", "[PositionController]") {
    drone::control::PositionController controller;
    controller.setTargetPosition(10.0, 10.0);
    controller.setEnabled(false);

    controller.update(drone::Vector3(0.0, 0.0, 0.0), drone::Vector3(0.0, 0.0, 0.0), 0.02);

    REQUIRE(controller.getPitchReference() == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(controller.getRollReference() == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("Position error on X drives positive roll command", "[PositionController]") {
    drone::control::PositionController controller;
    controller.setEnabled(true);
    controller.setPositionGain(1.0);
    controller.setVelocityGains(0.2, 0.0);
    controller.setMaxTilt(0.5);
    controller.setTargetPosition(5.0, 0.0);

    controller.update(drone::Vector3(0.0, 0.0, 0.0), drone::Vector3(0.0, 0.0, 0.0), 0.1);

    REQUIRE(controller.getRollReference() > 0.0);
    REQUIRE(controller.getPitchReference() == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("Position error on Y drives positive pitch command", "[PositionController]") {
    drone::control::PositionController controller;
    controller.setEnabled(true);
    controller.setPositionGain(1.0);
    controller.setVelocityGains(0.2, 0.0);
    controller.setMaxTilt(0.5);
    controller.setTargetPosition(0.0, 4.0);

    controller.update(drone::Vector3(0.0, 0.0, 0.0), drone::Vector3(0.0, 0.0, 0.0), 0.1);

    REQUIRE(controller.getPitchReference() > 0.0);
    REQUIRE(controller.getRollReference() == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("Velocity target is saturated by max velocity", "[PositionController]") {
    drone::control::PositionController controller;
    controller.setEnabled(true);
    controller.setPositionGain(2.0);
    controller.setMaxVelocity(3.0);
    controller.setTargetPosition(100.0, 0.0);

    controller.update(drone::Vector3(0.0, 0.0, 0.0), drone::Vector3(0.0, 0.0, 0.0), 0.1);

    const auto velocity_target = controller.getVelocityTarget();
    REQUIRE(velocity_target.x == Catch::Approx(3.0).margin(1e-9));
    REQUIRE(velocity_target.y == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("Pitch and roll commands are clamped to max tilt", "[PositionController]") {
    drone::control::PositionController controller;
    controller.setEnabled(true);
    controller.setPositionGain(5.0);
    controller.setVelocityGains(10.0, 0.0);
    controller.setMaxTilt(0.2);
    controller.setTargetPosition(50.0, 50.0);

    controller.update(drone::Vector3(0.0, 0.0, 0.0), drone::Vector3(0.0, 0.0, 0.0), 0.1);

    REQUIRE(controller.getPitchReference() == Catch::Approx(0.2).margin(1e-12));
    REQUIRE(controller.getRollReference() == Catch::Approx(0.2).margin(1e-12));
}
