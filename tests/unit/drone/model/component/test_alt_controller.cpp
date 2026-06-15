#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "drone/model/components/altitude_controler.h"

using Catch::Approx;
using drone::model::components::AltitudeController;

TEST_CASE("AltitudeController P output for ascent", "[AltitudeController]") {
    AltitudeController ctrl(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, false, false, 0.5);
    ctrl.setTargetAltitude(10.0);

    double rpm_ref = 0.0;
    ctrl.update(0.0, 0.0, rpm_ref, 1.0);

    REQUIRE(ctrl.getLastTargetErrorM() == Approx(10.0));
    REQUIRE(ctrl.getLastPComponentRPM() == Approx(10.0));
    REQUIRE(ctrl.getLastIComponentRPM() == Approx(0.0));
    REQUIRE(ctrl.getLastDComponentRPM() == Approx(0.0));
    REQUIRE(rpm_ref == Approx(10.0));
}

TEST_CASE("AltitudeController P output for descent", "[AltitudeController]") {
    AltitudeController ctrl(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, false, false, 0.5);
    ctrl.setTargetAltitude(0.0);

    double rpm_ref = 0.0;
    ctrl.update(10.0, 0.0, rpm_ref, 1.0);

    REQUIRE(ctrl.getLastTargetErrorM() == Approx(-10.0));
    REQUIRE(ctrl.getLastPComponentRPM() == Approx(-10.0));
    REQUIRE(rpm_ref == Approx(-10.0));
}

TEST_CASE("AltitudeController integrates only inside activation band", "[AltitudeController]") {
    AltitudeController ctrl(1.0, 1.0, 1.0, 0.5, 0.0, 0.0, true, false, 0.5);
    ctrl.setTargetAltitude(1.0);

    double rpm_ref = 0.0;

    ctrl.update(0.8, 0.0, rpm_ref, 1.0);
    REQUIRE(ctrl.getLastTargetErrorM() == Approx(0.2));
    REQUIRE(ctrl.getLastIComponentRPM() == Approx(0.1));
    REQUIRE(rpm_ref == Approx(0.3));

    ctrl.update(0.7, 0.0, rpm_ref, 1.0);
    REQUIRE(ctrl.getLastTargetErrorM() == Approx(0.3));
    REQUIRE(ctrl.getLastIComponentRPM() == Approx(0.25));
    REQUIRE(rpm_ref == Approx(0.55));
}

TEST_CASE("AltitudeController derivative term reacts to error rate", "[AltitudeController]") {
    AltitudeController ctrl(1.0, 1.0, 1.0, 0.0, 0.0, 2.0, false, true, 100.0);
    ctrl.setTargetAltitude(10.0);

    double rpm_ref = 0.0;

    ctrl.update(0.0, 0.0, rpm_ref, 1.0);
    REQUIRE(ctrl.getLastDComponentRPM() == Approx(0.0));

    ctrl.update(5.0, 0.0, rpm_ref, 1.0);
    REQUIRE(ctrl.getLastTargetErrorM() == Approx(5.0));
    REQUIRE(ctrl.getLastDComponentRPM() == Approx(-10.0));
    REQUIRE(rpm_ref == Approx(-5.0));
}
