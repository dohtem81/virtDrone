#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp> // Add this line
#include "drone/model/components/altitude_controler.h"

using namespace drone::model::components;
using Catch::Approx;

TEST_CASE("AltitudeController updates altitude reference correctly P-controller", "[AltitudeController]") {
    AltitudeController alt_ctrl(1.0, 1.0, 1.0, 0.0); // P=1.0, max_alt_delta=1.0 m/s, I=0.0
    alt_ctrl.setTargetAltitude(10.0); // Target altitude of 10 meters

    double current_altitude = 0.0;
    double delta_time_s = 1.0; // 1 second time step
    double altitude_ref_inuse = 0.0;

    // First update should increase altitude reference towards target
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(1.0)); // Should move towards target by 1 meter

    current_altitude = 1.0;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(2.0)); // Should move towards target by another 1 meter (P control only)

    // Simulate update with current altitude now at 5 meters
    current_altitude = 5.0;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(6.0)); // Should adjust reference based on new error
}

TEST_CASE("AltitudeController respects max altitude change rate", "[AltitudeController]") {
    AltitudeController alt_ctrl(1.0, 0.5, 1.0, 0.0); // P=1.0, max_alt_delta=0.5 m/s, I=0.0
    alt_ctrl.setTargetAltitude(10.0); // Target altitude of 10 meters

    double current_altitude = 0.0;
    double delta_time_s = 1.0; // 1 second time step
    double altitude_ref_inuse = 0.0;

    // First update should increase altitude reference towards target but limited by max_alt_delta
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(0.5)); // Should move towards target by max 0.5 meters

    current_altitude = 0.5;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(1.0)); // Should move towards target by another max 0.5 meters
}

TEST_CASE("AltitudeController updates altitude reference correctly", "[AltitudeController]") {
    AltitudeController alt_ctrl(1.0, 1.0, 1.0, 0.1); // P=1.0, max_alt_delta=1.0 m/s, I=0.1
    alt_ctrl.setTargetAltitude(10.0); // Target altitude of 10 meters

    double current_altitude = 0.0;
    double delta_time_s = 1.0; // 1 second time step
    double altitude_ref_inuse = 0.0;

    // First update should increase altitude reference towards target
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(1.0)); // Should move towards target by 1 meter

    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(1.0)); // Should move towards target by another 1 meter (P control only)

    // Simulate update with current altitude now at 5 meters
    current_altitude = 5.0;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(6.0)); // Should adjust reference based on new error
}

TEST_CASE("AltitudeController updates altitude reference correctly P-controller - descent", "[AltitudeController]") {
    AltitudeController alt_ctrl(1.0, 1.0, 1.0, 0.0); // P=1.0, max_alt_delta=1.0 m/s, I=0.0
    alt_ctrl.setTargetAltitude(0.0); // Target altitude of 0 meters

    double current_altitude = 10.0;
    double delta_time_s = 1.0; // 1 second time step
    double altitude_ref_inuse = 0.0;

    // First update should increase altitude reference towards target
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(9.0)); // Should move towards target by 1 meter

    current_altitude = 9.0;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(8.0)); // Should move towards target by another 1 meter (P control only)

    // Simulate update with current altitude now at 5 meters
    current_altitude = 5.0;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(4.0)); // Should adjust reference based on new error
}

TEST_CASE("AltitudeController respects max altitude change rate - descent", "[AltitudeController]") {
    AltitudeController alt_ctrl(1.0, 0.5, 1.0, 0.0); // P=1.0, max_alt_delta=0.5 m/s, I=0.0
    alt_ctrl.setTargetAltitude(0.0); // Target altitude of 0 meters

    double current_altitude = 10.0;
    double delta_time_s = 1.0; // 1 second time step
    double altitude_ref_inuse = 0.0;

    // First update should increase altitude reference towards target but limited by max_alt_delta
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(9.5)); // Should move towards target by max 0.5 meters

    current_altitude = 9.5;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(9.0)); // Should move towards target by another max 0.5 meters
}

TEST_CASE("AltitudeController updates altitude reference correctly - descent", "[AltitudeController]") {
    AltitudeController alt_ctrl(1.0, 1.0, 1.0, 0.1); // P=1.0, max_alt_delta=1.0 m/s, I=0.1
    alt_ctrl.setTargetAltitude(0.0); // Target altitude of 0 meters

    double current_altitude = 10.0;
    double delta_time_s = 1.0; // 1 second time step
    double altitude_ref_inuse = 0.0;

    // First update should increase altitude reference towards target
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(9.0)); // Should move towards target by 1 meter

    current_altitude = 9.0;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(8.0)); // Should move towards target by another 1 meter (P control only)

    // Simulate update with current altitude now at 5 meters
    current_altitude = 5.0;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, 0.0, altitude_ref_inuse, delta_time_s);
    REQUIRE(alt_ctrl.getAltitudeRefInUse() == Approx(4.0)); // Should adjust reference based on new error
}

TEST_CASE("RPM control based on altitude reference - ascent", "[AltitudeController]") {
    AltitudeController alt_ctrl(1.0, 1.0, 1.0, 0.1); // P=1.0, max_alt_delta=1.0 m/s, I=0.1
    alt_ctrl.setTargetAltitude(10.0); // Target altitude of 10 meters
    double current_altitude = 0.0;
    double delta_time_s = 1.0; // 0.1 second time step
    double altitude_ref_inuse = 0.0;    

    // assumption is that 100RPM is ascent speed of 1 m/s
    double currentRPM = 0.0;
    double rpm_ref_out = 0.0;
    alt_ctrl.update(current_altitude, altitude_ref_inuse, currentRPM, rpm_ref_out, delta_time_s);
    REQUIRE(rpm_ref_out == Approx(1.0).margin(0.1)); // RPM reference should increase to ascend
    current_altitude += (currentRPM + rpm_ref_out) * 0.5 * delta_time_s; // Simulate altitude change based on RPM
    currentRPM = rpm_ref_out; // Simulate motor responding to RPM reference

    alt_ctrl.update(current_altitude, altitude_ref_inuse, currentRPM, rpm_ref_out, delta_time_s);
    REQUIRE(rpm_ref_out == Approx(2.3).margin(0.1)); // RPM reference should increase to ascend
    current_altitude += (currentRPM + rpm_ref_out) * 0.5 * delta_time_s; // Simulate altitude change based on RPM
    currentRPM = rpm_ref_out; // Simulate motor responding to RPM reference

    delta_time_s = 0.1;
    for (auto i = 0; i < 10; ++i) {
        alt_ctrl.update(current_altitude, altitude_ref_inuse, currentRPM, rpm_ref_out, delta_time_s);
        REQUIRE(rpm_ref_out > currentRPM); // RPM reference should continue to increase to ascend
        current_altitude += (currentRPM + rpm_ref_out) * 0.5 * delta_time_s; // Simulate altitude change based on RPM
        currentRPM = rpm_ref_out; // Simulate motor responding to RPM reference
    }

}