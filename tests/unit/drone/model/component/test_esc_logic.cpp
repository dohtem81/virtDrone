#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp> // Add this line
#include "drone/model/components/esc.h"

using namespace drone::simulator::components;
using Catch::Approx;

TEST_CASE("ESC_controlLogic getRPMRef_P calculates correct RPM reference - accelerate", "[ESC_controlLogic]") {
    double rpm_ref = 5000.0;
    double rpm_current = 3000.0;
    double rpm_max_delta = 1000.0;
    double control_param_p = 0.5;
    double delta_time_s = 0.1; // 100 ms time step
    double rpm_ref_out = 0.0;
    double rpm_ref_out_clamped = 0.0;

    ESC_controlLogic::getRPMRef_P(
        rpm_ref, 
        rpm_current, 
        rpm_max_delta, 
        control_param_p, 
        delta_time_s, 
        rpm_ref_out, 
        rpm_ref_out_clamped);

    REQUIRE(rpm_ref_out == Approx(3100.0));
    REQUIRE(rpm_ref_out_clamped == Approx(3100.0));
}

TEST_CASE("ESC_controlLogic getRPMRef_P calculates correct RPM reference - decelerate", "[ESC_controlLogic]") {
    double rpm_ref = 3000.0;
    double rpm_current = 5000.0;
    double rpm_max_delta = 1000.0;
    double control_param_p = 0.5;
    double delta_time_s = 0.1; // 100 ms time step
    double rpm_ref_out = 0.0;
    double rpm_ref_out_clamped = 0.0;

    ESC_controlLogic::getRPMRef_P(
        rpm_ref, 
        rpm_current, 
        rpm_max_delta, 
        control_param_p, 
        delta_time_s, 
        rpm_ref_out, 
        rpm_ref_out_clamped);

    REQUIRE(rpm_ref_out == Approx(4900.0));
    REQUIRE(rpm_ref_out_clamped == Approx(4900.0));
}

TEST_CASE("ESC_controlLogic getRPMRef_P calculates correct RPM reference with respect to ramp - accelerate", "[ESC_controlLogic]") {
    double rpm_ref = 15000.0;
    double rpm_current = 3000.0;
    double rpm_max_delta = 1000.0;
    double control_param_p = 0.5;
    double delta_time_s = 0.1; // 100 ms time step
    double rpm_ref_out = 0.0;
    double rpm_ref_out_clamped = 0.0;

    ESC_controlLogic::getRPMRef_P(
        rpm_ref, 
        rpm_current, 
        rpm_max_delta, 
        control_param_p, 
        delta_time_s, 
        rpm_ref_out, 
        rpm_ref_out_clamped);

    REQUIRE(rpm_ref_out == Approx(3600.0));
    REQUIRE(rpm_ref_out_clamped == Approx(3100.0));
}

TEST_CASE("ESC_controlLogic getRPMRef_P calculates correct RPM reference with respect to ramp - decelerate", "[ESC_controlLogic]") {
    double rpm_ref = 3000.0;
    double rpm_current = 15000.0;
    double rpm_max_delta = 1000.0;
    double control_param_p = 0.5;
    double delta_time_s = 0.1; // 100 ms time step
    double rpm_ref_out = 0.0;
    double rpm_ref_out_clamped = 0.0;

    ESC_controlLogic::getRPMRef_P(
        rpm_ref, 
        rpm_current, 
        rpm_max_delta, 
        control_param_p, 
        delta_time_s, 
        rpm_ref_out, 
        rpm_ref_out_clamped);

    REQUIRE(rpm_ref_out == Approx(14400.0));
    REQUIRE(rpm_ref_out_clamped == Approx(14900.0));
}

// these are long term acceleration/deceleration tests that check if we can reach the desired speed over multiple iterations with the ramp rate taken into account
TEST_CASE("ESC_controlLogic getRPMRef_P reaches desired speed over time - accelerate", "[ESC_controlLogic]") {
    double rpm_ref = 5000.0;
    double rpm_current = 0.0;
    double rpm_max_delta = 1000.0;
    double control_param_p = 1.0; // Use full proportional control for this test
    double delta_time_s = 1.0; // 1 second time step
    double rpm_ref_out = 0.0;
    double rpm_ref_out_clamped = 0.0;

    for (int i = 0; i < 5; ++i) { // Simulate for 5 seconds
        ESC_controlLogic::getRPMRef_P(
            rpm_ref, 
            rpm_current, 
            rpm_max_delta, 
            control_param_p, 
            delta_time_s, 
            rpm_ref_out, 
            rpm_ref_out_clamped);

        REQUIRE(rpm_ref_out_clamped == Approx(rpm_current + 1000.0).margin(10.0));

        rpm_current = rpm_ref_out_clamped; // Update current speed to the clamped output for next iteration
    }

    REQUIRE(rpm_current == Approx(5000.0).margin(100.0)); // Allow some margin due to ramping
}

TEST_CASE("ESC_controlLogic getRPMRef_P reaches desired speed over time - decelerate", "[ESC_controlLogic]") {
    double rpm_ref = 0.0;
    double rpm_current = 5000.0;
    double rpm_max_delta = 1000.0;
    double control_param_p = 1.0; // Use full proportional control for this test
    double delta_time_s = 1.0; // 1 second time step
    double rpm_ref_out = 0.0;
    double rpm_ref_out_clamped = 0.0;

    for (int i = 0; i < 5; ++i) { // Simulate for 5 seconds
        ESC_controlLogic::getRPMRef_P(
            rpm_ref, 
            rpm_current, 
            rpm_max_delta, 
            control_param_p, 
            delta_time_s, 
            rpm_ref_out, 
            rpm_ref_out_clamped);

        REQUIRE(rpm_ref_out_clamped == Approx(rpm_current - 1000.0).margin(10.0));
        rpm_current = rpm_ref_out_clamped; // Update current speed to the clamped output for next iteration
    }

    REQUIRE(rpm_current == Approx(0.0).margin(100.0)); // Allow some margin due to ramping
}