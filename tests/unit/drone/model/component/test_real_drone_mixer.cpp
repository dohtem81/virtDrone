#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "drone/model/components/altitude_controler.h"
#include "drone/runtime/real_drone.h"

namespace {

class StaticSensorSource final : public drone::runtime::SensorSource {
public:
    explicit StaticSensorSource(const drone::runtime::SensorFrame& frame)
        : frame_(frame) {}

    drone::runtime::SensorFrame readSensors() const override {
        return frame_;
    }

private:
    drone::runtime::SensorFrame frame_;
};

class CaptureActuatorSink final : public drone::runtime::ActuatorSink {
public:
    void applyActuators(const drone::runtime::ActuatorFrame& actuator_frame) override {
        last_frame = actuator_frame;
    }

    drone::runtime::ActuatorFrame last_frame{};
};

}  // namespace

TEST_CASE("RealDrone splits common reference with yaw differential", "[RealDrone][Mixer]") {
    drone::model::components::AltitudeController altitude_controller(
        0.0,
        1.0,
        0.0,
        0.0,
        5000.0,
        0.0,
        false,
        false,
        0.5);

    drone::runtime::RealDrone real_drone(altitude_controller);
    real_drone.setTargetYaw(0.2);
    real_drone.setAttitudeControlGains(100.0, 0.0, 0.0);

    drone::runtime::SensorFrame sensors;
    sensors.altitude_m = 0.0;
    sensors.yaw_rad = 0.0;

    StaticSensorSource sensor_source(sensors);
    CaptureActuatorSink actuator_sink;

    real_drone.update(0.01, sensor_source, actuator_sink);

    REQUIRE(actuator_sink.last_frame.common_motor_rpm == Catch::Approx(5000.0));
    REQUIRE(actuator_sink.last_frame.yaw_control_rpm == Catch::Approx(20.0));

    // X frame (FL, FR, RR, RL): yaw differential +,-,+,-
    REQUIRE(actuator_sink.last_frame.desired_motor_rpm_each[0] == Catch::Approx(5020.0));
    REQUIRE(actuator_sink.last_frame.desired_motor_rpm_each[1] == Catch::Approx(4980.0));
    REQUIRE(actuator_sink.last_frame.desired_motor_rpm_each[2] == Catch::Approx(5020.0));
    REQUIRE(actuator_sink.last_frame.desired_motor_rpm_each[3] == Catch::Approx(4980.0));
}

TEST_CASE("RealDrone preserves common reference while scaling differential on saturation", "[RealDrone][Mixer]") {
    drone::model::components::AltitudeController altitude_controller(
        0.0,
        1.0,
        0.0,
        0.0,
        19990.0,
        0.0,
        false,
        false,
        0.5);

    drone::runtime::RealDrone real_drone(altitude_controller);
    real_drone.setTargetRoll(1.0);
    real_drone.setAttitudeControlGains(0.0, 0.0, 5000.0);

    drone::runtime::SensorFrame sensors;
    sensors.roll_rad = 0.0;

    StaticSensorSource sensor_source(sensors);
    CaptureActuatorSink actuator_sink;

    real_drone.update(0.01, sensor_source, actuator_sink);

    for (double rpm : actuator_sink.last_frame.desired_motor_rpm_each) {
        REQUIRE(rpm >= 0.0);
        REQUIRE(rpm <= 20000.0);
    }

    const double avg_rpm =
        (actuator_sink.last_frame.desired_motor_rpm_each[0] +
         actuator_sink.last_frame.desired_motor_rpm_each[1] +
         actuator_sink.last_frame.desired_motor_rpm_each[2] +
         actuator_sink.last_frame.desired_motor_rpm_each[3]) /
        4.0;
    REQUIRE(avg_rpm == Catch::Approx(actuator_sink.last_frame.common_motor_rpm).margin(1e-6));
}
