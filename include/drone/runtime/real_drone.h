#ifndef DRONE_RUNTIME_REAL_DRONE_H
#define DRONE_RUNTIME_REAL_DRONE_H

#include "drone/model/components/altitude_controler.h"

#include <algorithm>
#include <array>

namespace drone::runtime {

constexpr std::size_t kMotorCount = 4;

struct SensorFrame {
    double altitude_m = 0.0;
    double gps_latitude_deg = 0.0;
    double gps_longitude_deg = 0.0;
    double gps_altitude_m = 0.0;
    double gps_velocity_north_mps = 0.0;
    double gps_velocity_east_mps = 0.0;
    double gps_velocity_down_mps = 0.0;
    double battery_voltage_v = 0.0;
    double battery_soc_percent = 0.0;
    double motor_temperature_c = 0.0;
    double motor_rpm = 0.0;
    std::array<double, kMotorCount> motor_rpm_each{};
    std::array<double, kMotorCount> motor_temperature_c_each{};
    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
    double roll_rad = 0.0;
};

struct ActuatorFrame {
    double desired_motor_rpm = 0.0;
    double common_motor_rpm = 0.0;
    std::array<double, kMotorCount> desired_motor_rpm_each{};
    double yaw_control_rpm = 0.0;
    double pitch_control_rpm = 0.0;
    double roll_control_rpm = 0.0;
    double desired_yaw_rad = 0.0;
    double desired_pitch_rad = 0.0;
    double desired_roll_rad = 0.0;
    double target_altitude_m = 0.0;
    double target_error_m = 0.0;
    double p_component_rpm = 0.0;
    double i_component_rpm = 0.0;
    double d_component_rpm = 0.0;
    double sensed_altitude_m = 0.0;
    double sensed_gps_latitude_deg = 0.0;
    double sensed_gps_longitude_deg = 0.0;
    double sensed_gps_altitude_m = 0.0;
    double sensed_gps_velocity_north_mps = 0.0;
    double sensed_gps_velocity_east_mps = 0.0;
    double sensed_gps_velocity_down_mps = 0.0;
    double sensed_battery_voltage_v = 0.0;
    double sensed_battery_soc_percent = 0.0;
    double sensed_motor_temperature_c = 0.0;
    double sensed_motor_rpm = 0.0;
    double sensed_yaw_rad = 0.0;
    double sensed_pitch_rad = 0.0;
    double sensed_roll_rad = 0.0;
};

class SensorSource {
public:
    virtual ~SensorSource() = default;
    virtual SensorFrame readSensors() const = 0;
};

class ActuatorSink {
public:
    virtual ~ActuatorSink() = default;
    virtual void applyActuators(const ActuatorFrame& actuator_frame) = 0;
};

class RealDrone {
public:
    explicit RealDrone(const model::components::AltitudeController& altitude_controller)
        : altitude_controller_(altitude_controller) {}

    void setTargetAltitude(double target_altitude_m) {
        altitude_controller_.setTargetAltitude(target_altitude_m);
    }

    void setTargetYaw(double target_yaw_rad) {
        target_yaw_rad_ = target_yaw_rad;
    }

    void setTargetPitch(double target_pitch_rad) {
        target_pitch_rad_ = target_pitch_rad;
    }

    void setTargetRoll(double target_roll_rad) {
        target_roll_rad_ = target_roll_rad;
    }

    void setAttitudeControlGains(double yaw_rpm_per_rad, double pitch_rpm_per_rad, double roll_rpm_per_rad) {
        yaw_gain_rpm_per_rad_ = yaw_rpm_per_rad;
        pitch_gain_rpm_per_rad_ = pitch_rpm_per_rad;
        roll_gain_rpm_per_rad_ = roll_rpm_per_rad;
    }

    void update(double dt_s, const SensorSource& sensor_source, ActuatorSink& actuator_sink) {
        const SensorFrame sensors = sensor_source.readSensors();

        double sensed_avg_motor_rpm = sensors.motor_rpm;
        double rpm_sum = 0.0;
        for (double rpm : sensors.motor_rpm_each) {
            rpm_sum += rpm;
        }
        if (rpm_sum > 0.0) {
            sensed_avg_motor_rpm = rpm_sum / static_cast<double>(kMotorCount);
        }

        double desired_common_motor_rpm = 0.0;
        altitude_controller_.update(
            sensors.altitude_m,
            sensed_avg_motor_rpm,
            desired_common_motor_rpm,
            dt_s);

        const double yaw_error_rad = target_yaw_rad_ - sensors.yaw_rad;
        const double pitch_error_rad = target_pitch_rad_ - sensors.pitch_rad;
        const double roll_error_rad = target_roll_rad_ - sensors.roll_rad;

        const double yaw_control_rpm = yaw_gain_rpm_per_rad_ * yaw_error_rad;
        const double pitch_control_rpm = pitch_gain_rpm_per_rad_ * pitch_error_rad;
        const double roll_control_rpm = roll_gain_rpm_per_rad_ * roll_error_rad;

        // X-frame allocation (FL, FR, RR, RL)
        std::array<double, kMotorCount> diff_rpm{
            -pitch_control_rpm + roll_control_rpm + yaw_control_rpm,
            -pitch_control_rpm - roll_control_rpm - yaw_control_rpm,
            +pitch_control_rpm - roll_control_rpm + yaw_control_rpm,
            +pitch_control_rpm + roll_control_rpm - yaw_control_rpm};

        // Preserve common RPM first, then scale differential terms on saturation
        constexpr double kMinMotorRpm = 0.0;
        constexpr double kMaxMotorRpm = 20000.0;
        double max_diff = *std::max_element(diff_rpm.begin(), diff_rpm.end());
        double min_diff = *std::min_element(diff_rpm.begin(), diff_rpm.end());
        double scale = 1.0;

        if (max_diff > 0.0 && desired_common_motor_rpm + max_diff > kMaxMotorRpm) {
            scale = std::min(scale, (kMaxMotorRpm - desired_common_motor_rpm) / max_diff);
        }
        if (min_diff < 0.0 && desired_common_motor_rpm + min_diff < kMinMotorRpm) {
            scale = std::min(scale, (kMinMotorRpm - desired_common_motor_rpm) / min_diff);
        }
        scale = std::clamp(scale, 0.0, 1.0);

        std::array<double, kMotorCount> desired_motor_rpm_each{};
        for (std::size_t i = 0; i < kMotorCount; ++i) {
            desired_motor_rpm_each[i] = std::clamp(
                desired_common_motor_rpm + scale * diff_rpm[i],
                kMinMotorRpm,
                kMaxMotorRpm);
        }

        actuator_sink.applyActuators(ActuatorFrame{
            desired_common_motor_rpm,
            desired_common_motor_rpm,
            desired_motor_rpm_each,
            yaw_control_rpm,
            pitch_control_rpm,
            roll_control_rpm,
            target_yaw_rad_,
            target_pitch_rad_,
            target_roll_rad_,
            altitude_controller_.getTargetAltitude(),
            altitude_controller_.getLastTargetErrorM(),
            altitude_controller_.getLastPComponentRPM(),
            altitude_controller_.getLastIComponentRPM(),
            altitude_controller_.getLastDComponentRPM(),
            sensors.altitude_m,
            sensors.gps_latitude_deg,
            sensors.gps_longitude_deg,
            sensors.gps_altitude_m,
            sensors.gps_velocity_north_mps,
            sensors.gps_velocity_east_mps,
            sensors.gps_velocity_down_mps,
            sensors.battery_voltage_v,
            sensors.battery_soc_percent,
            sensors.motor_temperature_c,
            sensors.motor_rpm,
            sensors.yaw_rad,
            sensors.pitch_rad,
            sensors.roll_rad});
    }

private:
    model::components::AltitudeController altitude_controller_;
    double target_yaw_rad_ = 0.0;
    double target_pitch_rad_ = 0.0;
    double target_roll_rad_ = 0.0;
    double yaw_gain_rpm_per_rad_ = 300.0;
    double pitch_gain_rpm_per_rad_ = 300.0;
    double roll_gain_rpm_per_rad_ = 300.0;
};

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_REAL_DRONE_H
