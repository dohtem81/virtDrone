#ifndef DRONE_RUNTIME_REAL_DRONE_H
#define DRONE_RUNTIME_REAL_DRONE_H

#include "drone/model/components/altitude_controler.h"
#include "drone/control/position_controller.h"
#include "drone/drone_data_types.h"
#include "drone/mission/mission_executor.h"
#include "drone/mission/mission_loader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace drone::runtime {

constexpr std::size_t kMotorCount = 4;

struct SensorFrame {
    double altitude_m = 0.0;
    double position_enu_x_m = 0.0;
    double position_enu_y_m = 0.0;
    double position_enu_z_m = 0.0;
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
    double sensed_position_enu_x_m = 0.0;
    double sensed_position_enu_y_m = 0.0;
    double sensed_position_enu_z_m = 0.0;
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
        : altitude_controller_(altitude_controller) {
        position_controller_ = std::make_unique<control::PositionController>();
        position_controller_->setEnabled(true);
    }

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

    void setAttitudeGains(double yaw_p, double yaw_d, double pitch_p, double pitch_d, double roll_p, double roll_d) {
        yaw_gain_rpm_per_rad_ = yaw_p;
        yaw_d_gain_rpm_per_rad_s_ = yaw_d;
        pitch_gain_rpm_per_rad_ = pitch_p;
        pitch_d_gain_rpm_per_rad_s_ = pitch_d;
        roll_gain_rpm_per_rad_ = roll_p;
        roll_d_gain_rpm_per_rad_s_ = roll_d;
    }

    /**
     * @brief Set target XY position for position controller
     * @param target_x_m East position in meters (ENU frame)
     * @param target_y_m North position in meters (ENU frame)
     */
    void setTargetPosition(double target_x_m, double target_y_m) {
        position_controller_->setTargetPosition(target_x_m, target_y_m);
        position_target_initialized_ = true;
    }

    /**
     * @brief Enable or disable position controller
     */
    void setPositionControlEnabled(bool enabled) {
        const bool was_enabled = position_controller_->isEnabled();
        position_controller_->setEnabled(enabled);
        if (enabled && !was_enabled) {
            position_target_initialized_ = false;
        }
    }

    /**
     * @brief Set maximum velocity for position controller
     */
    void setMaxVelocity(double max_velocity_mps) {
        position_controller_->setMaxVelocity(max_velocity_mps);
    }

    void setPositionGain(double kp_pos) {
        position_controller_->setPositionGain(kp_pos);
    }

    void setVelocityGains(double kp_vel, double kd_vel) {
        position_controller_->setVelocityGains(kp_vel, kd_vel);
    }

    /**
     * @brief Set maximum tilt (pitch/roll) angle for position controller
     */
    void setMaxTilt(double max_tilt_rad) {
        position_controller_->setMaxTilt(max_tilt_rad);
    }

    bool loadMissionFromFile(const std::string& mission_file, std::string* error_out = nullptr) {
        mission::Mission parsed_mission;
        if (!mission_loader_.loadFromFile(mission_file, parsed_mission, error_out)) {
            mission_loaded_ = false;
            return false;
        }

        mission_ = std::move(parsed_mission);
        mission_executor_.loadMission(mission_);
        mission_loaded_ = true;
        return true;
    }

    void startMission() {
        if (!mission_loaded_) {
            return;
        }
        mission_executor_.start();
    }

    void updateMission(const SensorFrame& sensor_frame, double dt_s) {
        if (!mission_loaded_) {
            return;
        }
        mission_executor_.update(*this, sensor_frame, dt_s);
    }

    mission::MissionStatus getMissionStatus() const {
        return mission_executor_.getStatus();
    }

    int getCurrentMissionStepId() const {
        return mission_executor_.getCurrentStepId();
    }

    std::string getCurrentMissionStepName() const {
        return mission_executor_.getCurrentStepName();
    }

    bool hasMissionLoaded() const {
        return mission_loaded_;
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

        // Altitude controller computes collective RPM for near-level flight.
        // We then compensate by the current thrust Z projection so altitude (Z) control
        // remains decoupled from XY motion (handled by yaw/roll controller below).
        double desired_common_motor_rpm_level = 0.0;
        altitude_controller_.update(
            sensors.altitude_m,
            sensed_avg_motor_rpm,
            desired_common_motor_rpm_level,
            dt_s);

        const double thrust_z_projection = std::cos(sensors.roll_rad) * std::cos(sensors.pitch_rad);
        constexpr double kMinThrustZProjection = 0.35;  // avoid excessive amplification near high tilt
        const double safe_thrust_z_projection = std::max(kMinThrustZProjection, thrust_z_projection);
        double desired_common_motor_rpm = desired_common_motor_rpm_level / safe_thrust_z_projection;
        desired_common_motor_rpm = std::clamp(desired_common_motor_rpm, 0.0, 20000.0);

        // XY position control: compute yaw and roll to reach target position
        // Strategy: Keep pitch=0, use yaw to face target, use roll to move laterally
        double effective_yaw_rad = target_yaw_rad_;
        double effective_pitch_rad = 0.0;  // Keep pitch at zero
        double effective_roll_rad = target_roll_rad_;

        if (position_controller_->isEnabled()) {
            if (!position_target_initialized_) {
                setTargetPosition(sensors.position_enu_x_m, sensors.position_enu_y_m);
            }
            
            // Calculate position error in ENU frame
            const double dx_enu = position_controller_->getTargetPosition().x - sensors.position_enu_x_m;
            const double dy_enu = position_controller_->getTargetPosition().y - sensors.position_enu_y_m;
            const double distance_xy = std::sqrt(dx_enu * dx_enu + dy_enu * dy_enu);
            
            // If we have significant position error, control yaw and roll
            if (distance_xy > 0.5) {  // Dead zone to avoid jitter
                // Yaw control: align body-right axis with the target direction
                // so roll-only control can drive XY without using pitch.
                constexpr double kHalfPi = 1.5707963267948966;
                const double target_yaw_for_roll_motion = std::atan2(dy_enu, dx_enu) - kHalfPi;
                effective_yaw_rad = target_yaw_for_roll_motion;

                // Roll control from signed lateral error in body frame (right axis).
                const double yaw_c = std::cos(effective_yaw_rad);
                const double yaw_s = std::sin(effective_yaw_rad);
                const double error_body_right_m = -yaw_s * dx_enu + yaw_c * dy_enu;

                const double kp_xy = 0.08;
                const double max_roll_for_xy = 0.3;  // Max ~17 degrees tilt
                const double roll_cmd = -kp_xy * error_body_right_m;
                effective_roll_rad = std::clamp(roll_cmd, -max_roll_for_xy, max_roll_for_xy);
            } else {
                // Close to target, minimize roll
                effective_roll_rad = 0.0;
            }
        }

        const double yaw_error_rad = effective_yaw_rad - sensors.yaw_rad;
        const double pitch_error_rad = effective_pitch_rad - sensors.pitch_rad;
        const double roll_error_rad = effective_roll_rad - sensors.roll_rad;

        // PD control with derivative damping
        const double yaw_error_rate_rad_s = (yaw_error_rad - prev_yaw_error_rad_) / dt_s;
        const double pitch_error_rate_rad_s = (pitch_error_rad - prev_pitch_error_rad_) / dt_s;
        const double roll_error_rate_rad_s = (roll_error_rad - prev_roll_error_rad_) / dt_s;
        
        prev_yaw_error_rad_ = yaw_error_rad;
        prev_pitch_error_rad_ = pitch_error_rad;
        prev_roll_error_rad_ = roll_error_rad;

        const double yaw_control_rpm = yaw_gain_rpm_per_rad_ * yaw_error_rad 
                                      + yaw_d_gain_rpm_per_rad_s_ * yaw_error_rate_rad_s;
        const double pitch_control_rpm = pitch_gain_rpm_per_rad_ * pitch_error_rad
                                        + pitch_d_gain_rpm_per_rad_s_ * pitch_error_rate_rad_s;
        const double roll_control_rpm = roll_gain_rpm_per_rad_ * roll_error_rad
                                       + roll_d_gain_rpm_per_rad_s_ * roll_error_rate_rad_s;

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
            effective_yaw_rad,
            effective_pitch_rad,
            effective_roll_rad,
            altitude_controller_.getTargetAltitude(),
            altitude_controller_.getLastTargetErrorM(),
            altitude_controller_.getLastPComponentRPM(),
            altitude_controller_.getLastIComponentRPM(),
            altitude_controller_.getLastDComponentRPM(),
            sensors.altitude_m,
            sensors.position_enu_x_m,
            sensors.position_enu_y_m,
            sensors.position_enu_z_m,
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
    std::unique_ptr<control::PositionController> position_controller_;
    double target_yaw_rad_ = 0.0;
    double target_pitch_rad_ = 0.0;
    double target_roll_rad_ = 0.0;
    double yaw_gain_rpm_per_rad_ = 200.0;
    double pitch_gain_rpm_per_rad_ = 250.0;
    double roll_gain_rpm_per_rad_ = 250.0;
    double yaw_d_gain_rpm_per_rad_s_ = 50.0;
    double pitch_d_gain_rpm_per_rad_s_ = 60.0;
    double roll_d_gain_rpm_per_rad_s_ = 60.0;
    double prev_yaw_error_rad_ = 0.0;
    double prev_pitch_error_rad_ = 0.0;
    double prev_roll_error_rad_ = 0.0;
    bool position_target_initialized_ = false;
    mission::MissionLoader mission_loader_;
    mission::Mission mission_;
    mission::MissionExecutor mission_executor_;
    bool mission_loaded_ = false;
};

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_REAL_DRONE_H
