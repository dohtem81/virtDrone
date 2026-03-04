#ifndef DRONE_RUNTIME_REAL_DRONE_H
#define DRONE_RUNTIME_REAL_DRONE_H

#include "drone/model/components/altitude_controler.h"
#include "drone/control/position_controller.h"
#include "drone/drone_data_types.h"
#include "drone/mission/mission_executor.h"
#include "drone/mission/mission_loader.h"

#include <algorithm>
#include <array>
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

    void setAttitudeControlGains(double yaw_rpm_per_rad, double pitch_rpm_per_rad, double roll_rpm_per_rad) {
        yaw_gain_rpm_per_rad_ = yaw_rpm_per_rad;
        pitch_gain_rpm_per_rad_ = pitch_rpm_per_rad;
        roll_gain_rpm_per_rad_ = roll_rpm_per_rad;
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

        double desired_common_motor_rpm = 0.0;
        altitude_controller_.update(
            sensors.altitude_m,
            sensed_avg_motor_rpm,
            desired_common_motor_rpm,
            dt_s);

        // Update position controller with current state (if enabled)
        Vector3 current_position_enu_m(sensors.position_enu_x_m,
                           sensors.position_enu_y_m,
                           sensors.position_enu_z_m);
        Vector3 current_velocity_enu_mps(sensors.gps_velocity_east_mps,
                                         sensors.gps_velocity_north_mps,
                                         sensors.gps_velocity_down_mps);
        if (position_controller_->isEnabled() && !position_target_initialized_) {
            setTargetPosition(sensors.position_enu_x_m, sensors.position_enu_y_m);
        }
        position_controller_->update(current_position_enu_m, current_velocity_enu_mps, dt_s);

        // Determine pitch/roll targets: use position controller if enabled, else use direct targets
        double effective_pitch_rad = target_pitch_rad_;
        double effective_roll_rad = target_roll_rad_;

        if (position_controller_->isEnabled()) {
            effective_pitch_rad = position_controller_->getPitchReference();
            effective_roll_rad = position_controller_->getRollReference();
        }

        const double yaw_error_rad = target_yaw_rad_ - sensors.yaw_rad;
        const double pitch_error_rad = effective_pitch_rad - sensors.pitch_rad;
        const double roll_error_rad = effective_roll_rad - sensors.roll_rad;

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
    double yaw_gain_rpm_per_rad_ = 300.0;
    double pitch_gain_rpm_per_rad_ = 300.0;
    double roll_gain_rpm_per_rad_ = 300.0;
    bool position_target_initialized_ = false;
    mission::MissionLoader mission_loader_;
    mission::Mission mission_;
    mission::MissionExecutor mission_executor_;
    bool mission_loaded_ = false;
};

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_REAL_DRONE_H
