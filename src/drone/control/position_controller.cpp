#include "drone/control/position_controller.h"

#include <algorithm>
#include <cmath>

namespace drone::control {

PositionController::PositionController() = default;

void PositionController::setTargetPosition(double target_x_m, double target_y_m) {
    target_position_enu_m_.x = target_x_m;
    target_position_enu_m_.y = target_y_m;
}

void PositionController::setMaxVelocity(double max_velocity_mps) {
    max_velocity_mps_ = std::max(0.1, max_velocity_mps);
}

void PositionController::setMaxTilt(double max_tilt_rad) {
    max_tilt_rad_ = std::abs(max_tilt_rad);
    if (max_tilt_rad_ > 1.5) {  // Prevent >~90 degrees
        max_tilt_rad_ = 1.5;
    }
}

void PositionController::setPositionGain(double kp_pos) {
    kp_pos_ = std::max(0.0, kp_pos);
}

void PositionController::setVelocityGains(double kp_vel, double kd_vel) {
    kp_vel_ = std::max(0.0, kp_vel);
    kd_vel_ = std::max(0.0, kd_vel);
}

void PositionController::setEnabled(bool enabled) {
    enabled_ = enabled;
}

void PositionController::reset() {
    position_error_enu_m_ = Vector3(0.0, 0.0, 0.0);
    velocity_target_enu_mps_ = Vector3(0.0, 0.0, 0.0);
    velocity_error_enu_mps_ = Vector3(0.0, 0.0, 0.0);
    last_velocity_error_enu_mps_ = Vector3(0.0, 0.0, 0.0);
    pitch_reference_rad_ = 0.0;
    roll_reference_rad_ = 0.0;
}

void PositionController::update(const Vector3& current_position_enu_m,
                                const Vector3& current_velocity_enu_mps,
                                double dt_s) {
    if (!enabled_ || dt_s <= 0.0) {
        pitch_reference_rad_ = 0.0;
        roll_reference_rad_ = 0.0;
        return;
    }

    current_position_enu_m_ = current_position_enu_m;
    current_velocity_enu_mps_ = current_velocity_enu_mps;

    // ============= OUTER LOOP: Position error → Velocity target =============
    position_error_enu_m_.x = target_position_enu_m_.x - current_position_enu_m_.x;
    position_error_enu_m_.y = target_position_enu_m_.y - current_position_enu_m_.y;
    position_error_enu_m_.z = 0.0;  // Position controller ignores Z

    // Velocity target from position error (proportional term only)
    velocity_target_enu_mps_.x = kp_pos_ * position_error_enu_m_.x;
    velocity_target_enu_mps_.y = kp_pos_ * position_error_enu_m_.y;
    velocity_target_enu_mps_.z = 0.0;

    // Saturate velocity target to max_velocity_mps
    double vel_target_mag = std::sqrt(
        velocity_target_enu_mps_.x * velocity_target_enu_mps_.x +
        velocity_target_enu_mps_.y * velocity_target_enu_mps_.y);

    if (vel_target_mag > max_velocity_mps_) {
        double scale = max_velocity_mps_ / vel_target_mag;
        velocity_target_enu_mps_.x *= scale;
        velocity_target_enu_mps_.y *= scale;
    }

    // ============= INNER LOOP: Velocity error → Pitch/Roll =============
    velocity_error_enu_mps_.x = velocity_target_enu_mps_.x - current_velocity_enu_mps_.x;
    velocity_error_enu_mps_.y = velocity_target_enu_mps_.y - current_velocity_enu_mps_.y;
    velocity_error_enu_mps_.z = 0.0;

    // Derivative of velocity error (for D term)
    Vector3 velocity_error_derivative_enu = velocity_error_enu_mps_;
    if (dt_s > 0.0) {
        velocity_error_derivative_enu.x =
            (velocity_error_enu_mps_.x - last_velocity_error_enu_mps_.x) / dt_s;
        velocity_error_derivative_enu.y =
            (velocity_error_enu_mps_.y - last_velocity_error_enu_mps_.y) / dt_s;
    }
    last_velocity_error_enu_mps_ = velocity_error_enu_mps_;

    // Pitch/Roll commands from velocity error
    // In ENU frame:
    // - Pitch (θ) command comes from North (Y) velocity error
    // - Roll (φ) command comes from East (X) velocity error
    // Positive pitch tilts nose forward → positive Y velocity
    // Positive roll tilts right wing down → positive X (East) velocity

    double pitch_cmd = kp_vel_ * velocity_error_enu_mps_.y +
                       kd_vel_ * velocity_error_derivative_enu.y;

    double roll_cmd = kp_vel_ * velocity_error_enu_mps_.x +
                      kd_vel_ * velocity_error_derivative_enu.x;

    // Clamp to maximum tilt
    pitch_reference_rad_ = std::clamp(pitch_cmd, -max_tilt_rad_, max_tilt_rad_);
    roll_reference_rad_ = std::clamp(roll_cmd, -max_tilt_rad_, max_tilt_rad_);
}

}  // namespace drone::control
