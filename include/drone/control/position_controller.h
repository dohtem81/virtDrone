#ifndef DRONE_CONTROL_POSITION_CONTROLLER_H
#define DRONE_CONTROL_POSITION_CONTROLLER_H

#include "drone/drone_data_types.h"

namespace drone::control {

/**
 * @brief Cascaded position controller for XY movement
 *
 * Implements a two-loop controller:
 * - Outer loop: position error → velocity target (with saturation)
 * - Inner loop: velocity error → pitch/roll attitude commands (with angle clamping)
 *
 * This decouples horizontal movement from vertical (altitude) control.
 * Position controller does not affect altitude or yaw.
 */
class PositionController {
public:
    PositionController();

    /**
     * @brief Set the target XY position (ENU frame)
     * @param target_x_m East position in meters
     * @param target_y_m North position in meters
     */
    void setTargetPosition(double target_x_m, double target_y_m);

    /**
     * @brief Get the target position
     */
    Vector3 getTargetPosition() const { return target_position_enu_m_; }

    /**
     * @brief Set maximum velocity magnitude (outer loop saturation)
     * @param max_velocity_mps Maximum velocity in m/s
     */
    void setMaxVelocity(double max_velocity_mps);

    /**
     * @brief Set maximum tilt angle (pitch/roll clamp)
     * @param max_tilt_rad Maximum absolute tilt in radians
     */
    void setMaxTilt(double max_tilt_rad);

    /**
     * @brief Set position loop proportional gain
     * @param kp_pos Gain from position error to velocity target
     */
    void setPositionGain(double kp_pos);

    /**
     * @brief Set velocity loop gains (proportional and derivative)
     * @param kp_vel Proportional gain (error to angle)
     * @param kd_vel Derivative gain (velocity to angle feedforward)
     */
    void setVelocityGains(double kp_vel, double kd_vel);

    /**
     * @brief Enable or disable position controller
     * @param enabled If false, outputs will be zero
     */
    void setEnabled(bool enabled);

    bool isEnabled() const { return enabled_; }

    /**
     * @brief Main update function - compute pitch/roll references
     * @param current_position_enu_m Current position in ENU frame (meters)
     * @param current_velocity_enu_mps Current velocity in ENU frame (m/s)
     * @param dt_s Delta time since last update (seconds)
     */
    void update(const Vector3& current_position_enu_m,
                const Vector3& current_velocity_enu_mps,
                double dt_s);

    /**
     * @brief Get the computed pitch reference (command to attitude controller)
     */
    double getPitchReference() const { return pitch_reference_rad_; }

    /**
     * @brief Get the computed roll reference (command to attitude controller)
     */
    double getRollReference() const { return roll_reference_rad_; }

    /**
     * @brief Get the current velocity target from outer loop
     */
    Vector3 getVelocityTarget() const { return velocity_target_enu_mps_; }

    /**
     * @brief Get position error (diagnostic)
     */
    Vector3 getPositionError() const { return position_error_enu_m_; }

    /**
     * @brief Get velocity error (diagnostic)
     */
    Vector3 getVelocityError() const { return velocity_error_enu_mps_; }

    /**
     * @brief Reset internal state (velocity tracking, derivatives)
     */
    void reset();

private:
    // Targets
    Vector3 target_position_enu_m_{0.0, 0.0, 0.0};

    // Current measurements
    Vector3 current_position_enu_m_{0.0, 0.0, 0.0};
    Vector3 current_velocity_enu_mps_{0.0, 0.0, 0.0};

    // Errors
    Vector3 position_error_enu_m_{0.0, 0.0, 0.0};
    Vector3 velocity_target_enu_mps_{0.0, 0.0, 0.0};
    Vector3 velocity_error_enu_mps_{0.0, 0.0, 0.0};

    // Derivative term (for D component in velocity loop)
    Vector3 last_velocity_error_enu_mps_{0.0, 0.0, 0.0};

    // Control gains
    double kp_pos_ = 1.0;      // position error → velocity target
    double kp_vel_ = 10.0;     // velocity error → attitude angle
    double kd_vel_ = 2.0;      // d(velocity error) → attitude feedforward

    // Limits
    double max_velocity_mps_ = 20.0;
    double max_tilt_rad_ = 1.3;  // ~74 degrees

    // Output (read by RealDrone)
    double pitch_reference_rad_ = 0.0;
    double roll_reference_rad_ = 0.0;

    // State
    bool enabled_ = false;
};

}  // namespace drone::control

#endif  // DRONE_CONTROL_POSITION_CONTROLLER_H
