#ifndef DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H
#define DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H

#include "drone/model/components/esc.h"
#include <algorithm>

namespace drone::model::components {
class AltitudeController {
public:
    AltitudeController() = default;
    AltitudeController(double alt_param_p,
                       double max_alt_delta_m_per_s,
                       double control_param_p = 1.0,
                       double control_param_i = 0.1,
                       double const neutral_rpm = 0.0,
                       double control_param_d = 0.0,
                       bool enable_i_component = true,
                                             bool enable_d_component = false,
                                             double activation_error_band_m = 0.5)
        : alt_param_p_(alt_param_p),
          max_alt_delta_m_(max_alt_delta_m_per_s),
          control_param_p_(control_param_p),
          control_param_i_(control_param_i),
          control_param_d_(control_param_d),
          neutral_rpm_(neutral_rpm),
          enable_i_component_(enable_i_component),
                    enable_d_component_(enable_d_component),
                    activation_error_band_m_(activation_error_band_m) {}

    void setTargetAltitude(double altitude_m) {
        target_altitude_m_ = altitude_m;
    }

    double getTargetAltitude() const {
        return target_altitude_m_;
    }

    void update(const double& current_altitude_m, const double& currentRPM, double& rpm_ref_out, double delta_time_s) {
        // calculate alt ref based on altitude error and control parameters
        (void)currentRPM;
        last_target_error_m_ = target_altitude_m_ - current_altitude_m;
        
        // Simple linear mapping for demonstration: higher altitude reference leads to higher RPM reference
        double altitude_error = target_altitude_m_ - current_altitude_m;

        // calculate RPM reference based on altitude error and control parameters
        double control_output = control_param_p_ * altitude_error; // Proportional control output
        last_p_component_rpm_ = control_output;
        const bool is_within_activation_band = std::abs(altitude_error) < activation_error_band_m_;
        // we use I part of control only when close to the reference to help reduce steady state error, otherwise it can cause overshoot and instability
        if (enable_i_component_ && is_within_activation_band) {
            i_component += altitude_error * control_param_i_ * delta_time_s; // Update integral error
            i_component = std::clamp(i_component, -5000.0, 5000.0); // Clamp integral error to prevent windup
        }

        double d_component = 0.0;
        if (enable_d_component_ && is_within_activation_band && delta_time_s > 0.0) {
            if (has_prev_altitude_error_) {
                const double altitude_error_derivative = (altitude_error - prev_altitude_error_) / delta_time_s;
                d_component = control_param_d_ * altitude_error_derivative;
            }
            has_prev_altitude_error_ = true;
            prev_altitude_error_ = altitude_error;
        } else {
            has_prev_altitude_error_ = false;
            prev_altitude_error_ = altitude_error;
        }
        last_i_component_rpm_ = i_component;
        last_d_component_rpm_ = d_component;

        control_output += i_component + d_component + neutral_rpm_; // PID control output
        //rpm_ref_out = currentRPM + control_output; // Adjust RPM reference based on control output
        rpm_ref_out = control_output;
    }

    double getLastTargetErrorM() const {
        return last_target_error_m_;
    }

    double getLastPComponentRPM() const {
        return last_p_component_rpm_;
    }

    double getLastIComponentRPM() const {
        return last_i_component_rpm_;
    }

    double getLastDComponentRPM() const {
        return last_d_component_rpm_;
    }

private:
    double target_altitude_m_ = 0.0; // Target altitude in meters
    double alt_param_p_ = 1.0; // Proportional control parameter
    double control_param_p_ = 1.0; // Proportional control parameter
    double control_param_i_ = 0.1; // Integral control parameter 
    double control_param_d_ = 0.0; // Derivative control parameter
    double i_component = 0.0; // I part 
    double max_alt_delta_m_ = 1.0; // Maximum altitude change in meters per second
    double neutral_rpm_ = 0.0; // Neutral RPM reference (e.g., hover RPM)
    bool enable_i_component_ = true;
    bool enable_d_component_ = false;
    double activation_error_band_m_ = 0.5;
    double prev_altitude_error_ = 0.0;
    bool has_prev_altitude_error_ = false;
    double last_target_error_m_ = 0.0; // target_altitude - current_altitude
    double last_p_component_rpm_ = 0.0;
    double last_i_component_rpm_ = 0.0;
    double last_d_component_rpm_ = 0.0;
};

} // namespace drone::model::components
#endif // DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H