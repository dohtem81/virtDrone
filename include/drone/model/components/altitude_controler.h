#ifndef DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H
#define DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H

#include "drone/model/components/esc.h"
#include <algorithm>

namespace drone::model::components {
class AltitudeController {
public:
    AltitudeController() = default;
    AltitudeController(double alt_param_p, double max_alt_delta_m_per_s, double control_param_p = 1.0, double control_param_i = 0.1, double const neutral_rpm = 0.0)
        : alt_param_p_(alt_param_p), max_alt_delta_m_(max_alt_delta_m_per_s), control_param_p_(control_param_p), control_param_i_(control_param_i), neutral_rpm_(neutral_rpm) {}

    void setTargetAltitude(double altitude_m) {
        target_altitude_m_ = altitude_m;
    }

    double getTargetAltitude() const {
        return target_altitude_m_;
    }

    void updateAltRefInUse(const double& current_altitude_m, double delta_time_s) {
        double altitude_error = target_altitude_m_ - current_altitude_m;
        altitude_ref_inuse_ += alt_param_p_ * altitude_error * delta_time_s; // Proportional control output
        // Clamp the altitude reference change to max_alt_delta_m_ per second
        altitude_ref_inuse_ = 
            std::clamp(altitude_ref_inuse_, 
                current_altitude_m - max_alt_delta_m_ * delta_time_s, 
                current_altitude_m + max_alt_delta_m_ * delta_time_s);
    }

    void update(const double& current_altitude_m, double altitude_ref_inuse, const double& currentRPM, double& rpm_ref_out, double delta_time_s) {
        // calculate alt ref based on altitude error and control parameters
        updateAltRefInUse(current_altitude_m, delta_time_s);
        
        // Simple linear mapping for demonstration: higher altitude reference leads to higher RPM reference
        double altitude_error = altitude_ref_inuse_ - current_altitude_m;

        // calculate RPM reference based on altitude error and control parameters
        double control_output = control_param_p_ * altitude_error; // Proportional control output
        // we use I part of control only when close to the reference to help reduce steady state error, otherwise it can cause overshoot and instability
        if (std::abs(altitude_error) < 0.5) { // only use I control when within 1 meter of the reference
            i_component += altitude_error * control_param_i_ * delta_time_s; // Update integral error
            i_component = std::clamp(i_component, -5000.0, 5000.0); // Clamp integral error to prevent windup
        }

        control_output += i_component + neutral_rpm_; // Integral control output
        //rpm_ref_out = currentRPM + control_output; // Adjust RPM reference based on control output
        rpm_ref_out = control_output;
    }

    double getAltitudeRefInUse() const {
        return altitude_ref_inuse_;
    }

private:
    double target_altitude_m_ = 0.0; // Target altitude in meters
    double altitude_ref_inuse_ = 0.0; // Current altitude reference being used (for internal tracking)
    double alt_param_p_ = 1.0; // Proportional control parameter
    double control_param_p_ = 1.0; // Proportional control parameter
    double control_param_i_ = 0.1; // Integral control parameter 
    double i_component = 0.0; // I part 
    double max_alt_delta_m_ = 1.0; // Maximum altitude change in meters per second
    double neutral_rpm_ = 0.0; // Neutral RPM reference (e.g., hover RPM)
};

} // namespace drone::model::components
#endif // DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H