#ifndef DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H
#define DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H

#include "drone/model/components/esc.h"
#include <algorithm>

namespace drone::model::components {
class AltitudeController {
public:
    AltitudeController() = default;
    AltitudeController(double alt_param_p, double max_alt_delta_m_per_s, double control_param_p = 1.0, double control_param_i = 0.1)
        : alt_param_p_(alt_param_p), max_alt_delta_m_(max_alt_delta_m_per_s), control_param_p_(control_param_p), control_param_i_(control_param_i) {}

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
        double control_output = control_param_p_ * altitude_error * delta_time_s; // Proportional control output
        integral_error_ += altitude_error * delta_time_s; // Update integral error
        control_output += control_param_i_ * integral_error_; // Integral control output
        rpm_ref_out = currentRPM + control_output; // Adjust RPM reference based on control output
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
    double integral_error_ = 0.0; // Integral error for I control 
    double max_alt_delta_m_ = 1.0; // Maximum altitude change in meters per second
};

} // namespace drone::model::components
#endif // DRONE_MODEL_COMPONENTS_ALTITUDE_CONTROLER_H