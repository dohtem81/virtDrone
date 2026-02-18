#ifndef DRONE_MODEL_COMPONENTS_ESC_H
#define DRONE_MODEL_COMPONENTS_ESC_H

namespace drone::simulator::components {
class ESC_controlLogic {
public:
    static void getRPMRef_P(
        double rpm_ref,
        double rpm_current,
        double rpm_max_delta,
        double control_param_p,
        double delta_time_s,
        double& rpm_ref_out,
        double& rpm_ref_out_clamped) {

            // calculate current delta between current speed and ref
            double delta_rpm = rpm_ref - rpm_current;

            // apply proportional control
            double control_output = control_param_p * delta_rpm * delta_time_s; // Scale control output by time step
            
            // this is output not through the speed ramp
            rpm_ref_out = rpm_current + control_output;

            // clamp control output to max allowed change
            if (control_output > rpm_max_delta * delta_time_s) {
                control_output = rpm_max_delta * delta_time_s;
            } else if (control_output < -rpm_max_delta * delta_time_s) {
                control_output = -rpm_max_delta * delta_time_s;
            }
            
            // this one takes into account speed ramp
            rpm_ref_out_clamped = rpm_current + control_output;
        }
};


} // namespace drone::simulator::components
#endif // DRONE_MODEL_COMPONENTS_ESC_H