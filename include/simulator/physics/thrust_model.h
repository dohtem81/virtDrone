#pragma once

#include <cstdint>

#include "drone/model/components/elect_motor.h"

namespace drone::simulator::physics {

struct ThrustModelParams {
    double kT;             // base thrust coefficient
    double kQ;             // base torque coefficient
    double diameter_m;     // blade diameter [m]
    double shape_coeff;    // simplified shape factor (dimensionless)
};

class ThrustModel final {
public:
    // omega_rad_s: angular velocity [rad/s]
    static double computeThrustN(double omega_rad_s, const ThrustModelParams& params);
    static double computeTorqueNm(double omega_rad_s, const ThrustModelParams& params);

    // motor: uses current RPM to compute omega
    static double computeThrustN(const drone::model::components::ElecMotor* motor,
                                 const ThrustModelParams& params);
    static double computeTorqueNm(const drone::model::components::ElecMotor* motor,
                                  const ThrustModelParams& params);
};

}  // namespace drone::simulator::physics