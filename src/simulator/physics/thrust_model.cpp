#include "simulator/physics/thrust_model.h"

#include <cmath>

namespace drone::simulator::physics {

static double scaleFactor(const ThrustModelParams& params) {
    return params.diameter_m * params.shape_coeff;
}

static double rpmToRadPerSec(double rpm) {
    return rpm * 2.0 * M_PI / 60.0;
}

double ThrustModel::computeThrustN(double omega_rad_s, const ThrustModelParams& params) {
    const double scale = scaleFactor(params);
    return params.kT * scale * omega_rad_s * omega_rad_s;
}

double ThrustModel::computeTorqueNm(double omega_rad_s, const ThrustModelParams& params) {
    const double scale = scaleFactor(params);
    return params.kQ * scale * omega_rad_s * omega_rad_s;
}

double ThrustModel::computeThrustN(const drone::model::components::ElecMotor* motor,
                                   const ThrustModelParams& params) {
    if (!motor) {
        return 0.0;
    }
    ThrustModelParams adjusted = params;
    adjusted.diameter_m = motor->getSpecs().blade_diameter_m;
    adjusted.shape_coeff = motor->getSpecs().blade_shape_coeff;

    const double omega = rpmToRadPerSec(motor->getSpeedRPM());
    return computeThrustN(omega, adjusted);
}

double ThrustModel::computeTorqueNm(const drone::model::components::ElecMotor* motor,
                                    const ThrustModelParams& params) {
    if (!motor) {
        return 0.0;
    }
    ThrustModelParams adjusted = params;
    adjusted.diameter_m = motor->getSpecs().blade_diameter_m;
    adjusted.shape_coeff = motor->getSpecs().blade_shape_coeff;

    const double omega = rpmToRadPerSec(motor->getSpeedRPM());
    return computeTorqueNm(omega, adjusted);
}

}  // namespace drone::simulator::physics