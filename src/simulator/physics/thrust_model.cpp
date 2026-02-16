#include "simulator/physics/thrust_model.h"

namespace drone::simulator::physics {

static double scaleFactor(const ThrustModelParams& params) {
    return params.diameter_m * params.shape_coeff;
}

double ThrustModel::computeThrustN(double omega_rad_s, const ThrustModelParams& params) {
    const double scale = scaleFactor(params);
    return params.kT * scale * omega_rad_s * omega_rad_s;
}

double ThrustModel::computeTorqueNm(double omega_rad_s, const ThrustModelParams& params) {
    const double scale = scaleFactor(params);
    return params.kQ * scale * omega_rad_s * omega_rad_s;
}

}  // namespace drone::simulator::physics