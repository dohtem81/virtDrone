#ifndef SIMULATOR_PHYSICS_FORCE_DYNAMICS_H
#define SIMULATOR_PHYSICS_FORCE_DYNAMICS_H

#include "drone/drone_data_types.h"

namespace drone::simulator::physics {

drone::Vector3 rotateBodyToEnu(const drone::Vector3& body_vec, const drone::AttitudeYPR& attitude_ypr);

drone::Vector3 computeGravityEnu(double mass_kg, double gravity_ms2 = 9.81);

drone::Vector3 computeLinearDampingEnu(const drone::Vector3& velocity_enu_mps, double damping_n_per_mps);

drone::Vector3 computeNetForceEnu(
    const drone::Vector3& thrust_body_n,
    const drone::AttitudeYPR& attitude_ypr,
    double mass_kg,
    const drone::Vector3& velocity_enu_mps,
    double damping_n_per_mps,
    double gravity_ms2 = 9.81);

}  // namespace drone::simulator::physics

#endif  // SIMULATOR_PHYSICS_FORCE_DYNAMICS_H
