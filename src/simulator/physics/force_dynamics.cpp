#include "simulator/physics/force_dynamics.h"

#include <cmath>

namespace drone::simulator::physics {

drone::Vector3 rotateBodyToEnu(const drone::Vector3& body_vec, const drone::AttitudeYPR& attitude_ypr) {
    const double yaw = attitude_ypr.yaw_rad;
    const double pitch = attitude_ypr.pitch_rad;
    const double roll = attitude_ypr.roll_rad;

    const double cz = std::cos(yaw);
    const double sz = std::sin(yaw);
    const double cy = std::cos(pitch);
    const double sy = std::sin(pitch);
    const double cx = std::cos(roll);
    const double sx = std::sin(roll);

    const double x = (cz * cy) * body_vec.x + (cz * sy * sx - sz * cx) * body_vec.y + (cz * sy * cx + sz * sx) * body_vec.z;
    const double y = (sz * cy) * body_vec.x + (sz * sy * sx + cz * cx) * body_vec.y + (sz * sy * cx - cz * sx) * body_vec.z;
    const double z = (-sy) * body_vec.x + (cy * sx) * body_vec.y + (cy * cx) * body_vec.z;

    return drone::Vector3(x, y, z);
}

drone::Vector3 computeGravityEnu(double mass_kg, double gravity_ms2) {
    return drone::Vector3(0.0, 0.0, -mass_kg * gravity_ms2);
}

drone::Vector3 computeLinearDampingEnu(const drone::Vector3& velocity_enu_mps, double damping_n_per_mps) {
    return velocity_enu_mps * damping_n_per_mps;
}

drone::Vector3 computeNetForceEnu(
    const drone::Vector3& thrust_body_n,
    const drone::AttitudeYPR& attitude_ypr,
    double mass_kg,
    const drone::Vector3& velocity_enu_mps,
    double damping_n_per_mps,
    double gravity_ms2) {
    const drone::Vector3 thrust_enu_n = rotateBodyToEnu(thrust_body_n, attitude_ypr);
    const drone::Vector3 gravity_enu_n = computeGravityEnu(mass_kg, gravity_ms2);
    const drone::Vector3 damping_enu_n = computeLinearDampingEnu(velocity_enu_mps, damping_n_per_mps);
    return thrust_enu_n + gravity_enu_n - damping_enu_n;
}

}  // namespace drone::simulator::physics
