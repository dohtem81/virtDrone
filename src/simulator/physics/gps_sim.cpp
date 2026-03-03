#include "simulator/physics/gps_sim.h"

#include <algorithm>
#include <cmath>

namespace drone::simulator::physics {

namespace {
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kMinCosLatitude = 1.0e-6;
}  // namespace

void GPSSim::setAltitudeM(double altitude_m) {
    Position3D pos = getPosition();
    pos.altitude_m = altitude_m;
    setPosition(pos);
}

void GPSSim::setReferenceGeodetic(const drone::Position3D& reference_position) {
    reference_position_ = reference_position;
}

void GPSSim::setPerfectEnuState(const drone::Vector3& position_enu_m, const drone::Vector3& velocity_enu_mps) {
    const double ref_lat_rad = reference_position_.latitude_deg * kDegToRad;
    const double cos_ref_lat = std::max(kMinCosLatitude, std::abs(std::cos(ref_lat_rad)));

    const double delta_lat_rad = position_enu_m.y / kEarthRadiusM;
    const double delta_lon_rad = position_enu_m.x / (kEarthRadiusM * cos_ref_lat);

    Position3D position;
    position.latitude_deg = reference_position_.latitude_deg + delta_lat_rad * kRadToDeg;
    position.longitude_deg = reference_position_.longitude_deg + delta_lon_rad * kRadToDeg;
    position.altitude_m = reference_position_.altitude_m + position_enu_m.z;
    setPosition(position);

    Velocity3D velocity;
    velocity.north_mps = velocity_enu_mps.y;
    velocity.east_mps = velocity_enu_mps.x;
    velocity.down_mps = -velocity_enu_mps.z;
    setVelocity(velocity);
}

// GPSSim is a thin wrapper around GPSModule_base for simulation use.
}  // namespace drone::simulator::physics
