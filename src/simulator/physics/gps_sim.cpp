#include "simulator/physics/gps_sim.h"

namespace drone::simulator::physics {

void GPSSim::setAltitudeM(double altitude_m) {
    // Update the position in the base class
    Position3D pos = getPosition();
    pos.altitude_m = altitude_m;
    setPosition(pos);
}

// GPSSim is a thin wrapper around GPSModule_base for simulation use.
}  // namespace drone::simulator::physics
