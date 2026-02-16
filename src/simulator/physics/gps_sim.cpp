#include "simulator/physics/gps_sim.h"

namespace drone::simulator::physics {

void GPSSim::setAltitudeM(double altitude_m) {
    altitude_m_ = altitude_m;
}

// GPSSim is a thin wrapper around GPSModule_base for simulation use.
}  // namespace drone::simulator::physics
