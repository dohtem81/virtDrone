#ifndef GPS_PHYSICS_H
#define GPS_PHYSICS_H

// #include "drone/model/components/gps_sensor.h"
#include "drone/drone_data_types.h"  // For Position3D, Velocity3D

namespace drone::simulator::physics {

class GPSPhysics {
public:
    // static void updateGPSPhysics(drone::model::components::GPSSensor& sensor, 
    //     const drone::Position3D& new_pos, 
    //     const drone::Velocity3D& new_vel, 
    //     uint64_t dt_us);
    // // Add noise, accuracy simulation, etc.
};

}  // namespace drone::simulator::physics

#endif  // GPS_PHYSICS_H