#ifndef GPS_VELOCITY_SENSOR_H
#define GPS_VELOCITY_SENSOR_H

#include "drone/drone_data_types.h"  // For Velocity3D
#include "simulator/physics/gps_physics.h"

namespace drone::model::sensors {

class GPSVelocitySensor {
public:
    GPSVelocitySensor(double accuracy_mps);
    Velocity3D getVelocity() const;
    double getAccuracy() const;

private:
    friend class drone::simulator::physics::GPSPhysics;  // Allow only GPSPhysics to set
    void setVelocity(const Velocity3D& vel);

    Velocity3D velocity_;
    double accuracy_mps_;
};

}  // namespace drone::model::sensors

#endif  // GPS_VELOCITY_SENSOR_H