#ifndef GPS_POSITION_SENSOR_H
#define GPS_POSITION_SENSOR_H

#include "drone/drone_data_types.h"  // For Position3D

namespace drone::model::sensors {

// Sub-sensor for position (GNSS-like)
class GPSPositionSensor {
public:
    GPSPositionSensor(double accuracy_m);
    drone::Position3D getPosition() const;
    void setPosition(const drone::Position3D& pos);
    double getAccuracy() const;
private:
    drone::Position3D position_;
    double accuracy_m_;
};

}  // namespace drone::model::sensors

#endif  // GPS_POSITION_SENSOR_H