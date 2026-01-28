#include "drone/model/sensors/gps_position_sensor.h"
#include "drone/drone_data_types.h"  // For Position3D

namespace drone::model::sensors {

GPSPositionSensor::GPSPositionSensor(double accuracy_m) : accuracy_m_(accuracy_m) {}

drone::Position3D GPSPositionSensor::getPosition() const {
    return position_;
}

void GPSPositionSensor::setPosition(const drone::Position3D& pos) {
    position_ = pos;
}

double GPSPositionSensor::getAccuracy() const {
    return accuracy_m_;
}

}  // namespace drone::model::sensors