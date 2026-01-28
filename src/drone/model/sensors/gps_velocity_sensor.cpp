#include "drone/model/sensors/gps_velocity_sensor.h"

namespace drone::model::sensors {

GPSVelocitySensor::GPSVelocitySensor(double accuracy_mps) : accuracy_mps_(accuracy_mps) {}

Velocity3D GPSVelocitySensor::getVelocity() const {
    return velocity_;
}

void GPSVelocitySensor::setVelocity(const Velocity3D& vel) {
    velocity_ = vel;
}

double GPSVelocitySensor::getAccuracy() const {
    return accuracy_mps_;
}

}  // namespace drone::model::components