#include "drone/model/components/gps_sensor.h"
#include <chrono>

using namespace drone::model::components;
using namespace drone::model::sensors;

GPSSensor::GPSSensor(const std::string& name, const GPSSensorSpecs& specs, AnalogIOSpec io_spec)
    : BaseSensor(name, io_spec),
      specs_(specs),
      position_sensor_(specs.horizontal_accuracy_m),  // Fully qualify if needed: drone::model::sensors::GPSPositionSensor(specs.horizontal_accuracy_m)
      velocity_sensor_(specs.velocity_accuracy_mps),  // Fully qualify if needed: drone::model::sensors::GPSVelocitySensor(specs.velocity_accuracy_mps)
      satellite_count_(0),
      timestamp_us_(0) {}

void GPSSensor::update() {
    // No-op: Simulation handled externally by physics module
}

GPSReading GPSSensor::getReading() const {
    return {
        position_sensor_.getPosition(),
        velocity_sensor_.getVelocity(),
        satellite_count_,
        getStatus(),
        timestamp_us_
    };
}

const GPSSensorSpecs& GPSSensor::getSpecs() const {
    return specs_;
}

void GPSSensor::setPosition(const Position3D& pos) {
    position_sensor_.setPosition(pos);
    timestamp_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void GPSSensor::setVelocity(const Velocity3D& vel) {
    velocity_sensor_.setVelocity(vel);
    timestamp_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void GPSSensor::setSatelliteCount(uint32_t count) {
    satellite_count_ = count;
}

void GPSSensor::setStatus(SensorStatus status) {
    BaseSensor::setStatus(status);
}

}  // namespace drone::model::components


namespace drone::model::sensors {
// GPSVelocitySensor implementation
drone::model::sensors::GPSVelocitySensor::GPSVelocitySensor(double accuracy_mps) : accuracy_mps_(accuracy_mps) {}

drone::Velocity3D drone::model::sensors::GPSVelocitySensor::getVelocity() const {
    return velocity_;
}

void drone::model::sensors::GPSVelocitySensor::setVelocity(const drone::Velocity3D& vel) {
    velocity_ = vel;
}

double drone::model::sensors::GPSVelocitySensor::getAccuracy() const {
    return accuracy_mps_;
}

// GPSPositionSensor implementation
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


