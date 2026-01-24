#include "drone/model/sensors/base_sensor.h"

BaseSensor::BaseSensor(const std::string& name, SensorType type)
    : name_(name), status_(SensorStatus::INACTIVE), type_(type) {}

std::string BaseSensor::getName() const {
    return name_;
}

SensorStatus BaseSensor::getStatus() const {
    return status_;
}

SensorType BaseSensor::getType() const {
    return type_;
}

void BaseSensor::setStatus(SensorStatus status) {
    status_ = status;
}