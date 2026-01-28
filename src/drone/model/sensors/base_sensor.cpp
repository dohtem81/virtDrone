#include "drone/model/sensors/base_sensor.h"

namespace drone::model::sensors {

/**
 * @brief Constructor implementation.
 * Initializes the sensor with the given name and type, setting status to INACTIVE.
 */
BaseSensor::BaseSensor(const std::string& name, AnalogIOSpec type)
    : name_(name), status_(SensorStatus::INACTIVE), type_(std::make_unique<AnalogIOSpec>(type)) {}

/**
 * @brief Gets the sensor name.
 * @return The sensor name.
 */
std::string BaseSensor::getName() const {
    return name_;
}

/**
 * @brief Gets the sensor status.
 * @return Const reference to the status.
 */
const SensorStatus& BaseSensor::getStatus() const {
    return status_;
}

/**
 * @brief Gets the analog IO specification.
 * @return Const reference to the type.
 */
const AnalogIOSpec& BaseSensor::getType() const {
    return *type_;
}

/**
 * @brief Sets the sensor status.
 * @param status The new status to set.
 */
void BaseSensor::setStatus(SensorStatus status) {
    status_ = status;
}

/**
 * @brief Sets the last counts reading from the sensor.
 * @param reading The counts reading to set.
 * @return True if the reading is within valid range, false otherwise.
 */
bool BaseSensor::setLastCountsReading(uint64_t reading) {
    last_reading_ = reading;
    // call update to refresh sensor state per its specific implementation
    update();    

    if (reading < type_->counts_range.min || reading > type_->counts_range.max) {
        setStatus(SensorStatus::ERROR);
        return false;
    }

    // update sensor status to ACTIVE on valid reading
    setStatus(SensorStatus::ACTIVE);

    return true;
}

}  // namespace drone::model::sensors