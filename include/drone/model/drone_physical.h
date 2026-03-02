// drone_physical.h
#ifndef DRONE_PHYSICAL_H
#define DRONE_PHYSICAL_H

#include <memory>
#include <string>

#include "drone/model/components/battery_base.h"
#include "drone/model/components/gps_module_base.h"
#include "drone/model/sensors/temperature_sensor.h"

namespace drone::model {

/**
 * @brief Represents the physical drone from the drone's perspective.
 *
 * This class provides sensor data endpoints - only the information that the drone's
 * actual sensors can provide. This is the interface that the drone firmware/software
 * would interact with, representing what the drone "knows" about itself through sensors.
 *
 * This class is not concerned with physics calculations or detailed component management.
 * It serves as a contract for what sensor readings are available.
 */
class DronePhysical {
public:
    DronePhysical(const std::string& name,
                  std::unique_ptr<components::Battery_base> battery,
                  std::unique_ptr<sensors::TemperatureSensor> temperature_sensor,
                  std::unique_ptr<components::GPSModule_base> gps)
        : name_(name),
          battery_(std::move(battery)),
          temperature_sensor_(std::move(temperature_sensor)),
          gps_(std::move(gps)) {}

    virtual ~DronePhysical() = default;

    // Explicitly define move semantics
    DronePhysical(DronePhysical&&) = default;
    DronePhysical& operator=(DronePhysical&&) = default;

    // Delete copy semantics (unique_ptr members make it non-copyable)
    DronePhysical(const DronePhysical&) = delete;
    DronePhysical& operator=(const DronePhysical&) = delete;

    // Sensor data endpoints - read-only information available to the drone

    /// @brief Get the drone's name/identifier
    const std::string& getName() const { return name_; }

    /// @brief Get current altitude in meters (from GPS)
    double getAltitudeM() const {
        if (!gps_) {
            return 0.0;
        }
        return gps_->getPosition().altitude_m;
    }

    /// @brief Get battery voltage (from battery sensor)
    double getBatteryVoltageV() const {
        if (!battery_) {
            return 0.0;
        }
        return battery_->getVoltageV();
    }

    /// @brief Get battery state of charge (from battery sensor)
    double getBatterySOC() const {
        if (!battery_) {
            return 0.0;
        }
        return battery_->getStateOfChargePercent();
    }

    /// @brief Get temperature reading (from temperature sensor)
    double getTemperatureC() const {
        if (!temperature_sensor_) {
            return 0.0;
        }
        return temperature_sensor_->getTemperature();
    }

    /// @brief Get GPS position data
    components::GPSModule_base* getGPS() const { return gps_.get(); }

    /// @brief Get battery sensor
    components::Battery_base* getBattery() const { return battery_.get(); }

    /// @brief Get temperature sensor
    sensors::TemperatureSensor* getTemperatureSensor() const { return temperature_sensor_.get(); }

protected:
    std::string name_;
    std::unique_ptr<components::Battery_base> battery_;
    std::unique_ptr<sensors::TemperatureSensor> temperature_sensor_;
    std::unique_ptr<components::GPSModule_base> gps_;

    // Allow derived classes to update sensor components
    void setBattery(std::unique_ptr<components::Battery_base> battery) {
        battery_ = std::move(battery);
    }

    void setTemperatureSensor(std::unique_ptr<sensors::TemperatureSensor> sensor) {
        temperature_sensor_ = std::move(sensor);
    }

    void setGPS(std::unique_ptr<components::GPSModule_base> gps) {
        gps_ = std::move(gps);
    }
};

}  // namespace drone::model

#endif  // DRONE_PHYSICAL_H
