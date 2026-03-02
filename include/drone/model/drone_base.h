// drone_base.h
#ifndef DRONE_BASE_H
#define DRONE_BASE_H

#include <memory>
#include <string>
#include <vector>

#include "drone/model/components/battery_base.h"
#include "drone/model/components/elect_motor.h"
#include "drone/model/components/gps_module_base.h"
#include "drone/model/drone_physical.h"
#include "drone/model/sensors/temperature_sensor.h"

namespace drone::model {

/**
 * @brief Simulator base class for drone models with full physics calculations.
 *
 * This class extends DronePhysical and adds:
 * - Motor component management
 * - Physics calculations (total weight, power consumption, etc.)
 * - Complete component lifecycle management
 *
 * This is used in simulation environments where all physical aspects need to be calculated.
 */
class DroneBase : public DronePhysical {
public:
    DroneBase(const std::string& name,
              std::vector<components::ElecMotor> motors,
              std::unique_ptr<components::Battery_base> battery,
              std::unique_ptr<sensors::TemperatureSensor> temperature_sensor,
              std::unique_ptr<components::GPSModule_base> gps,
              double body_weight_kg)
        : DronePhysical(name, std::move(battery), std::move(temperature_sensor), std::move(gps)),
          motors_(std::move(motors)),
          body_weight_kg_(body_weight_kg) {}

    virtual ~DroneBase() = default;

    // Explicitly define move semantics
    DroneBase(DroneBase&&) = default;
    DroneBase& operator=(DroneBase&&) = default;

    // Delete copy semantics
    DroneBase(const DroneBase&) = delete;
    DroneBase& operator=(const DroneBase&) = delete;

    // Motor management - simulation-specific
    std::vector<components::ElecMotor>& getMotors() { return motors_; }
    const std::vector<components::ElecMotor>& getMotors() const { return motors_; }
    void setMotors(std::vector<components::ElecMotor> motors) { motors_ = std::move(motors); }

    // Battery component management - simulation-specific
    void setBattery(std::unique_ptr<components::Battery_base> battery) {
        DronePhysical::setBattery(std::move(battery));
    }

    // Temperature sensor management - simulation-specific
    void setTemperatureSensor(std::unique_ptr<sensors::TemperatureSensor> sensor) {
        DronePhysical::setTemperatureSensor(std::move(sensor));
    }

    // GPS management - simulation-specific
    void setGPS(std::unique_ptr<components::GPSModule_base> gps) {
        DronePhysical::setGPS(std::move(gps));
    }

    // Physics calculations - simulation-specific
    double getBodyWeightKg() const { return body_weight_kg_; }
    void setBodyWeightKg(double weight_kg) { body_weight_kg_ = weight_kg; }

    /**
     * @brief Calculate total drone weight including all components.
     *
     * This is a physics calculation combining:
     * - Body weight
     * - All motor weights
     * - Battery weight
     * - Sensor weights (temperature, GPS)
     *
     * @return Total weight in kilograms
     */
    double getTotalWeightKg() const {
        double total = body_weight_kg_;
        for (const auto& motor : motors_) {
            total += motor.getWeightKg();
        }
        if (battery_) {
            total += battery_->getWeightKg();
        }
        if (temperature_sensor_) {
            total += temperature_sensor_->getWeightKg();
        }
        if (gps_) {
            total += gps_->getWeightKg();
        }
        return total;
    }

private:
    std::vector<components::ElecMotor> motors_;
    double body_weight_kg_;
};

}  // namespace drone::model

#endif  // DRONE_BASE_H
