// drone_base.h
#ifndef DRONE_BASE_H
#define DRONE_BASE_H

#include <memory>
#include <string>
#include <vector>

#include "drone/model/components/battery_base.h"
#include "drone/model/components/elect_motor.h"
#include "drone/model/components/gps_module_base.h"
#include "drone/model/sensors/temperature_sensor.h"

namespace drone::model {

/**
 * @brief Base class for any drone model.
 *
 * This class owns the core components common to all drones:
 * motors, a battery, and a temperature sensor.
 */
class DroneBase {
public:
		DroneBase(const std::string& name,
							std::vector<components::ElecMotor> motors,
							std::unique_ptr<components::Battery_base> battery,
							std::unique_ptr<sensors::TemperatureSensor> temperature_sensor,
							std::unique_ptr<components::GPSModule_base> gps,
							double body_weight_kg)
		: name_(name),
		  motors_(std::move(motors)),
		  battery_(std::move(battery)),
					temperature_sensor_(std::move(temperature_sensor)),
					gps_(std::move(gps)),
					body_weight_kg_(body_weight_kg) {}

	virtual ~DroneBase() = default;

	const std::string& getName() const { return name_; }

	std::vector<components::ElecMotor>& getMotors() { return motors_; }
	const std::vector<components::ElecMotor>& getMotors() const { return motors_; }
	void setMotors(std::vector<components::ElecMotor> motors) { motors_ = std::move(motors); }

	components::Battery_base* getBattery() const { return battery_.get(); }
	void setBattery(std::unique_ptr<components::Battery_base> battery) { battery_ = std::move(battery); }

	sensors::TemperatureSensor* getTemperatureSensor() const { return temperature_sensor_.get(); }
	void setTemperatureSensor(std::unique_ptr<sensors::TemperatureSensor> sensor) { temperature_sensor_ = std::move(sensor); }

	components::GPSModule_base* getGPS() const { return gps_.get(); }
	void setGPS(std::unique_ptr<components::GPSModule_base> gps) { gps_ = std::move(gps); }

	double getAltitudeM() const {
		if (!gps_) {
			return 0.0;
		}
		return gps_->getPosition().altitude_m;
	}

	double getBodyWeightKg() const { return body_weight_kg_; }
	void setBodyWeightKg(double weight_kg) { body_weight_kg_ = weight_kg; }

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
	std::string name_;
	std::vector<components::ElecMotor> motors_;
	std::unique_ptr<components::Battery_base> battery_;
	std::unique_ptr<sensors::TemperatureSensor> temperature_sensor_;
	std::unique_ptr<components::GPSModule_base> gps_;
	double body_weight_kg_;
};

}  // namespace drone::model

#endif  // DRONE_BASE_H
