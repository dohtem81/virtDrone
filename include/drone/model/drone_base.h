// drone_base.h
#ifndef DRONE_BASE_H
#define DRONE_BASE_H

#include <memory>
#include <string>
#include <vector>

#include "drone/model/components/battery_base.h"
#include "drone/model/components/elect_motor.h"
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
							std::unique_ptr<sensors::TemperatureSensor> temperature_sensor)
		: name_(name),
		  motors_(std::move(motors)),
		  battery_(std::move(battery)),
					temperature_sensor_(std::move(temperature_sensor)) {}

	virtual ~DroneBase() = default;

	const std::string& getName() const { return name_; }

	std::vector<components::ElecMotor>& getMotors() { return motors_; }
	const std::vector<components::ElecMotor>& getMotors() const { return motors_; }
	void setMotors(std::vector<components::ElecMotor> motors) { motors_ = std::move(motors); }

	components::Battery_base* getBattery() const { return battery_.get(); }
	void setBattery(std::unique_ptr<components::Battery_base> battery) { battery_ = std::move(battery); }

	sensors::TemperatureSensor* getTemperatureSensor() const { return temperature_sensor_.get(); }
	void setTemperatureSensor(std::unique_ptr<sensors::TemperatureSensor> sensor) { temperature_sensor_ = std::move(sensor); }

private:
	std::string name_;
	std::vector<components::ElecMotor> motors_;
	std::unique_ptr<components::Battery_base> battery_;
	std::unique_ptr<sensors::TemperatureSensor> temperature_sensor_;
};

}  // namespace drone::model

#endif  // DRONE_BASE_H
