#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "drone/model/sensors/base_sensor.h"
#include <memory>

namespace drone::model::sensors {

struct TemperatureSensorRanges {
    double min_temperature;  // Minimum temperature in Celsius
    double max_temperature;  // Maximum temperature in Celsius

    TemperatureSensorRanges(double min = -50.0, double max = 50.0)
        : min_temperature(min), max_temperature(max) {}
};

struct TemperatureSensorReading {
    double temperature;  // Temperature in gieven units
    std::string units;  // Units of measurement
    SensorStatus status;  // Sensor status

    // Constructor
    TemperatureSensorReading(double temp = 0.0, const std::string& unit = "Celsius", SensorStatus stat = SensorStatus::INACTIVE)
        : temperature(temp), units(unit), status(stat) {}
};

class TemperatureSensor : public BaseSensor {
public:
    TemperatureSensor(const std::string& name, 
        const AnalogIOSpec& spec, const TemperatureSensorRanges& ranges = TemperatureSensorRanges(), double weight_kg = 0.02);
    virtual ~TemperatureSensor() = default;

    TemperatureSensor(const TemperatureSensor&) = delete;
    TemperatureSensor& operator=(const TemperatureSensor&) = delete;
    TemperatureSensor(TemperatureSensor&&) noexcept = default;
    TemperatureSensor& operator=(TemperatureSensor&&) noexcept = default;

    // Implement the pure virtual update method
    void update() override;

    // Temperature-specific methods
    double getTemperature() const;

    // get ranges
    const TemperatureSensorRanges* getRanges() const { return &ranges_; }

    // get units
    std::string getUnits() const { return units; }

    // get weight
    double getWeightKg() const { return weight_kg_; }

private:
    double temperature_;  // Current temperature reading
    TemperatureSensorRanges ranges_;
    std::string units = "Celsius";
    double weight_kg_;
};

}  // namespace drone::model::sensors

#endif // TEMPERATURE_SENSOR_H
