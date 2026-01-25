#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "drone/model/sensors/base_sensor.h"

struct TemperatureSensorRanges {
    static constexpr double MIN_TEMPERATURE = -50.0; // Minimum temperature in Celsius
    static constexpr double MAX_TEMPERATURE = 50.0;  // Maximum temperature in Celsius
};

class TemperatureSensor : public BaseSensor {
public:
    TemperatureSensor(const std::string& name, const TemperatureSensorRanges* ranges);
    virtual ~TemperatureSensor() = default;

    // Implement the pure virtual update method
    void update() override;

    // Temperature-specific methods
    double getTemperature() const;

private:
    double temperature_;  // Current temperature reading
    TemperatureSensorRanges ranges_;
};

#endif // TEMPERATURE_SENSOR_H
