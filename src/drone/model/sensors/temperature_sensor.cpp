#include "drone/model/sensors/temperature_sensor.h"
#include <cstdlib>  // For rand()
#include <ctime>   // For time()

TemperatureSensor::TemperatureSensor(const std::string& name, const TemperatureSensorRanges& ranges)
    : BaseSensor(name, SensorType::SENSING), temperature_(20.0) {
    // Seed random number generator for simulated temperature readings
    std::srand(std::time(nullptr));
    ranges_ = *ranges;
}

void TemperatureSensor::update() {
    // Simulate temperature reading with some variation
    // For a real sensor, this would read from hardware
    double variation = (std::rand() % 200 - 100) / 100.0;  // Random variation between -1.0 and 1.0
    temperature_ += variation;

    // Keep temperature within reasonable bounds (e.g., -50 to 50 degrees Celsius)
    if (temperature_ < ranges_.MIN_TEMPERATURE) temperature_ = ranges_.MIN_TEMPERATURE;
    if (temperature_ > ranges_.MAX_TEMPERATURE) temperature_ = ranges_.MAX_TEMPERATURE;

    // Set status to ACTIVE if update succeeds
    setStatus(SensorStatus::ACTIVE);
}

double TemperatureSensor::getTemperature() const {
    return temperature_;
}