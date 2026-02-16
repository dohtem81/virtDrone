#include "drone/model/sensors/temperature_sensor.h"
#include <cstdlib>  // For rand()
#include <ctime>   // For time()
#include "drone/model/utils.h"

namespace drone::model::sensors {

TemperatureSensor::TemperatureSensor(
    const std::string& name, 
    const AnalogIOSpec& spec, 
    const TemperatureSensorRanges& ranges,
    double weight_kg)
        : BaseSensor(name, spec), ranges_(ranges), temperature_(0.0), weight_kg_(weight_kg) {
    // Seed random number generator for simulated temperature readings
    std::srand(std::time(nullptr));
    update();  // Initial update to set temperature
}

void TemperatureSensor::update() {
    // temperature calculation based on last reading
    // get counts times unit per count plus min temperature
    temperature_ = Utils::mapRange(
        Utils::clamp<uint64_t>(last_reading_, type_->counts_range.min, type_->counts_range.max),
        type_->counts_range.min,
        type_->counts_range.max,
        ranges_.min_temperature,
        ranges_.max_temperature
    );

    // adjust for ranges
    temperature_ = Utils::clamp<double>(temperature_, ranges_.min_temperature, ranges_.max_temperature);
}

double TemperatureSensor::getTemperature() const {
    return temperature_;
}

}  // namespace drone::model::sensors