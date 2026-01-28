#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <string>
#include <cstdint>
#include "drone/model/sensors/base_sensor.h"
#include "drone/model/sensors/gps_position_sensor.h"
#include "drone/model/sensors/gps_velocity_sensor.h"
#include "drone/drone_data_types.h"  // For Position3D, Velocity3D

namespace drone::model::components {

using namespace drone::model::sensors;

// Struct for overall GPS specs
struct GPSSensorSpecs {
    double horizontal_accuracy_m;
    double vertical_accuracy_m;
    double velocity_accuracy_mps;
    uint32_t update_rate_hz;
    uint32_t max_satellites;
};

// Composite GPS sensor/component
struct GPSReading {
    Position3D position;
    Velocity3D velocity;
    uint32_t satellite_count;
    drone::model::sensors::SensorStatus status;
    uint64_t timestamp_us;
};

class GPSSensor : public BaseSensor {
public:
    GPSSensor(const std::string& name, const GPSSensorSpecs& specs, AnalogIOSpec io_spec);
    virtual ~GPSSensor() = default;

    // Implement pure virtual update (no-op or delegate to physics)
    virtual void update() override { /* Delegate to physics if needed */ }

    // Composite methods
    GPSReading getReading() const;
    const GPSSensorSpecs& getSpecs() const;

    // Access sub-sensors
    const GPSPositionSensor& getPositionSensor() const { return position_sensor_; }
    //const GPSVelocitySensor& getVelocitySensor() const { return velocity_sensor_; }

    // Setters for simulation
    void setPosition(const Position3D& pos);
    void setVelocity(const Velocity3D& vel);
    void setSatelliteCount(uint32_t count);
    void setStatus(drone::model::sensors::SensorStatus status);

private:
    GPSSensorSpecs specs_;
    GPSPositionSensor position_sensor_;
    //GPSVelocitySensor velocity_sensor_;
    uint32_t satellite_count_;
    uint64_t timestamp_us_;
};

}  // namespace drone::model::components

#endif  // GPS_SENSOR_H