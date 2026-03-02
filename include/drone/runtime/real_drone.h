#ifndef DRONE_RUNTIME_REAL_DRONE_H
#define DRONE_RUNTIME_REAL_DRONE_H

#include "drone/model/components/altitude_controler.h"

namespace drone::runtime {

struct SensorFrame {
    double altitude_m = 0.0;
    double battery_voltage_v = 0.0;
    double battery_soc_percent = 0.0;
    double motor_temperature_c = 0.0;
    double motor_rpm = 0.0;
};

struct ActuatorFrame {
    double desired_motor_rpm = 0.0;
    double target_altitude_m = 0.0;
    double target_error_m = 0.0;
    double p_component_rpm = 0.0;
    double i_component_rpm = 0.0;
    double d_component_rpm = 0.0;
    double sensed_altitude_m = 0.0;
    double sensed_battery_voltage_v = 0.0;
    double sensed_battery_soc_percent = 0.0;
    double sensed_motor_temperature_c = 0.0;
    double sensed_motor_rpm = 0.0;
};

class SensorSource {
public:
    virtual ~SensorSource() = default;
    virtual SensorFrame readSensors() const = 0;
};

class ActuatorSink {
public:
    virtual ~ActuatorSink() = default;
    virtual void applyActuators(const ActuatorFrame& actuator_frame) = 0;
};

class RealDrone {
public:
    explicit RealDrone(const model::components::AltitudeController& altitude_controller)
        : altitude_controller_(altitude_controller) {}

    void setTargetAltitude(double target_altitude_m) {
        altitude_controller_.setTargetAltitude(target_altitude_m);
    }

    void update(double dt_s, const SensorSource& sensor_source, ActuatorSink& actuator_sink) {
        const SensorFrame sensors = sensor_source.readSensors();
        double desired_motor_rpm = 0.0;
        altitude_controller_.update(
            sensors.altitude_m,
            sensors.motor_rpm,
            desired_motor_rpm,
            dt_s);
        actuator_sink.applyActuators(ActuatorFrame{
            desired_motor_rpm,
            altitude_controller_.getTargetAltitude(),
            altitude_controller_.getLastTargetErrorM(),
            altitude_controller_.getLastPComponentRPM(),
            altitude_controller_.getLastIComponentRPM(),
            altitude_controller_.getLastDComponentRPM(),
            sensors.altitude_m,
            sensors.battery_voltage_v,
            sensors.battery_soc_percent,
            sensors.motor_temperature_c,
            sensors.motor_rpm});
    }

private:
    model::components::AltitudeController altitude_controller_;
};

}  // namespace drone::runtime

#endif  // DRONE_RUNTIME_REAL_DRONE_H
