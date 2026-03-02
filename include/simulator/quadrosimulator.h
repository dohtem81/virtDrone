#ifndef QUADROSIMULATOR_H
#define QUADROSIMULATOR_H

#include "simulation_base.h"
#include "drone/runtime/real_drone.h"
#include "drone/model/quadrocopter.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/physics/thrust_model.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/gps_sim.h"
#include "drone/model/drone_base.h"
#include <memory>
#include <string>

namespace drone::simulator {

class QuaroSimulation final : public drone::simulator::SimulationBase,
                              public drone::runtime::SensorSource,
                              public drone::runtime::ActuatorSink {
public:
    friend std::shared_ptr<QuaroSimulation> QuadroSimulationFactory(
        std::string name,
        drone::model::components::ElecMotorSpecs emSpecs,
        drone::model::sensors::AnalogIOSpec aIOSpec,
        drone::model::components::BatterySpecs batterySpecs,
        drone::model::sensors::AnalogIOSpec tempIOSpec,
        drone::model::sensors::TemperatureSensorRanges tempSensorRanges,
        double temp_sensor_weight_kg,
        drone::model::components::GPSSensorSpecs gpsSpecs,
        double body_weight_kg,
        double blade_diameter_m,
        double blade_shape_coefficient,
        uint64_t steps,
        double dt_s);

    drone::runtime::SensorFrame readSensors() const override;
    void applyActuators(const drone::runtime::ActuatorFrame& actuator_frame) override;

protected:
    void onStart();
    void onStop();
    void onStep(double dt_s);

public:
    ~QuaroSimulation() = default;

private:
    QuaroSimulation() = default;
    std::unique_ptr<drone::model::Quadrocopter> quad_;
    double elapsed_s_;
    double altitude_m_{0.0};
    double vertical_speed_mps_{0.0};
    double desired_rpm_{0.0};
    double target_altitude_m_{0.0};
    double target_error_m_{0.0};
    double p_component_rpm_{0.0};
    double i_component_rpm_{0.0};
    double d_component_rpm_{0.0};
    double sensed_altitude_m_{0.0};
    double sensed_battery_voltage_v_{0.0};
    double sensed_battery_soc_percent_{0.0};
    double sensed_motor_temperature_c_{0.0};
    double sensed_motor_rpm_{0.0};
    bool is_running_ = false;
};

/**
 * @brief Factory function to create a QuaroSimulation instance.
 */
std::shared_ptr<QuaroSimulation> QuadroSimulationFactory(
    std::string name,
    drone::model::components::ElecMotorSpecs emSpecs,
    drone::model::sensors::AnalogIOSpec aIOSpec,
    drone::model::components::BatterySpecs batterySpecs,
    drone::model::sensors::AnalogIOSpec tempIOSpec,
    drone::model::sensors::TemperatureSensorRanges tempSensorRanges,
    double temp_sensor_weight_kg,
    drone::model::components::GPSSensorSpecs gpsSpecs,
    double body_weight_kg,
    double blade_diameter_m,
    double blade_shape_coefficient,
    uint64_t steps,
    double dt_s);

}  // namespace drone::simulator
#endif  // QUADROSIMULATOR_H