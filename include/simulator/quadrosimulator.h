#ifndef QUADROSIMULATOR_H
#define QUADROSIMULATOR_H

#include "simulation_base.h"
#include "drone/runtime/real_drone.h"
#include "drone/model/quadrocopter.h"
#include "drone/drone_data_types.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/physics/thrust_model.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/gps_sim.h"
#include "simulator/environment/weather_model.h"
#include "simulator/config/weather_config.h"
#include "drone/model/drone_base.h"
#include <array>
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
    void setWeatherConfig(const drone::simulator::config::WeatherConfig& weather_config);

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
    drone::Vector3 position_enu_m_{};
    drone::Vector3 velocity_enu_mps_{};
    drone::Vector3 acceleration_enu_ms2_{};
    drone::AttitudeYPR attitude_ypr_rad_{};
    double altitude_m_{0.0};
    double vertical_speed_mps_{0.0};
    double desired_rpm_{0.0};
    double common_motor_rpm_{0.0};
    std::array<double, drone::runtime::kMotorCount> desired_motor_rpm_each_{};
    double target_altitude_m_{0.0};
    double target_error_m_{0.0};
    double p_component_rpm_{0.0};
    double i_component_rpm_{0.0};
    double d_component_rpm_{0.0};
    double yaw_control_rpm_{0.0};
    double pitch_control_rpm_{0.0};
    double roll_control_rpm_{0.0};
    double sensed_altitude_m_{0.0};
    double sensed_gps_latitude_deg_{0.0};
    double sensed_gps_longitude_deg_{0.0};
    double sensed_gps_altitude_m_{0.0};
    double sensed_gps_velocity_north_mps_{0.0};
    double sensed_gps_velocity_east_mps_{0.0};
    double sensed_gps_velocity_down_mps_{0.0};
    drone::simulator::environment::WeatherModel weather_model_{};
    drone::simulator::environment::WeatherSample weather_sample_{};
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