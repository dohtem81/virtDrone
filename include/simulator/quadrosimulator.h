#ifndef QUADROSIMULATOR_H
#define QUADROSIMULATOR_H

#include "simulation_base.h"
#include "drone/model/quadrocopter.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/physics/thrust_model.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/gps_sim.h"
#include "drone/model/drone_base.h"
#include "drone/model/components/altitude_controler.h"
#include <memory>
#include <string>

namespace drone::simulator {

class QuaroSimulation final : public drone::simulator::SimulationBase {
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
        drone::model::components::AltitudeController altitudeController,
        uint64_t steps,
        double dt_s);

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
    drone::model::components::AltitudeController altitudeController,
    uint64_t steps,
    double dt_s);

}  // namespace drone::simulator
#endif  // QUADROSIMULATOR_H