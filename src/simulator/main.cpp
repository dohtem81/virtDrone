#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

#include "drone/model/quadrocopter.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/simulation_base.h"

namespace {

class SimulationApp final : public drone::simulator::SimulationBase {
public:
    SimulationApp()
        : quad_(drone::model::Quadrocopter::createWithBatterySim(
              "Quad",
              drone::model::components::ElecMotorSpecs(15000.0, 14.8, 20.0, 0.9, 0.4),
              drone::model::sensors::AnalogIOSpec(
                  drone::model::sensors::AnalogIOSpec::IODirection::OUTPUT,
                  drone::model::sensors::AnalogIOSpec::CurrentRange::ZERO_TO_10V,
                  0, 10000),
              drone::model::components::BatterySpecs(4, drone::model::components::CellSpecs(1500.0, 4.2)),
              drone::model::sensors::AnalogIOSpec(
                  drone::model::sensors::AnalogIOSpec::IODirection::INPUT,
                  drone::model::sensors::AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
                  4000, 20000),
              drone::model::sensors::TemperatureSensorRanges(-50.0, 150.0))),
          elapsed_s_(0.0) {}

protected:
    void onStart() override {
        std::cout << "Simulation started" << std::endl;
    }

    void onStop() override {
        std::cout << "Simulation stopped" << std::endl;
    }

    void onStep(double dt_s) override {
        elapsed_s_ += dt_s;
        const double ramp_time_s = 2.0;
        const double ramp_ratio = std::min(elapsed_s_ / ramp_time_s, 1.0);

        auto& motors = quad_.getMotors();
        const uint64_t delta_ms = static_cast<uint64_t>(dt_s * 1000.0);
        double total_current_a = 0.0;
        for (auto& motor : motors) {
            motor.setDesiredSpeedRPM(motor.getSpecs().max_speed_rpm * ramp_ratio);
            drone::simulator::physics::MotorPhysics::updateMotorPhysics(motor, delta_ms);
            total_current_a += motor.getCurrentA();
        }

        auto* battery = quad_.getBattery();
        if (auto* battery_sim = dynamic_cast<drone::simulator::physics::BatterySim*>(battery)) {
            battery_sim->setCurrentA(total_current_a);
            battery_sim->update(delta_ms);
        }
        const double capacity_mah = battery ? battery->getRemainingCapacityMah() : 0.0;

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "dt=" << dt_s << "s "
                  << "battery_mAh=" << capacity_mah;

        for (size_t i = 0; i < motors.size(); ++i) {
            const auto& motor = motors[i];
            std::cout << " | M" << (i + 1)
                      << " tempC=" << motor.getTemperatureC()
                      << " currentA=" << motor.getCurrentA()
                      << " rpm=" << motor.getSpeedRPM();
        }
        std::cout << std::endl;
    }

private:
    drone::model::Quadrocopter quad_;
    double elapsed_s_;
};

bool parseArgs(int argc, char** argv, uint64_t& steps, double& dt_s) {
    if (argc >= 2) {
        try {
            steps = static_cast<uint64_t>(std::stoull(argv[1]));
        } catch (...) {
            return false;
        }
    }
    if (argc >= 3) {
        try {
            dt_s = std::stod(argv[2]);
        } catch (...) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    uint64_t steps = 10;
    double dt_s = 0.01;

    if (!parseArgs(argc, argv, steps, dt_s)) {
        std::cerr << "Usage: " << argv[0] << " [steps] [dt_s]" << std::endl;
        return 1;
    }

    SimulationApp sim;
    sim.start();
    sim.runForSteps(steps, dt_s);
    sim.stop();

    return 0;
}
