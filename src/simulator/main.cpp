#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

#include "drone/model/quadrocopter.h"
#include "simulator/physics/battery_sim.h"
#include "simulator/physics/gps_sim.h"
#include "simulator/physics/motor_physics.h"
#include "simulator/simulation_base.h"

namespace {

class SimulationApp final : public drone::simulator::SimulationBase {
public:
    SimulationApp()
        : quad_(drone::model::Quadrocopter::createWithBatterySim(
              "Quad",
              drone::model::components::ElecMotorSpecs(15000.0, 14.8, 20.0, 0.9, 0.4, 0.12),
              drone::model::sensors::AnalogIOSpec(
                  drone::model::sensors::AnalogIOSpec::IODirection::OUTPUT,
                  drone::model::sensors::AnalogIOSpec::CurrentRange::ZERO_TO_10V,
                  0, 10000),
              drone::model::components::BatterySpecs(4, drone::model::components::CellSpecs(1500.0, 4.2), 0.35),
              drone::model::sensors::AnalogIOSpec(
                  drone::model::sensors::AnalogIOSpec::IODirection::INPUT,
                  drone::model::sensors::AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
                  4000, 20000),
              drone::model::sensors::TemperatureSensorRanges(-50.0, 150.0),
              0.02,
              drone::model::components::GPSSensorSpecs(),
              0.2,
              0.127, // blade diameter in meters
              1.0   // blade shape coefficient
              )),
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
        double avg_rpm = 0.0;
        double currentBattVoltageV = quad_.getBattery() ? quad_.getBattery()->getVoltageV() : 0.0;

        for (auto& motor : motors) {
            motor.setDesiredSpeedRPM(motor.getSpecs().max_speed_rpm);
            drone::simulator::physics::MotorPhysics::updateMotorPhysics(motor, delta_ms, quad_.getBattery());
            total_current_a += motor.getCurrentA();
            avg_rpm += motor.getSpeedRPM();
        }

        if (!motors.empty()) {
            avg_rpm /= static_cast<double>(motors.size());
        }

        // for each motor calculate vector of lifting force based on blade diameter, shape coefficient, and RPM
        // then sum the vertical components of the forces to get total lift
        // for simplicity, assume lift is proportional to RPM^2, blade area, and shape coefficient
        // each motor should produce 18N
        double rho = 1.225;  // air density (kg/m^3)

        double total_lift = 0.0;
        for (const auto& motor : motors) {
            double D = motor.getSpecs().blade_diameter_m;
            double n = motor.getSpeedRPM() / 60.0;  // rev/s
            double omega = 2.0 * M_PI * n;  // rad/s

            double A = M_PI * D * D / 4.0;  // disk area (m²)
            double Ct = 0.2 * motor.getSpecs().blade_shape_coeff;
            double thrust = 2.0 * rho * A * Ct * omega * omega * D * D / 4.0;

            total_lift += thrust;
        }


        // now calculate net force and update altitude
        double mass_kg = quad_.getTotalWeightKg();
        double gravity_mps2 = 9.81;

        double kv = 5.0;  // Vertical drag coefficient, adjustable
        double weight_n = mass_kg * gravity_mps2;
        double vertical_drag = -kv * vertical_speed_mps_;
        double net_force_n = total_lift - weight_n + vertical_drag;

        double acceleration_mps2 = net_force_n / mass_kg;
        // update vertical speed and altitude
        vertical_speed_mps_ += acceleration_mps2 * dt_s;
        altitude_m_ += vertical_speed_mps_ * dt_s;
        if (altitude_m_ < 0.0) {
            altitude_m_ = 0.0;
            vertical_speed_mps_ = 0.0;
        }   

        if (auto* gps_sim = dynamic_cast<drone::simulator::physics::GPSSim*>(quad_.getGPS())) {
            gps_sim->setAltitudeM(altitude_m_);
        }

        auto* battery = quad_.getBattery();
        if (auto* battery_sim = dynamic_cast<drone::simulator::physics::BatterySim*>(battery)) {
            battery_sim->setCurrentA(total_current_a);
            battery_sim->update(delta_ms);
        }

        const double capacity_mah = battery ? battery->getRemainingCapacityMah() : 0.0;

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "t=" << elapsed_s_ << "s "
                  << "alt_m=" << altitude_m_
                  << " dt=" << dt_s << "s "
                  << " b%=" << (battery ? battery->getStateOfChargePercent() : 0.0)
                  << " bV=" << currentBattVoltageV;

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
    double altitude_m_{0.0};
    double vertical_speed_mps_{0.0};
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
