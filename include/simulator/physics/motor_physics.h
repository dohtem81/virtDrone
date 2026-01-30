#ifndef MOTOR_PHYSICS_H
#define MOTOR_PHYSICS_H

#include <cstdint>
#include "drone/model/components/elect_motor.h"

namespace drone::simulator::physics {

class MotorPhysics {
public:
    // Update speed dynamics for the motor
    static void updateSpeed(drone::model::components::ElecMotor& motor, uint64_t delta_ms);

    // Calculate current based on desired speed
    static void calculateCurrent(drone::model::components::ElecMotor& motor);

    // Calculate power losses
    static void calculateLosses(drone::model::components::ElecMotor& motor);

    // Update temperature dynamics
    static void updateTemperature(drone::model::components::ElecMotor& motor, uint64_t delta_ms);

    // Calculate battery drain over time
    static double calculateBatteryDrain(const drone::model::components::ElecMotor& motor, double time_s);
    // Update all motor physics
    static void updateMotorPhysics(drone::model::components::ElecMotor& motor, uint64_t delta_ms);
};

}  // namespace drone::simulator::physics

#endif // MOTOR_PHYSICS_H