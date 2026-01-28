#include "simulator/physics/gps_physics.h"
#include <random>  // For adding noise

namespace drone::simulator::physics {

void GPSPhysics::updateGPSPhysics(drone::model::components::GPSSensor& sensor, const drone::Position3D& new_pos, const drone::Velocity3D& new_vel, uint64_t dt_us) {
    // Simulate with noise based on accuracy
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> pos_noise(0, sensor.getPositionSensor().getAccuracy());
    std::normal_distribution<> vel_noise(0, sensor.getVelocitySensor().getAccuracy());

    drone::Position3D noisy_pos = {new_pos.latitude_deg, new_pos.longitude_deg, new_pos.altitude_m};
    drone::Velocity3D noisy_vel = {new_vel.north_mps, new_vel.east_mps, new_vel.down_mps};

    sensor.setPosition(noisy_pos);  // Private setter, accessible via friendship
    sensor.setVelocity(noisy_vel);
    sensor.setSatelliteCount(8);  // Example
    sensor.setStatus(drone::model::sensors::SensorStatus::ACTIVE);
}

}  // namespace drone::simulator::physics