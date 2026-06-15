#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "drone/model/components/altitude_controler.h"
#include "drone/runtime/simulation_gateway_rpc.h"
#include "simulator/runtime/noisy_sensor_source.h"

namespace {

class RecordingGateway final : public drone::runtime::SimulationGateway {
public:
    drone::runtime::SensorFrame readSensors() const override {
        events_.push_back("readSensors");
        return sensors_;
    }

    void applyActuators(const drone::runtime::ActuatorFrame& actuator_frame) override {
        events_.push_back("applyActuators");
        last_actuators_ = actuator_frame;
    }

    void step(double dt_s) override {
        events_.push_back("step");
        last_dt_s_ = dt_s;
    }

    void setSensors(const drone::runtime::SensorFrame& sensors) {
        sensors_ = sensors;
    }

    const std::vector<std::string>& events() const {
        return events_;
    }

    drone::runtime::ActuatorFrame last_actuators_{};
    double last_dt_s_ = 0.0;

private:
    mutable std::vector<std::string> events_;
    drone::runtime::SensorFrame sensors_{};
};

}  // namespace

TEST_CASE("simulation RPC service and stub preserve gateway call order", "[SimulationGateway][RPC][Loop]") {
    drone::model::components::AltitudeController altitude_controller(
        0.0,
        1.0,
        0.0,
        0.0,
        5000.0,
        0.0,
        false,
        false,
        0.5);

    drone::runtime::RealDrone real_drone(altitude_controller);
    RecordingGateway gateway;
    drone::runtime::LocalSimulationGatewayRpcService service(gateway);
    drone::runtime::SimulationGatewayRpcStub stub(service);

    drone::runtime::SensorFrame sensors;
    sensors.altitude_m = 0.0;
    gateway.setSensors(sensors);

    drone::simulator::runtime::NoisySensorSource noisy_source(stub);
    drone::runtime::runSimulationStep(real_drone, noisy_source, stub, stub, 0.01);

    REQUIRE(gateway.events().size() == 3);
    REQUIRE(gateway.events()[0] == "readSensors");
    REQUIRE(gateway.events()[1] == "applyActuators");
    REQUIRE(gateway.events()[2] == "step");
    REQUIRE(gateway.last_dt_s_ == Catch::Approx(0.01));
}