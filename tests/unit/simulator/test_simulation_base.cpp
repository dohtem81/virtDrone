#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "simulator/simulation_base.h"

using namespace drone::simulator;
using Catch::Approx;

namespace {

class TestSimulation final : public SimulationBase {
public:
    int start_calls = 0;
    int stop_calls = 0;
    int step_calls = 0;
    double last_dt = 0.0;

protected:
    void onStart() override { ++start_calls; }
    void onStop() override { ++stop_calls; }
    void onStep(double dt_s) override {
        ++step_calls;
        last_dt = dt_s;
    }
};

}  // namespace

TEST_CASE("SimulationBase start/stop are idempotent", "[SimulationBase]") {
    TestSimulation sim;

    REQUIRE_FALSE(sim.isRunning());

    sim.start();
    REQUIRE(sim.isRunning());
    REQUIRE(sim.start_calls == 1);

    sim.start();
    REQUIRE(sim.start_calls == 1);

    sim.stop();
    REQUIRE_FALSE(sim.isRunning());
    REQUIRE(sim.stop_calls == 1);

    sim.stop();
    REQUIRE(sim.stop_calls == 1);
}

TEST_CASE("SimulationBase step ignores non-positive dt", "[SimulationBase]") {
    TestSimulation sim;

    sim.step(0.0);
    sim.step(-0.1);
    REQUIRE(sim.step_calls == 0);

    sim.step(0.05);
    REQUIRE(sim.step_calls == 1);
    REQUIRE(sim.last_dt == Approx(0.05));
}

TEST_CASE("SimulationBase runForSteps advances steps", "[SimulationBase]") {
    TestSimulation sim;

    sim.runForSteps(0, 0.01);
    REQUIRE(sim.step_calls == 0);

    sim.runForSteps(3, 0.02);
    REQUIRE(sim.step_calls == 3);
    REQUIRE(sim.last_dt == Approx(0.02));
}
