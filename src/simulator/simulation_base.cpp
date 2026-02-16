#include "simulator/simulation_base.h"

namespace drone::simulator {

SimulationBase::SimulationBase()
    : running_(false) {}

void SimulationBase::start() {
    if (running_) {
        return;
    }
    running_ = true;
    onStart();
}

void SimulationBase::stop() {
    if (!running_) {
        return;
    }
    onStop();
    running_ = false;
}

void SimulationBase::step(double delta_time_s) {
    if (delta_time_s <= 0.0) {
        return;
    }
    onStep(delta_time_s);
}

void SimulationBase::runForSteps(uint64_t steps, double delta_time_s) {
    if (delta_time_s <= 0.0) {
        return;
    }
    for (uint64_t i = 0; i < steps; ++i) {
        step(delta_time_s);
    }
}

}  // namespace drone::simulator
