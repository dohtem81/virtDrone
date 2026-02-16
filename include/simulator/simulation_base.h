#ifndef SIMULATION_BASE_H
#define SIMULATION_BASE_H

#include <cstdint>

namespace drone::simulator {

/**
 * @brief Base class for the simulator root.
 *
 * Provides step-based simulation lifecycle. Override onStep() to
 * implement simulation behavior per step.
 */
class SimulationBase {
public:
     SimulationBase();
    virtual ~SimulationBase() = default;

    void start();
    void stop();

    bool isRunning() const { return running_; }

    void step(double delta_time_s);
    void runForSteps(uint64_t steps, double delta_time_s);

protected:
    virtual void onStart() {}
    virtual void onStop() {}
    virtual void onStep(double dt_s) { (void)dt_s; }

private:
    bool running_;
};

}  // namespace drone::simulator

#endif  // SIMULATION_BASE_H
