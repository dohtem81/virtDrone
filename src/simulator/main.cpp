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
