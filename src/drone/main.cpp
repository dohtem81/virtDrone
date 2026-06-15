#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "drone/runtime/control_session.h"
#include "drone/runtime/simulation_gateway_factory.h"

namespace {

bool parseArgs(
    int argc,
    char** argv,
    drone::runtime::ControlBackend& backend,
    std::string& simulator_address,
    uint64_t& steps,
    double& dt_s,
    std::string& altitude_config_file,
    std::string& attitude_config_file,
    std::string& mission_file,
    std::string& logs_dir) {
    int arg_index = 1;
    if (argc >= 2) {
        const std::string first_arg = argv[1];
        if (first_arg == "hardware") {
            backend = drone::runtime::ControlBackend::Hardware;
            ++arg_index;
        } else if (first_arg == "grpc" || first_arg == "sim" || first_arg == "simulation") {
            backend = drone::runtime::ControlBackend::GrpcSimulation;
            ++arg_index;
        }
    }

    if (argc > arg_index) {
        simulator_address = argv[arg_index++];
    }
    if (argc > arg_index) {
        try {
            steps = static_cast<uint64_t>(std::stoull(argv[arg_index++]));
        } catch (...) {
            return false;
        }
    }
    if (argc > arg_index) {
        try {
            dt_s = std::stod(argv[arg_index++]);
        } catch (...) {
            return false;
        }
    }
    if (argc > arg_index) {
        altitude_config_file = argv[arg_index++];
    }
    if (argc > arg_index) {
        attitude_config_file = argv[arg_index++];
    }
    if (argc > arg_index) {
        mission_file = argv[arg_index++];
    }
    if (argc > arg_index) {
        logs_dir = argv[arg_index++];
    }
    return true;
}

std::filesystem::path resolveLogsDir(const std::string& logs_dir_arg) {
    namespace fs = std::filesystem;

    if (!logs_dir_arg.empty()) {
        const fs::path requested_dir = fs::path(logs_dir_arg);
        fs::create_directories(requested_dir);
        return requested_dir;
    }

    const fs::path cwd_tutorial = fs::path("docs") / "tutorials";
    const fs::path parent_tutorial = fs::path("..") / "docs" / "tutorials";

    if (fs::exists(cwd_tutorial) && fs::is_directory(cwd_tutorial)) {
        return cwd_tutorial;
    }
    if (fs::exists(parent_tutorial) && fs::is_directory(parent_tutorial)) {
        return parent_tutorial;
    }

    fs::create_directories(cwd_tutorial);
    return cwd_tutorial;
}

}  // namespace

int main(int argc, char** argv) {
    drone::runtime::ControlBackend backend = drone::runtime::ControlBackend::GrpcSimulation;
    std::string simulator_address = "localhost:50051";
    uint64_t steps = 10;
    double dt_s = 0.01;
    std::string altitude_config_file = "config/altitude_controller.yaml";
    std::string attitude_config_file = "config/attitude_controller.yaml";
    std::string mission_file;
    std::string logs_dir;

    if (!parseArgs(argc, argv, backend, simulator_address, steps, dt_s, altitude_config_file, attitude_config_file, mission_file, logs_dir)) {
        std::cerr << "Usage: " << argv[0] << " [backend] [simulator_address] [steps] [dt_s] [altitude_config_file] [attitude_config_file] [mission_file] [logs_dir]" << std::endl;
        std::cerr << "  backend: grpc | sim | simulation | hardware (default: grpc)" << std::endl;
        return 1;
    }

    const std::filesystem::path output_logs_dir = resolveLogsDir(logs_dir);
    std::unique_ptr<drone::runtime::SimulationGateway> gateway =
        drone::runtime::CreateControlGateway(backend, simulator_address);

    if (!gateway) {
        std::cerr << "Selected backend is not available yet: hardware" << std::endl;
        return 1;
    }

    return drone::runtime::RunControlSession(
        *gateway,
        altitude_config_file,
        attitude_config_file,
        mission_file,
        steps,
        dt_s,
        output_logs_dir);
}
