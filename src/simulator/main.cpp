#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "drone/model/quadrocopter.h"
#include "drone/runtime/simulation_gateway_grpc.h"
#include "grpcpp/grpcpp.h"
#include "simulator/config/weather_config.h"
#include "simulator/quadrosimulator.h"

namespace {

std::string localTimestampNow() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif
    std::ostringstream out;
    out << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
    return out.str();
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

void logEvent(std::ofstream& events_log, double sim_elapsed_s, const std::string& message) {
    if (!events_log.is_open()) {
        return;
    }
    events_log << std::fixed << std::setprecision(6)
               << localTimestampNow() << ","
               << sim_elapsed_s << ","
               << message << "\n";
}

bool parseArgs(int argc, char** argv, std::string& weather_config_file, std::string& logs_dir, std::string& listen_address) {
    if (argc >= 2) {
        weather_config_file = argv[1];
    }
    if (argc >= 3) {
        logs_dir = argv[2];
    }
    if (argc >= 4) {
        listen_address = argv[3];
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::string weather_config_file = "config/weather.yaml";
    std::string logs_dir;
    std::string listen_address = "0.0.0.0:50051";
    double sim_elapsed_s = 0.0;

    if (!parseArgs(argc, argv, weather_config_file, logs_dir, listen_address)) {
        std::cerr << "Usage: " << argv[0] << " [weather_config_file] [logs_dir] [listen_address]" << std::endl;
        std::cerr << "  weather_config_file: YAML config file path (default: config/weather.yaml)" << std::endl;
        std::cerr << "  logs_dir: output directory for simulation_telemetry.csv and simulation_events.log (optional, default: docs/tutorials)" << std::endl;
        std::cerr << "  listen_address: gRPC listen address (default: 0.0.0.0:50051)" << std::endl;
        return 1;
    }

    const std::filesystem::path output_logs_dir = resolveLogsDir(logs_dir);
    const std::string telemetry_log_file = (output_logs_dir / "simulation_telemetry.csv").string();
    const std::string events_log_file = (output_logs_dir / "simulation_events.log").string();

    std::ofstream events_log(events_log_file, std::ios::out | std::ios::trunc);
    if (!events_log.is_open()) {
        std::cerr << "Failed to open events log file: " << events_log_file << std::endl;
        return 1;
    }

    logEvent(events_log, sim_elapsed_s,
             "SIMULATION_START weather_config='" + weather_config_file +
             "' logs_dir='" + output_logs_dir.string() +
             "' telemetry_csv='" + telemetry_log_file +
             "' listen_address='" + listen_address + "'");

    drone::simulator::config::WeatherConfig weather_config;
    if (!weather_config.loadFromFile(weather_config_file)) {
        logEvent(events_log, sim_elapsed_s, "WARN weather config load failed: '" + weather_config_file + "' using defaults");
    } else {
        logEvent(events_log, sim_elapsed_s, "Loaded weather config: '" + weather_config_file + "'");
    }

    drone::model::components::ElecMotorSpecs motor_specs(15000.0, 14.8, 20.0, 0.9, 0.4, 0.12);
    drone::model::sensors::AnalogIOSpec motor_io_spec(
        drone::model::sensors::AnalogIOSpec::IODirection::OUTPUT,
        drone::model::sensors::AnalogIOSpec::CurrentRange::ZERO_TO_10V,
        0,
        10000);
    drone::model::components::CellSpecs cell_specs(1500.0, 4.2);
    drone::model::components::BatterySpecs battery_specs(4, cell_specs, 0.35);
    drone::model::sensors::AnalogIOSpec temp_io_spec(
        drone::model::sensors::AnalogIOSpec::IODirection::INPUT,
        drone::model::sensors::AnalogIOSpec::CurrentRange::FOUR_TO_20mA,
        4000,
        20000);
    drone::model::sensors::TemperatureSensorRanges temp_ranges(-50.0, 150.0);
    drone::model::components::GPSSensorSpecs gps_specs;

    auto sim = drone::simulator::QuadroSimulationFactory(
        "QuadTest",
        motor_specs,
        motor_io_spec,
        battery_specs,
        temp_io_spec,
        temp_ranges,
        0.02,
        gps_specs,
        1.2,
        0.3,
        1.0,
        1,
        0.01);

    sim->setWeatherConfig(weather_config);
    if (!sim->setTelemetryLogFile(telemetry_log_file)) {
        logEvent(events_log, sim_elapsed_s, "ERROR failed to open telemetry csv: '" + telemetry_log_file + "'");
        return 1;
    }

    sim->start();
    logEvent(events_log, sim_elapsed_s, "Simulation runtime started");

    drone::runtime::GrpcSimulationGatewayService grpc_service(*sim);
    ::grpc::ServerBuilder builder;
    builder.AddListeningPort(listen_address, ::grpc::InsecureServerCredentials());
    builder.RegisterService(&grpc_service);
    std::unique_ptr<::grpc::Server> server = builder.BuildAndStart();
    if (!server) {
        logEvent(events_log, sim_elapsed_s, "ERROR failed to start gRPC server at '" + listen_address + "'");
        sim->stop();
        return 1;
    }

    logEvent(events_log, sim_elapsed_s, "gRPC server listening at '" + listen_address + "'");
    server->Wait();

    sim->stop();
    logEvent(events_log, sim_elapsed_s, "SIMULATION_STOP");
    events_log.close();
    return 0;
}
