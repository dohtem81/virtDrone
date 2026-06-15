cmake_minimum_required(VERSION 3.16)
project(virtDrone)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to build tests
option(VIRTD_BUILD_TESTS "Build unit tests" ON)

# Find yaml-cpp
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0
    )
    FetchContent_MakeAvailable(yaml-cpp)
endif()

# Transport layer support for split control/simulator deployment
include(FetchContent)
FetchContent_Declare(
    protobuf
    GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
    GIT_TAG v27.2
)
FetchContent_Declare(
    grpc
    GIT_REPOSITORY https://github.com/grpc/grpc.git
    GIT_TAG v1.65.1
)
FetchContent_MakeAvailable(protobuf grpc)

set(proto_dir ${CMAKE_SOURCE_DIR}/proto)
set(generated_proto_dir ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${generated_proto_dir})

set(simulation_gateway_proto ${proto_dir}/simulation_gateway.proto)
set(simulation_gateway_proto_srcs ${generated_proto_dir}/simulation_gateway.pb.cc)
set(simulation_gateway_proto_hdrs ${generated_proto_dir}/simulation_gateway.pb.h)
set(simulation_gateway_grpc_srcs ${generated_proto_dir}/simulation_gateway.grpc.pb.cc)
set(simulation_gateway_grpc_hdrs ${generated_proto_dir}/simulation_gateway.grpc.pb.h)

add_custom_command(
    OUTPUT ${simulation_gateway_proto_srcs} ${simulation_gateway_proto_hdrs} ${simulation_gateway_grpc_srcs} ${simulation_gateway_grpc_hdrs}
    COMMAND protobuf::protoc
    ARGS --cpp_out=${generated_proto_dir} --grpc_out=${generated_proto_dir} --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin> -I ${proto_dir} ${simulation_gateway_proto}
    DEPENDS ${simulation_gateway_proto}
)

add_library(simulation_gateway_proto
    ${simulation_gateway_proto_srcs}
    ${simulation_gateway_grpc_srcs}
)

target_include_directories(simulation_gateway_proto
    PUBLIC ${generated_proto_dir}
)

target_link_libraries(simulation_gateway_proto
    PUBLIC protobuf::libprotobuf gRPC::grpc++
)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/libs)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Core drone library (without simulator dependencies)
add_library(drone
    src/drone/model/model.cpp
    src/drone/control/control.cpp
    src/drone/control/position_controller.cpp
    src/drone/mission/mission_executor.cpp
    src/drone/mission/completion_evaluator.cpp
    src/drone/mission/mission_loader.cpp
    src/drone/telemetry/telemetry.cpp
    src/drone/model/sensors/base_sensor.cpp
    src/drone/model/sensors/temperature_sensor.cpp
    src/drone/model/components/elect_motor.cpp
    src/drone/config/attitude_controller_config.cpp
)

target_link_libraries(drone
    PUBLIC
    yaml-cpp::yaml-cpp
)

add_library(drone_transport
    src/drone/runtime/simulation_gateway_grpc.cpp
    src/drone/runtime/control_session.cpp
)

target_include_directories(drone_transport
    PUBLIC ${generated_proto_dir}
)

target_link_libraries(drone_transport
    PUBLIC
    drone
    simulation_gateway_proto
    gRPC::grpc++
)

# Simulator library
add_library(simulator
    src/simulator/physics/physics.cpp
    src/simulator/environment/environment.cpp
    src/simulator/environment/weather_model.cpp
    src/simulator/integration/integration.cpp
    src/simulator/physics/motor_physics.cpp
    src/simulator/physics/battery_cell_physics.cpp
    src/simulator/physics/battery_sim.cpp
    src/simulator/physics/gps_sim.cpp
    src/simulator/physics/thrust_model.cpp
    src/simulator/physics/force_dynamics.cpp
    src/simulator/simulation_base.cpp
    src/simulator/quadrosimulator.cpp
)

# Link simulator to drone
target_link_libraries(simulator
    PRIVATE
    drone
)

# Drone-Simulator integration (depends on both drone and simulator)
add_library(drone_sim
    src/drone/model/quadrocopter.cpp
)

target_link_libraries(drone_sim
    PRIVATE
    drone
    simulator
)

# Simulator executable
add_executable(simulator_app
    src/simulator/main.cpp
)
target_link_libraries(simulator_app
    PRIVATE
    simulator
    drone_sim
    drone
    drone_transport
    yaml-cpp::yaml-cpp
)

# Control-side executable for split-process runs
add_executable(drone_app
    src/drone/main.cpp
)

target_link_libraries(drone_app
    PRIVATE
    drone
    drone_transport
    yaml-cpp::yaml-cpp
)

# Common libs
add_library(common
    libs/common/math_utils.cpp
)

# Testing with Catch2 (if enabled)
if(VIRTD_BUILD_TESTS)
    include(FetchContent)

    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.0
    )
    FetchContent_MakeAvailable(Catch2)

    enable_testing()

    # Tests are now handled in the tests/ subdirectory
endif()

# Add the tests subdirectory
add_subdirectory(tests)