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

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/libs)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Core drone library (without simulator dependencies)
add_library(drone
    src/drone/model/model.cpp
    src/drone/control/control.cpp
    src/drone/telemetry/telemetry.cpp
    src/drone/model/sensors/base_sensor.cpp
    src/drone/model/sensors/temperature_sensor.cpp
    src/drone/model/components/elect_motor.cpp
)

# Simulator library
add_library(simulator
    src/simulator/physics/physics.cpp
    src/simulator/environment/environment.cpp
    src/simulator/integration/integration.cpp
    src/simulator/physics/motor_physics.cpp
    src/simulator/physics/battery_cell_physics.cpp
    src/simulator/physics/battery_sim.cpp
    src/simulator/physics/gps_sim.cpp
    src/simulator/physics/thrust_model.cpp
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
    drone
    simulator
    drone_sim
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