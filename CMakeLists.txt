cmake_minimum_required(VERSION 3.16)
project(virtDrone)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to build tests
option(VIRTD_BUILD_TESTS "Build unit tests" ON)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/libs)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Simulator library
add_library(simulator
    src/simulator/physics/physics.cpp
    src/simulator/environment/environment.cpp
    src/simulator/integration/integration.cpp
    src/simulator/physics/motor_physics.cpp
    # src/simulator/physics/gps_physics.cpp
)

# Drone library
add_library(drone
    src/drone/model/model.cpp
    src/drone/control/control.cpp
    src/drone/telemetry/telemetry.cpp
    src/drone/model/sensors/base_sensor.cpp
    src/drone/model/sensors/temperature_sensor.cpp  # Added to include TemperatureSensor implementation
    src/drone/model/components/elect_motor.cpp
    # src/drone/model/sensors/gps_position_sensor.cpp
    # src/drone/model/sensors/gps_velocity_sensor.cpp    
    # src/drone/model/components/gps_sensor.cpp
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