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
)

# Drone library
add_library(drone
    src/drone/model/model.cpp
    src/drone/control/control.cpp
    src/drone/telemetry/telemetry.cpp
    src/drone/model/sensors/base_sensor.cpp
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

    # Add test executables
    add_executable(test_base_sensor
        tests/unit/drone/model/sensors/test_base_sensor.cpp
    )
    target_link_libraries(test_base_sensor
        drone
        Catch2::Catch2WithMain
    )

    include(Catch)
    catch_discover_tests(test_base_sensor)
endif()