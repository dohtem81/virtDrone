cmake_minimum_required(VERSION 3.16)
project(virtDrone)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GTest (optional for testing)
find_package(GTest)

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

# Testing (only if GTest is found)
if(GTEST_FOUND)
    include_directories(${GTEST_INCLUDE_DIRS})
    enable_testing()

    # Add test executables
    add_executable(test_base_sensor
        tests/unit/drone/model/sensors/test_base_sensor.cpp
    )
    target_link_libraries(test_base_sensor
        drone
        ${GTEST_LIBRARIES}
        gtest_main
        pthread
    )
    add_test(NAME BaseSensorTest COMMAND test_base_sensor)
endif()

# For web (Python), no CMake needed