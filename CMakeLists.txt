cmake_minimum_required(VERSION 3.16)
project(virtDrone)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/libs)

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
)

# Common libs
add_library(common
    libs/common/math_utils.cpp
)

# External libs (placeholder)
# add_subdirectory(libs/external)

# Executables or tests can be added here
# add_executable(simulator_test tests/unit/simulator_test.cpp)
# target_link_libraries(simulator_test simulator common)

# For web (Python), no CMake needed