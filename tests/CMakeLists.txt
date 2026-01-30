# Catch2 is already fetched in the main CMakeLists.txt, so no need to refetch

# Add the test executable
add_executable(test_base_sensor
    unit/drone/model/sensors/test_base_sensor.cpp
)

add_executable(test_temperature_sensor
    unit/drone/model/sensors/test_temperature_sensor.cpp
)

add_executable(test_utils
    unit/drone/model/test_utils.cpp
)

add_executable(test_motors
    unit/drone/model/component/test_elect_motor.cpp
)

add_executable(test_motor_physics
    unit/simulator/physics/test_motor_physics.cpp
)

add_executable(test_gps_sensor
    unit/drone/model/component/test_gps_sensor.cpp
)

add_executable(test_energy
    unit/drone/model/test_battery_base.cpp
)

# Link against Catch2 and the drone library
target_link_libraries(test_base_sensor
    PRIVATE
        Catch2::Catch2WithMain
        drone  # Link to the drone library for dependencies
)

# Link against Catch2 and the drone library
target_link_libraries(test_temperature_sensor
    PRIVATE
        Catch2::Catch2WithMain
        drone  # Link to the drone library for dependencies
)

target_link_libraries(test_utils
    PRIVATE
        Catch2::Catch2WithMain
        drone  # Link to the drone library for dependencies
)

target_link_libraries(test_motors
    PRIVATE
        Catch2::Catch2WithMain
        drone  # Link to the drone library for dependencies
)

target_link_libraries(test_motor_physics
    PRIVATE
        Catch2::Catch2WithMain
        drone  # Link to the drone library for dependencies
        simulator  # Link to the simulator library for dependencies
)

target_link_libraries(test_gps_sensor   
    PRIVATE
        Catch2::Catch2WithMain
        drone  # Link to the drone library for dependencies
)

target_link_libraries(test_energy   
    PRIVATE
        Catch2::Catch2WithMain
        drone  # Link to the drone library for dependencies
)

# Register the test with CTest
add_test(NAME test_utils COMMAND test_utils)
add_test(NAME test_base_sensor COMMAND test_base_sensor)
add_test(NAME test_temperature_sensor COMMAND test_temperature_sensor)
add_test(NAME test_motors COMMAND test_motors)
add_test(NAME test_motor_physics COMMAND test_motor_physics)
add_test(NAME test_gps_sensor COMMAND test_gps_sensor)
add_test(NAME test_energy COMMAND test_energy)

# Enable test discovery for Catch2
include(Catch)
catch_discover_tests(test_utils)
catch_discover_tests(test_base_sensor)
catch_discover_tests(test_temperature_sensor)
catch_discover_tests(test_motors)
catch_discover_tests(test_motor_physics)
catch_discover_tests(test_gps_sensor)
catch_discover_tests(test_energy)