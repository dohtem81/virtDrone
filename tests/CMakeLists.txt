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

# Register the test with CTest
add_test(NAME test_utils COMMAND test_utils)
add_test(NAME test_base_sensor COMMAND test_base_sensor)
add_test(NAME test_temperature_sensor COMMAND test_temperature_sensor)
add_test(NAME test_motors COMMAND test_motors)
# Enable test discovery for Catch2
include(Catch)
catch_discover_tests(test_utils)
catch_discover_tests(test_base_sensor)
catch_discover_tests(test_temperature_sensor)
catch_discover_tests(test_motors)
