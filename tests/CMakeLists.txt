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

add_executable(test_force_dynamics
    unit/simulator/physics/test_force_dynamics.cpp
)

add_executable(test_gps_sim
    unit/simulator/physics/test_gps_sim.cpp
)

add_executable(test_noisy_sensor_source_gps
    unit/simulator/runtime/test_noisy_sensor_source_gps.cpp
)

add_executable(test_weather_model
    unit/simulator/environment/test_weather_model.cpp
)

add_executable(test_weather_config
    unit/simulator/config/test_weather_config.cpp
)

add_executable(test_quadrosimulator_ground_lock
    unit/simulator/test_quadrosimulator_ground_lock.cpp
)

add_executable(test_simulation_base
    unit/simulator/test_simulation_base.cpp
)

add_executable(test_gps_sensor
    unit/drone/model/component/test_gps_sensor.cpp
)

add_executable(test_energy
    unit/drone/model/test_battery_base.cpp
)

add_executable(test_drone_base
    unit/drone/model/test_drone_base.cpp
)

add_executable(test_esc_logic
    unit/drone/model/component/test_esc_logic.cpp
)

add_executable(test_alt_ctrl_logic
    unit/drone/model/component/test_alt_controller.cpp
)

add_executable(test_real_drone_mixer
    unit/drone/model/component/test_real_drone_mixer.cpp
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

target_link_libraries(test_force_dynamics
    PRIVATE
        Catch2::Catch2WithMain
        drone
        simulator
)

target_link_libraries(test_gps_sim
    PRIVATE
        Catch2::Catch2WithMain
        drone
        simulator
)

target_link_libraries(test_noisy_sensor_source_gps
    PRIVATE
        Catch2::Catch2WithMain
        drone
        simulator
)

target_link_libraries(test_weather_model
    PRIVATE
        Catch2::Catch2WithMain
        drone
        simulator
)

target_link_libraries(test_weather_config
    PRIVATE
        Catch2::Catch2WithMain
        drone
        simulator
    yaml-cpp::yaml-cpp
)

target_link_libraries(test_quadrosimulator_ground_lock
    PRIVATE
        Catch2::Catch2WithMain
        drone
        simulator
        drone_sim
)

target_link_libraries(test_simulation_base
    PRIVATE
        Catch2::Catch2WithMain
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
        simulator  # Link to the simulator library for dependencies
)

target_link_libraries(test_drone_base
    PRIVATE
        Catch2::Catch2WithMain
        drone  # Link to the drone library for dependencies
        simulator  # Link to the simulator library for dependencies
)

target_link_libraries(test_esc_logic
    PRIVATE
        Catch2::Catch2WithMain
 )

 target_link_libraries(test_alt_ctrl_logic
     PRIVATE
        Catch2::Catch2WithMain
 )

target_link_libraries(test_real_drone_mixer
    PRIVATE
        Catch2::Catch2WithMain
        drone
)


# Register the test with CTest
add_test(NAME test_utils COMMAND test_utils)
add_test(NAME test_base_sensor COMMAND test_base_sensor)
add_test(NAME test_temperature_sensor COMMAND test_temperature_sensor)
add_test(NAME test_motors COMMAND test_motors)
add_test(NAME test_motor_physics COMMAND test_motor_physics)
add_test(NAME test_force_dynamics COMMAND test_force_dynamics)
add_test(NAME test_gps_sim COMMAND test_gps_sim)
add_test(NAME test_noisy_sensor_source_gps COMMAND test_noisy_sensor_source_gps)
add_test(NAME test_weather_model COMMAND test_weather_model)
add_test(NAME test_weather_config COMMAND test_weather_config)
add_test(NAME test_quadrosimulator_ground_lock COMMAND test_quadrosimulator_ground_lock)
add_test(NAME test_simulation_base COMMAND test_simulation_base)
add_test(NAME test_gps_sensor COMMAND test_gps_sensor)
add_test(NAME test_energy COMMAND test_energy)
add_test(NAME test_drone_base COMMAND test_drone_base)
add_test(NAME test_esc_logic COMMAND test_esc_logic)
add_test(NAME test_alt_ctrl_logic COMMAND test_alt_ctrl_logic)
add_test(NAME test_real_drone_mixer COMMAND test_real_drone_mixer)
# Enable test discovery for Catch2
include(Catch)
catch_discover_tests(test_utils)
catch_discover_tests(test_base_sensor)
catch_discover_tests(test_temperature_sensor)
catch_discover_tests(test_motors)
catch_discover_tests(test_motor_physics)
catch_discover_tests(test_force_dynamics)
catch_discover_tests(test_gps_sim)
catch_discover_tests(test_noisy_sensor_source_gps)
catch_discover_tests(test_weather_model)
catch_discover_tests(test_weather_config)
catch_discover_tests(test_quadrosimulator_ground_lock)
catch_discover_tests(test_simulation_base)
catch_discover_tests(test_gps_sensor)
catch_discover_tests(test_energy)
catch_discover_tests(test_drone_base)
catch_discover_tests(test_esc_logic)
catch_discover_tests(test_alt_ctrl_logic)
catch_discover_tests(test_real_drone_mixer)