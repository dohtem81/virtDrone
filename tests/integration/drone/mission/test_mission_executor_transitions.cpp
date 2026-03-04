#include <catch2/catch_test_macros.hpp>

#include <memory>

#include "drone/model/components/altitude_controler.h"
#include "drone/runtime/real_drone.h"
#include "drone/mission/mission_executor.h"

TEST_CASE("MissionExecutor advances through time and completion steps", "[MissionExecutor]") {
    using namespace drone::mission;

    drone::model::components::AltitudeController altitude_controller;
    drone::runtime::RealDrone real_drone(altitude_controller);
    drone::runtime::SensorFrame sensor{};
    sensor.altitude_m = 0.0;
    sensor.position_enu_x_m = 0.0;
    sensor.position_enu_y_m = 0.0;

    Mission mission;
    mission.name = "transition_test";

    MissionStep step1;
    step1.step_id = 1;
    step1.name = "timed_hover";
    auto hover1 = std::make_unique<HoverAction>();
    hover1->target_altitude_m = 5.0;
    step1.action = std::move(hover1);
    step1.advance_mode = AdvanceMode::TIME_BASED;
    step1.duration_s = 0.1;

    MissionStep step2;
    step2.step_id = 2;
    step2.name = "wait_altitude";
    auto hover2 = std::make_unique<HoverAction>();
    hover2->target_altitude_m = 10.0;
    step2.action = std::move(hover2);
    step2.advance_mode = AdvanceMode::COMPLETION_BASED;
    step2.completion_criteria.condition_type = CompletionConditionType::ALTITUDE_REACHED;
    step2.completion_criteria.target_altitude_m = 10.0;
    step2.completion_criteria.altitude_tolerance_m = 0.2;

    mission.steps.emplace_back(std::move(step1));
    mission.steps.emplace_back(std::move(step2));

    MissionExecutor executor;
    executor.loadMission(mission);
    executor.start();
    REQUIRE(executor.getStatus() == MissionStatus::RUNNING);
    REQUIRE(executor.getCurrentStepId() == 1);

    executor.update(real_drone, sensor, 0.05);
    REQUIRE(executor.getCurrentStepId() == 1);
    executor.update(real_drone, sensor, 0.05);
    REQUIRE(executor.getCurrentStepId() == 2);

    sensor.altitude_m = 10.0;
    executor.update(real_drone, sensor, 0.01);
    REQUIRE(executor.getStatus() == MissionStatus::COMPLETED);
}

TEST_CASE("MissionExecutor completes step on position_reached criteria", "[MissionExecutor]") {
    using namespace drone::mission;

    drone::model::components::AltitudeController altitude_controller;
    drone::runtime::RealDrone real_drone(altitude_controller);
    drone::runtime::SensorFrame sensor{};

    Mission mission;
    mission.name = "position_reached_test";

    MissionStep step;
    step.step_id = 10;
    step.name = "go_to_xy";

    auto goto_action = std::make_unique<GoToPositionAction>();
    goto_action->target_position_enu_m = drone::Vector3(10.0, -4.0, 0.0);
    goto_action->target_altitude_m = 12.0;
    goto_action->max_tilt_rad = 0.4;
    goto_action->max_velocity_mps = 4.0;
    step.action = std::move(goto_action);

    step.advance_mode = AdvanceMode::COMPLETION_BASED;
    step.completion_criteria.condition_type = CompletionConditionType::POSITION_REACHED;
    step.completion_criteria.target_position_enu_m = drone::Vector3(10.0, -4.0, 0.0);
    step.completion_criteria.position_tolerance_m = 0.5;
    step.completion_criteria.target_altitude_m = 12.0;
    step.completion_criteria.altitude_tolerance_m = 0.3;
    step.timeout_s = 3.0;

    mission.steps.emplace_back(std::move(step));

    MissionExecutor executor;
    executor.loadMission(mission);
    executor.start();

    REQUIRE(executor.getStatus() == MissionStatus::RUNNING);
    REQUIRE(executor.getCurrentStepId() == 10);

    sensor.position_enu_x_m = 9.1;
    sensor.position_enu_y_m = -4.9;
    sensor.position_enu_z_m = 12.0;
    executor.update(real_drone, sensor, 0.05);
    REQUIRE(executor.getStatus() == MissionStatus::RUNNING);

    sensor.position_enu_x_m = 10.2;
    sensor.position_enu_y_m = -4.1;
    sensor.position_enu_z_m = 12.1;
    executor.update(real_drone, sensor, 0.05);
    REQUIRE(executor.getStatus() == MissionStatus::COMPLETED);
}
