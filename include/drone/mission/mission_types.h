#ifndef DRONE_MISSION_MISSION_TYPES_H
#define DRONE_MISSION_MISSION_TYPES_H

#include "drone/drone_data_types.h"

#include <memory>
#include <string>
#include <vector>

namespace drone::mission {

enum class ActionType {
    HOVER,
    GO_TO_POSITION,
    LAND,
    SET_ATTITUDE,
    CHANGE_ALTITUDE,
    ROTATE_YAW,
};

enum class AdvanceMode {
    TIME_BASED,
    COMPLETION_BASED,
};

enum class CompletionConditionType {
    TIME_ELAPSED,
    ALTITUDE_REACHED,
    ALTITUDE_AND_VELOCITY,
    POSITION_REACHED,
    YAW_REACHED,
    ATTITUDE_REACHED,
    VELOCITY_LOW,
    LANDED,
};

enum class TimeoutBehavior {
    ABORT,
    PROCEED,
    RETRY,
};

struct CompletionCriteria {
    CompletionConditionType condition_type = CompletionConditionType::TIME_ELAPSED;
    Vector3 target_position_enu_m{0.0, 0.0, 0.0};
    double position_tolerance_m = 2.0;
    double target_altitude_m = 0.0;
    double altitude_tolerance_m = 1.0;
    double target_yaw_rad = 0.0;
    double yaw_tolerance_rad = 0.1;
    double target_pitch_rad = 0.0;
    double target_roll_rad = 0.0;
    double pitch_tolerance_rad = 0.1;
    double roll_tolerance_rad = 0.1;
    double max_velocity_mps = 0.1;
    double velocity_tolerance_mps = 0.1;
    double duration_s = 0.0;
    double hold_duration_s = 0.0;
    double timeout_s = 60.0;
};

class MissionAction {
public:
    virtual ~MissionAction() = default;
    virtual ActionType getActionType() const = 0;
    virtual std::string getDescription() const = 0;
};

class HoverAction : public MissionAction {
public:
    ActionType getActionType() const override { return ActionType::HOVER; }
    std::string getDescription() const override { return "Hover at " + std::to_string(target_altitude_m) + "m"; }

    double target_altitude_m = 0.0;
    double yaw_rad = 0.0;
};

class GoToPositionAction : public MissionAction {
public:
    ActionType getActionType() const override { return ActionType::GO_TO_POSITION; }
    std::string getDescription() const override {
        return "Fly to (" + std::to_string(target_position_enu_m.x) + ", " +
               std::to_string(target_position_enu_m.y) + ") at " +
               std::to_string(target_altitude_m) + "m";
    }

    Vector3 target_position_enu_m{0.0, 0.0, 0.0};
    double target_altitude_m = 0.0;
    std::string altitude_mode = "absolute";
    double max_tilt_rad = 0.5;
    double max_velocity_mps = 10.0;
};

class LandAction : public MissionAction {
public:
    ActionType getActionType() const override { return ActionType::LAND; }
    std::string getDescription() const override { return "Land"; }

    double descent_rate_mps = 1.0;
};

class SetAttitudeAction : public MissionAction {
public:
    ActionType getActionType() const override { return ActionType::SET_ATTITUDE; }
    std::string getDescription() const override { return "Set attitude"; }

    double target_pitch_rad = 0.0;
    double target_roll_rad = 0.0;
    double target_yaw_rad = 0.0;
};

class ChangeAltitudeAction : public MissionAction {
public:
    ActionType getActionType() const override { return ActionType::CHANGE_ALTITUDE; }
    std::string getDescription() const override { return "Change altitude to " + std::to_string(target_altitude_m) + "m"; }

    double target_altitude_m = 0.0;
    double rate_mps = 2.0;
};

class RotateYawAction : public MissionAction {
public:
    ActionType getActionType() const override { return ActionType::ROTATE_YAW; }
    std::string getDescription() const override { return "Rotate yaw"; }

    double target_yaw_rad = 0.0;
};

struct MissionStep {
    int step_id = 0;
    std::string name = "Unnamed Step";
    std::unique_ptr<MissionAction> action;

    AdvanceMode advance_mode = AdvanceMode::TIME_BASED;
    double duration_s = 0.0;

    CompletionCriteria completion_criteria;
    TimeoutBehavior timeout_behavior = TimeoutBehavior::ABORT;
    double timeout_s = 60.0;
    int fallback_step_id = -1;
    int retry_count = 0;

    bool enabled = true;
};

struct InitialConditions {
    double altitude_m = 0.0;
    Vector3 position_enu_m{0.0, 0.0, 0.0};
    double yaw_rad = 0.0;
    double battery_soc_percent = 100.0;
};

struct Mission {
    std::string name = "Untitled Mission";
    std::string description;
    std::string version = "1.0";

    std::string author;
    std::string created;
    double max_duration_s = 0.0;

    InitialConditions initial_conditions;
    std::vector<MissionStep> steps;
};

}  // namespace drone::mission

#endif  // DRONE_MISSION_MISSION_TYPES_H
