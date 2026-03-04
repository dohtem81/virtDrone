#include "drone/mission/mission_loader.h"

#include <memory>

#include <yaml-cpp/yaml.h>

namespace drone::mission {
namespace {

template <typename T>
void readIfPresent(const YAML::Node& node, const char* key, T& value) {
    if (node[key]) {
        value = node[key].as<T>();
    }
}

void readVector2EnuIfPresent(const YAML::Node& node, const char* key, drone::Vector3& vector_out) {
    if (!node[key]) {
        return;
    }
    const auto vector_node = node[key];
    if (!vector_node.IsMap()) {
        return;
    }
    readIfPresent(vector_node, "x", vector_out.x);
    readIfPresent(vector_node, "y", vector_out.y);
}

AdvanceMode parseAdvanceMode(const std::string& mode) {
    if (mode == "completion_based") {
        return AdvanceMode::COMPLETION_BASED;
    }
    return AdvanceMode::TIME_BASED;
}

TimeoutBehavior parseTimeoutBehavior(const std::string& behavior) {
    if (behavior == "proceed") {
        return TimeoutBehavior::PROCEED;
    }
    if (behavior == "retry") {
        return TimeoutBehavior::RETRY;
    }
    return TimeoutBehavior::ABORT;
}

CompletionConditionType parseCompletionCondition(const std::string& condition_type) {
    if (condition_type == "altitude_reached") {
        return CompletionConditionType::ALTITUDE_REACHED;
    }
    if (condition_type == "altitude_and_velocity") {
        return CompletionConditionType::ALTITUDE_AND_VELOCITY;
    }
    if (condition_type == "position_reached") {
        return CompletionConditionType::POSITION_REACHED;
    }
    if (condition_type == "yaw_reached") {
        return CompletionConditionType::YAW_REACHED;
    }
    if (condition_type == "attitude_reached") {
        return CompletionConditionType::ATTITUDE_REACHED;
    }
    if (condition_type == "velocity_low") {
        return CompletionConditionType::VELOCITY_LOW;
    }
    if (condition_type == "landed") {
        return CompletionConditionType::LANDED;
    }
    return CompletionConditionType::TIME_ELAPSED;
}

std::unique_ptr<MissionAction> parseAction(const YAML::Node& step_node) {
    if (!step_node["action"]) {
        return nullptr;
    }

    const std::string action = step_node["action"].as<std::string>();
    if (action == "hover") {
        auto hover = std::make_unique<HoverAction>();
        readIfPresent(step_node, "target_altitude_m", hover->target_altitude_m);
        readIfPresent(step_node, "yaw_rad", hover->yaw_rad);
        return hover;
    }
    if (action == "go_to_position") {
        auto go_to = std::make_unique<GoToPositionAction>();
        readVector2EnuIfPresent(step_node, "target_position_enu_m", go_to->target_position_enu_m);
        readIfPresent(step_node, "target_altitude_m", go_to->target_altitude_m);
        readIfPresent(step_node, "altitude_mode", go_to->altitude_mode);
        readIfPresent(step_node, "max_tilt_rad", go_to->max_tilt_rad);
        readIfPresent(step_node, "max_velocity_mps", go_to->max_velocity_mps);
        return go_to;
    }
    if (action == "land") {
        auto land = std::make_unique<LandAction>();
        readIfPresent(step_node, "descent_rate_mps", land->descent_rate_mps);
        return land;
    }
    if (action == "set_attitude") {
        auto set_attitude = std::make_unique<SetAttitudeAction>();
        readIfPresent(step_node, "target_pitch_rad", set_attitude->target_pitch_rad);
        readIfPresent(step_node, "target_roll_rad", set_attitude->target_roll_rad);
        readIfPresent(step_node, "target_yaw_rad", set_attitude->target_yaw_rad);
        return set_attitude;
    }
    if (action == "change_altitude") {
        auto change_altitude = std::make_unique<ChangeAltitudeAction>();
        readIfPresent(step_node, "target_altitude_m", change_altitude->target_altitude_m);
        readIfPresent(step_node, "rate_mps", change_altitude->rate_mps);
        return change_altitude;
    }
    if (action == "rotate_yaw") {
        auto rotate_yaw = std::make_unique<RotateYawAction>();
        readIfPresent(step_node, "target_yaw_rad", rotate_yaw->target_yaw_rad);
        return rotate_yaw;
    }
    return nullptr;
}

void parseCompletionCriteria(const YAML::Node& step_node, CompletionCriteria& criteria_out) {
    if (!step_node["completion_criteria"]) {
        return;
    }
    const auto criteria_node = step_node["completion_criteria"];

    if (criteria_node["condition_type"]) {
        criteria_out.condition_type = parseCompletionCondition(criteria_node["condition_type"].as<std::string>());
    }

    readVector2EnuIfPresent(criteria_node, "target_position_enu_m", criteria_out.target_position_enu_m);
    readIfPresent(criteria_node, "position_tolerance_m", criteria_out.position_tolerance_m);

    readIfPresent(criteria_node, "target_altitude_m", criteria_out.target_altitude_m);
    readIfPresent(criteria_node, "altitude_tolerance_m", criteria_out.altitude_tolerance_m);

    readIfPresent(criteria_node, "target_yaw_rad", criteria_out.target_yaw_rad);
    readIfPresent(criteria_node, "yaw_tolerance_rad", criteria_out.yaw_tolerance_rad);

    readIfPresent(criteria_node, "target_pitch_rad", criteria_out.target_pitch_rad);
    readIfPresent(criteria_node, "target_roll_rad", criteria_out.target_roll_rad);
    readIfPresent(criteria_node, "pitch_tolerance_rad", criteria_out.pitch_tolerance_rad);
    readIfPresent(criteria_node, "roll_tolerance_rad", criteria_out.roll_tolerance_rad);

    readIfPresent(criteria_node, "max_velocity_mps", criteria_out.max_velocity_mps);
    readIfPresent(criteria_node, "velocity_tolerance_mps", criteria_out.velocity_tolerance_mps);

    readIfPresent(criteria_node, "duration_s", criteria_out.duration_s);
    readIfPresent(criteria_node, "hold_duration_s", criteria_out.hold_duration_s);
    readIfPresent(criteria_node, "timeout_s", criteria_out.timeout_s);
}

}  // namespace

bool MissionLoader::loadFromFile(const std::string& file_path, Mission& mission_out, std::string* error_out) const {
    try {
        const YAML::Node root = YAML::LoadFile(file_path);
        if (!root["mission"]) {
            if (error_out) {
                *error_out = "Missing top-level 'mission' node";
            }
            return false;
        }

        const YAML::Node mission_node = root["mission"];
        Mission mission;
        readIfPresent(mission_node, "name", mission.name);
        readIfPresent(mission_node, "description", mission.description);
        readIfPresent(mission_node, "version", mission.version);

        if (mission_node["metadata"]) {
            const auto metadata = mission_node["metadata"];
            readIfPresent(metadata, "author", mission.author);
            readIfPresent(metadata, "created", mission.created);
            readIfPresent(metadata, "max_duration_s", mission.max_duration_s);
        }

        if (mission_node["initial_conditions"]) {
            const auto initial = mission_node["initial_conditions"];
            readIfPresent(initial, "altitude_m", mission.initial_conditions.altitude_m);
            readVector2EnuIfPresent(initial, "position_enu_m", mission.initial_conditions.position_enu_m);
            readIfPresent(initial, "yaw_rad", mission.initial_conditions.yaw_rad);
            readIfPresent(initial, "battery_soc_percent", mission.initial_conditions.battery_soc_percent);
        }

        if (!mission_node["steps"] || !mission_node["steps"].IsSequence()) {
            if (error_out) {
                *error_out = "'mission.steps' must be a sequence";
            }
            return false;
        }

        for (const auto& step_node : mission_node["steps"]) {
            MissionStep step;

            readIfPresent(step_node, "step_id", step.step_id);
            readIfPresent(step_node, "name", step.name);
            readIfPresent(step_node, "enabled", step.enabled);

            step.action = parseAction(step_node);
            if (!step.action) {
                if (error_out) {
                    *error_out = "Invalid or missing action for step_id=" + std::to_string(step.step_id);
                }
                return false;
            }

            std::string advance_mode = "time_based";
            readIfPresent(step_node, "advance_mode", advance_mode);
            step.advance_mode = parseAdvanceMode(advance_mode);

            readIfPresent(step_node, "duration_s", step.duration_s);
            readIfPresent(step_node, "timeout_s", step.timeout_s);
            readIfPresent(step_node, "fallback_step_id", step.fallback_step_id);
            readIfPresent(step_node, "retry_count", step.retry_count);

            std::string timeout_behavior = "abort";
            readIfPresent(step_node, "on_timeout", timeout_behavior);
            step.timeout_behavior = parseTimeoutBehavior(timeout_behavior);

            parseCompletionCriteria(step_node, step.completion_criteria);
            mission.steps.push_back(std::move(step));
        }

        if (mission.steps.empty()) {
            if (error_out) {
                *error_out = "Mission has no steps";
            }
            return false;
        }

        mission_out = std::move(mission);
        return true;
    } catch (const YAML::Exception& ex) {
        if (error_out) {
            *error_out = ex.what();
        }
        return false;
    }
}

}  // namespace drone::mission
