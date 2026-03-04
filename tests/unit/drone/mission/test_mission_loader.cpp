#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <string>

#include "drone/mission/mission_loader.h"

namespace {

std::filesystem::path writeTestYaml(const std::string& filename, const std::string& content) {
    const auto path = std::filesystem::current_path() / filename;
    std::ofstream out(path.string(), std::ios::trunc);
    out << content;
    out.close();
    return path;
}

}  // namespace

TEST_CASE("MissionLoader parses valid mission yaml", "[MissionLoader]") {
    const auto yaml_path = writeTestYaml(
        "mission_loader_valid.yaml",
        "mission:\n"
        "  name: 'Loader Test'\n"
        "  version: '1.0'\n"
        "  initial_conditions:\n"
        "    altitude_m: 0.0\n"
        "    position_enu_m:\n"
        "      x: 0.0\n"
        "      y: 0.0\n"
        "  steps:\n"
        "    - step_id: 1\n"
        "      name: 'Takeoff'\n"
        "      action: 'hover'\n"
        "      target_altitude_m: 8.0\n"
        "      advance_mode: 'time_based'\n"
        "      duration_s: 1.0\n"
        "    - step_id: 2\n"
        "      name: 'Move'\n"
        "      action: 'go_to_position'\n"
        "      target_position_enu_m:\n"
        "        x: 10.0\n"
        "        y: 5.0\n"
        "      target_altitude_m: 8.0\n"
        "      max_tilt_rad: 0.3\n"
        "      max_velocity_mps: 3.0\n"
        "      advance_mode: 'completion_based'\n"
        "      completion_criteria:\n"
        "        condition_type: 'position_reached'\n"
        "        target_position_enu_m:\n"
        "          x: 10.0\n"
        "          y: 5.0\n"
        "        position_tolerance_m: 1.0\n"
        "        target_altitude_m: 8.0\n"
        "        altitude_tolerance_m: 0.5\n");

    drone::mission::MissionLoader loader;
    drone::mission::Mission mission;
    std::string error;

    const bool ok = loader.loadFromFile(yaml_path.string(), mission, &error);

    REQUIRE(ok);
    REQUIRE(error.empty());
    REQUIRE(mission.name == "Loader Test");
    REQUIRE(mission.steps.size() == 2);
    REQUIRE(mission.steps[0].step_id == 1);
    REQUIRE(mission.steps[1].step_id == 2);

    const auto& criteria = mission.steps[1].completion_criteria;
    REQUIRE(criteria.condition_type == drone::mission::CompletionConditionType::POSITION_REACHED);
    REQUIRE(criteria.target_position_enu_m.x == 10.0);
    REQUIRE(criteria.target_position_enu_m.y == 5.0);
    REQUIRE(criteria.position_tolerance_m == 1.0);
    REQUIRE(criteria.target_altitude_m == 8.0);
    REQUIRE(criteria.altitude_tolerance_m == 0.5);

    std::filesystem::remove(yaml_path);
}

TEST_CASE("MissionLoader rejects mission without steps", "[MissionLoader]") {
    const auto yaml_path = writeTestYaml(
        "mission_loader_invalid.yaml",
        "mission:\n"
        "  name: 'Invalid'\n"
        "  version: '1.0'\n");

    drone::mission::MissionLoader loader;
    drone::mission::Mission mission;
    std::string error;

    const bool ok = loader.loadFromFile(yaml_path.string(), mission, &error);

    REQUIRE_FALSE(ok);
    REQUIRE_FALSE(error.empty());

    std::filesystem::remove(yaml_path);
}

TEST_CASE("MissionLoader falls back to TIME_ELAPSED for unknown condition_type", "[MissionLoader]") {
    const auto yaml_path = writeTestYaml(
        "mission_loader_unknown_condition.yaml",
        "mission:\n"
        "  name: 'Unknown Condition Fallback'\n"
        "  version: '1.0'\n"
        "  steps:\n"
        "    - step_id: 1\n"
        "      name: 'Fallback step'\n"
        "      action: 'hover'\n"
        "      target_altitude_m: 2.0\n"
        "      advance_mode: 'completion_based'\n"
        "      completion_criteria:\n"
        "        condition_type: 'not_a_real_condition'\n"
        "        duration_s: 0.25\n");

    drone::mission::MissionLoader loader;
    drone::mission::Mission mission;
    std::string error;

    const bool ok = loader.loadFromFile(yaml_path.string(), mission, &error);

    REQUIRE(ok);
    REQUIRE(error.empty());
    REQUIRE(mission.steps.size() == 1);
    REQUIRE(
        mission.steps[0].completion_criteria.condition_type ==
        drone::mission::CompletionConditionType::TIME_ELAPSED);
    REQUIRE(mission.steps[0].completion_criteria.duration_s == 0.25);

    std::filesystem::remove(yaml_path);
}
