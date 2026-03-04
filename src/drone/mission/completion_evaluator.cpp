#include "drone/mission/completion_evaluator.h"

#include "drone/runtime/real_drone.h"

#include <cmath>

namespace drone::mission {

CompletionEvaluator::CompletionEvaluator() = default;

bool CompletionEvaluator::isMet(const CompletionCriteria& criteria,
                                const runtime::SensorFrame& sensor_frame,
                                double dt_s) {
    bool condition_satisfied = false;

    switch (criteria.condition_type) {
        case CompletionConditionType::TIME_ELAPSED: {
            condition_satisfied = true;
            break;
        }

        case CompletionConditionType::ALTITUDE_REACHED: {
            double alt_error = std::abs(sensor_frame.altitude_m - criteria.target_altitude_m);
            condition_satisfied = alt_error <= criteria.altitude_tolerance_m;
            break;
        }

        case CompletionConditionType::ALTITUDE_AND_VELOCITY: {
            double alt_error = std::abs(sensor_frame.altitude_m - criteria.target_altitude_m);
            double vel_mag = std::sqrt(
                sensor_frame.gps_velocity_north_mps * sensor_frame.gps_velocity_north_mps +
                sensor_frame.gps_velocity_east_mps * sensor_frame.gps_velocity_east_mps +
                sensor_frame.gps_velocity_down_mps * sensor_frame.gps_velocity_down_mps);

            condition_satisfied = (alt_error <= criteria.altitude_tolerance_m) &&
                                  (vel_mag <= criteria.max_velocity_mps);
            break;
        }

        case CompletionConditionType::POSITION_REACHED: {
            const double alt_error = std::abs(sensor_frame.position_enu_z_m - criteria.target_altitude_m);
            const double dx = sensor_frame.position_enu_x_m - criteria.target_position_enu_m.x;
            const double dy = sensor_frame.position_enu_y_m - criteria.target_position_enu_m.y;
            const double pos_error = std::sqrt(dx * dx + dy * dy);

            condition_satisfied = (pos_error <= criteria.position_tolerance_m) &&
                                  (alt_error <= criteria.altitude_tolerance_m);
            break;
        }

        case CompletionConditionType::YAW_REACHED: {
            double yaw_error = std::abs(sensor_frame.yaw_rad - criteria.target_yaw_rad);
            if (yaw_error > M_PI) {
                yaw_error = 2.0 * M_PI - yaw_error;
            }
            condition_satisfied = yaw_error <= criteria.yaw_tolerance_rad;
            break;
        }

        case CompletionConditionType::ATTITUDE_REACHED: {
            double pitch_error = std::abs(sensor_frame.pitch_rad - criteria.target_pitch_rad);
            double roll_error = std::abs(sensor_frame.roll_rad - criteria.target_roll_rad);
            double yaw_error = std::abs(sensor_frame.yaw_rad - criteria.target_yaw_rad);

            if (yaw_error > M_PI) {
                yaw_error = 2.0 * M_PI - yaw_error;
            }

            condition_satisfied = (pitch_error <= criteria.pitch_tolerance_rad) &&
                                  (roll_error <= criteria.roll_tolerance_rad) &&
                                  (yaw_error <= criteria.yaw_tolerance_rad);
            break;
        }

        case CompletionConditionType::VELOCITY_LOW: {
            double vel_mag = std::sqrt(
                sensor_frame.gps_velocity_north_mps * sensor_frame.gps_velocity_north_mps +
                sensor_frame.gps_velocity_east_mps * sensor_frame.gps_velocity_east_mps +
                sensor_frame.gps_velocity_down_mps * sensor_frame.gps_velocity_down_mps);

            condition_satisfied = vel_mag <= criteria.max_velocity_mps;
            break;
        }

        case CompletionConditionType::LANDED: {
            condition_satisfied = sensor_frame.altitude_m <= criteria.altitude_tolerance_m;
            break;
        }
    }

    if (condition_satisfied) {
        if (last_condition_met_) {
            hold_duration_s_ += dt_s;
        } else {
            hold_duration_s_ = 0.0;
            last_condition_met_ = true;
        }
        return hold_duration_s_ >= criteria.hold_duration_s;
    }

    hold_duration_s_ = 0.0;
    last_condition_met_ = false;
    return false;
}

void CompletionEvaluator::reset() {
    hold_duration_s_ = 0.0;
    last_condition_met_ = false;
}

}  // namespace drone::mission
