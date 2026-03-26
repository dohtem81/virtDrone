#include "drone/mission/mission_executor.h"

#include "drone/runtime/real_drone.h"

namespace drone::mission {

void MissionExecutor::loadMission(const Mission& mission) {
    mission_ = &mission;
    status_ = MissionStatus::IDLE;
    current_step_index_ = 0;
    step_elapsed_time_s_ = 0.0;
    total_elapsed_time_s_ = 0.0;
    completion_evaluator_.reset();
    step_retry_count_ = 0;
    hover_reference_initialized_ = false;
}

void MissionExecutor::start() {
    if (!mission_ || mission_->steps.empty()) {
        status_ = MissionStatus::FAILED;
        return;
    }

    status_ = MissionStatus::RUNNING;
    current_step_index_ = 0;
    step_elapsed_time_s_ = 0.0;
    total_elapsed_time_s_ = 0.0;
    completion_evaluator_.reset();
    step_retry_count_ = 0;
    hover_reference_initialized_ = false;
}

void MissionExecutor::pause() {
    if (status_ == MissionStatus::RUNNING) {
        status_ = MissionStatus::PAUSED;
    }
}

void MissionExecutor::resume() {
    if (status_ == MissionStatus::PAUSED) {
        status_ = MissionStatus::RUNNING;
    }
}

void MissionExecutor::abort() {
    status_ = MissionStatus::ABORTED;
}

int MissionExecutor::getCurrentStepId() const {
    if (!mission_ || current_step_index_ >= mission_->steps.size()) {
        return -1;
    }
    return mission_->steps[current_step_index_].step_id;
}

std::string MissionExecutor::getCurrentStepName() const {
    if (!mission_ || current_step_index_ >= mission_->steps.size()) {
        return "No step";
    }
    return mission_->steps[current_step_index_].name;
}

std::string MissionExecutor::getCurrentStepTargetDescription() const {
    if (!mission_ || current_step_index_ >= mission_->steps.size()) {
        return "No target";
    }

    const auto& step = mission_->steps[current_step_index_];
    std::string target;

    if (step.action) {
        target = step.action->getDescription();
    } else {
        target = "No action";
    }

    if (step.advance_mode == AdvanceMode::COMPLETION_BASED) {
        target += " | completion: " + CompletionEvaluator::criteriaToString(step.completion_criteria);
    }

    return target;
}

void MissionExecutor::applyCurrentStepAction(runtime::RealDrone& drone,
                                             const runtime::SensorFrame& sensor_frame) {
    if (!mission_ || current_step_index_ >= mission_->steps.size()) {
        return;
    }

    const auto& step = mission_->steps[current_step_index_];
    if (!step.action || !step.enabled) {
        return;
    }

    switch (step.action->getActionType()) {
        case ActionType::HOVER: {
            auto* hover = dynamic_cast<HoverAction*>(step.action.get());
            if (hover) {
                if (!hover_reference_initialized_) {
                    hover_reference_x_m_ = sensor_frame.position_enu_x_m;
                    hover_reference_y_m_ = sensor_frame.position_enu_y_m;
                    hover_reference_initialized_ = true;
                }
                drone.setTargetPosition(hover_reference_x_m_, hover_reference_y_m_);
                drone.setTargetAltitude(hover->target_altitude_m);
                drone.setTargetYaw(hover->yaw_rad);
                drone.setPositionControlEnabled(true);
            }
            break;
        }

        case ActionType::GO_TO_POSITION: {
            auto* goto_pos = dynamic_cast<GoToPositionAction*>(step.action.get());
            if (goto_pos) {
                drone.setTargetPosition(goto_pos->target_position_enu_m.x,
                                        goto_pos->target_position_enu_m.y);
                drone.setTargetAltitude(goto_pos->target_altitude_m);
                drone.setMaxVelocity(goto_pos->max_velocity_mps);
                drone.setMaxTilt(goto_pos->max_tilt_rad);
                drone.setPositionControlEnabled(true);
            }
            break;
        }

        case ActionType::LAND: {
            auto* land = dynamic_cast<LandAction*>(step.action.get());
            if (land) {
                drone.setTargetAltitude(0.0);
                drone.setPositionControlEnabled(false);
            }
            break;
        }

        case ActionType::SET_ATTITUDE: {
            auto* attitude = dynamic_cast<SetAttitudeAction*>(step.action.get());
            if (attitude) {
                drone.setTargetPitch(attitude->target_pitch_rad);
                drone.setTargetRoll(attitude->target_roll_rad);
                drone.setTargetYaw(attitude->target_yaw_rad);
                drone.setPositionControlEnabled(false);
            }
            break;
        }

        case ActionType::CHANGE_ALTITUDE: {
            auto* alt_change = dynamic_cast<ChangeAltitudeAction*>(step.action.get());
            if (alt_change) {
                drone.setTargetAltitude(alt_change->target_altitude_m);
                drone.setPositionControlEnabled(false);
            }
            break;
        }

        case ActionType::ROTATE_YAW: {
            auto* yaw = dynamic_cast<RotateYawAction*>(step.action.get());
            if (yaw) {
                drone.setTargetYaw(yaw->target_yaw_rad);
                drone.setPositionControlEnabled(false);
            }
            break;
        }
    }
}

void MissionExecutor::advanceToNextStep() {
    if (!mission_) {
        status_ = MissionStatus::COMPLETED;
        return;
    }

    current_step_index_++;

    if (current_step_index_ >= mission_->steps.size()) {
        status_ = MissionStatus::COMPLETED;
        return;
    }

    step_elapsed_time_s_ = 0.0;
    completion_evaluator_.reset();
    step_retry_count_ = 0;
    hover_reference_initialized_ = false;
}

void MissionExecutor::handleStepTimeout() {
    if (!mission_ || current_step_index_ >= mission_->steps.size()) {
        return;
    }

    const auto& step = mission_->steps[current_step_index_];

    switch (step.timeout_behavior) {
        case TimeoutBehavior::ABORT:
            status_ = MissionStatus::ABORTED;
            break;

        case TimeoutBehavior::PROCEED:
            advanceToNextStep();
            break;

        case TimeoutBehavior::RETRY:
            if (step_retry_count_ < step.retry_count) {
                step_retry_count_++;
                step_elapsed_time_s_ = 0.0;
                completion_evaluator_.reset();
                hover_reference_initialized_ = false;
            } else {
                status_ = MissionStatus::ABORTED;
            }
            break;
    }
}

void MissionExecutor::update(runtime::RealDrone& drone,
                             const runtime::SensorFrame& sensor_frame,
                             double dt_s) {
    if (status_ != MissionStatus::RUNNING || !mission_ || mission_->steps.empty()) {
        return;
    }

    total_elapsed_time_s_ += dt_s;
    step_elapsed_time_s_ += dt_s;

    if (current_step_index_ >= mission_->steps.size()) {
        status_ = MissionStatus::COMPLETED;
        return;
    }

    const auto& step = mission_->steps[current_step_index_];
    if (!step.enabled) {
        advanceToNextStep();
        return;
    }

    applyCurrentStepAction(drone, sensor_frame);

    bool step_complete = false;

    if (step.advance_mode == AdvanceMode::TIME_BASED) {
        step_complete = step_elapsed_time_s_ >= step.duration_s;
    } else if (step.advance_mode == AdvanceMode::COMPLETION_BASED) {
        step_complete = completion_evaluator_.isMet(step.completion_criteria, sensor_frame, dt_s);
    }

    if (step_elapsed_time_s_ > step.timeout_s) {
        handleStepTimeout();
        return;
    }

    if (step_complete) {
        advanceToNextStep();
    }
}

}  // namespace drone::mission
