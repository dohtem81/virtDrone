#ifndef DRONE_MISSION_MISSION_EXECUTOR_H
#define DRONE_MISSION_MISSION_EXECUTOR_H

#include "drone/mission/completion_evaluator.h"
#include "drone/mission/mission_types.h"

#include <cstddef>

namespace drone::runtime {
class RealDrone;
struct SensorFrame;
}

namespace drone::mission {

enum class MissionStatus {
    IDLE,
    RUNNING,
    PAUSED,
    COMPLETED,
    ABORTED,
    FAILED,
};

class MissionExecutor {
public:
    void loadMission(const Mission& mission);
    void start();
    void update(runtime::RealDrone& drone,
                const runtime::SensorFrame& sensor_frame,
                double dt_s);
    void pause();
    void resume();
    void abort();

    MissionStatus getStatus() const { return status_; }
    int getCurrentStepId() const;
    std::string getCurrentStepName() const;
    std::string getCurrentStepTargetDescription() const;
    double getStepElapsedTime() const { return step_elapsed_time_s_; }
    double getTotalElapsedTime() const { return total_elapsed_time_s_; }
    bool isMissionLoaded() const { return mission_ != nullptr; }

private:
    void applyCurrentStepAction(runtime::RealDrone& drone,
                                const runtime::SensorFrame& sensor_frame);
    void advanceToNextStep();
    void handleStepTimeout();

    const Mission* mission_ = nullptr;
    MissionStatus status_ = MissionStatus::IDLE;
    size_t current_step_index_ = 0;

    double step_elapsed_time_s_ = 0.0;
    double total_elapsed_time_s_ = 0.0;

    CompletionEvaluator completion_evaluator_;
    int step_retry_count_ = 0;
    bool hover_reference_initialized_ = false;
    double hover_reference_x_m_ = 0.0;
    double hover_reference_y_m_ = 0.0;
};

}  // namespace drone::mission

#endif  // DRONE_MISSION_MISSION_EXECUTOR_H
