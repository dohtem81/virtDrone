#ifndef DRONE_MISSION_COMPLETION_EVALUATOR_H
#define DRONE_MISSION_COMPLETION_EVALUATOR_H

#include "drone/mission/mission_types.h"

namespace drone::runtime {
struct SensorFrame;
}

namespace drone::mission {

class CompletionEvaluator {
public:
    CompletionEvaluator();

    bool isMet(const CompletionCriteria& criteria,
               const runtime::SensorFrame& sensor_frame,
               double dt_s);

    double getHoldProgress() const { return hold_duration_s_; }

    void reset();

private:
    double hold_duration_s_ = 0.0;
    bool last_condition_met_ = false;
};

}  // namespace drone::mission

#endif  // DRONE_MISSION_COMPLETION_EVALUATOR_H
