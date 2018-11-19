//
// Created by aadc on 25.10.18.
//

#ifndef AADC_USER_MERGETASK_H
#define AADC_USER_MERGETASK_H

#include "LaneFollowingPerceptionTask.h"

class MergeTask: public LaneFollowingPerceptionTask {


    bool is_merge_in_progress = false;

public:
    MergeTask(int id, BaseTask::Type merge_type):LaneFollowingPerceptionTask(id, false,
            Point::Global(0, 0)) {

    }

    std::string getName() override;
    float getDrivingSpeed() override;

    void onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) override;

    void onSubGoalReached(const SubGoal &goal) override;

    void onStartTask() override;
};


#endif //AADC_USER_MERGETASK_H
