//
// Created by aadc on 25.10.18.
//

#include "MergeTask.h"


bool isMergeLane(const SubGoal &sg) {
    return sg.getMapObj().lp.lane_info.is_merging;
}
float MergeTask::getDrivingSpeed() {
    if (is_merge_in_progress) {
            if (env_state->getProbObsLeft() >=0.3) {
                return Speed::STOP;
            }
        return Speed::SLOW * 0.6;
    }
    return Speed::SLOW;
}

std::string MergeTask::getName() {
    return "Merge";
}

void MergeTask::onStartTask() {
     LaneFollowingPerceptionTask::onStartTask();
}

void MergeTask::onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) {
    LaneFollowingPerceptionTask::onSubGoalChanged(old_sg, new_sg);

    // detect first merge wp, set merge flag
    if (is_merge_in_progress) {
        LOG_INFO("merge in progress");
        float p_obs_left = env_state->getProbObsLeft();
        float p_obs_rear = env_state->getProbObsRear();
        LOG_INFO("US probs: left: %f, rear: %f", p_obs_left, p_obs_rear);
    }
    

    if (!isMergeLane(old_sg) && isMergeLane(new_sg)) {
        light_ctrl->setIndicatorLeftEnabled(true);
        is_merge_in_progress = true;
    }
}

void MergeTask::onSubGoalReached(const SubGoal &goal) {
    LaneFollowingPerceptionTask::onSubGoalReached(goal);


    // if merge is in progress and next wp is no merge wp anymore, we merged successfully

    if (is_merge_in_progress && !isMergeLane(goal)) {
        goal_reached = true;
        LOG_INFO("Merge complete");
    }
}
