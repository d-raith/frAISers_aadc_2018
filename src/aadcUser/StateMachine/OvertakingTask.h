//
// Created by aadc on 31.10.18.
//

#ifndef AADC_USER_OVERTAKINGTASK_H
#define AADC_USER_OVERTAKINGTASK_H

#include "LaneFollowingPerceptionTask.h"

class OvertakingTask: public LaneFollowingPerceptionTask {

    static constexpr float lane_point_distance_m = 0.2;
    Timer overtaking_timer = Timer(0);
    fraisers::models::Point initial_ot_pt = Point::Global(0,0);
    fraisers::models::Point ot_timer_trigger_pt = Point::Global(0,0);
    bool is_ot_mode = true;
    bool ot_timer_active = false;

    float dist_travelled = 0;

    SubGoal last_sg;

protected:
void planWaypoints(Point globalConversionAnchor, SubGoal goal, bool plan_to_goal,
                                   bool disable_goal_radius) override;

void onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) override;

Point calculateNewOtPoint(SubGoal *current_sg);

void onCheckStVoConstraints();

void onEmptyWaypoints() override;

public:
    OvertakingTask(int id, bool exit_on_park_sign = false, const Point &goal = Point::Global(0,0));

public:
    string getName() override;

    void onStartTask() override;

    float getGoalProximity() override;

    
};


#endif //AADC_USER_OVERTAKINGTASK_H
