//
// Created by aadc on 31.10.18.
//

#include "OvertakingTask.h"





void OvertakingTask::planWaypoints(Point globalConversionAnchor, SubGoal goal, bool plan_to_goal,
                                   bool disable_goal_radius) {


    /*bool is_ot_done = !env_state->isCarAhead()
            && !env_state->isCarRight()
            && overtaking_timer.isDone();*/

    bool is_ot_done = ot_timer_active && overtaking_timer.isDone();


    if (is_ot_done && is_ot_mode) {
        light_ctrl->setIndicatorRightEnabled(true);
        light_ctrl->setIndicatorLeftEnabled(false);
        is_ot_mode = !is_ot_done;

    }

    if (!is_ot_mode) {
        LOG_INFO("ot done");
    } else {
        LOG_INFO("ot running dist travelled: %f, timer done: %d, car ahead: %d, car right: %d",
            dist_travelled, overtaking_timer.isDone(), env_state->isCarAhead(), env_state->isCarRight());
    }



    goal.point = initial_ot_pt;
    LaneFollowingPerceptionTask::planWaypoints(globalConversionAnchor, goal, plan_to_goal,
                                                true);
}


float OvertakingTask::getGoalProximity() {
    return 0;
}

void OvertakingTask::onCheckStVoConstraints() {
    stVo.updateConstraint(*env_state, this, false, current_constraint);
    if (current_constraint && current_constraint->getSourceType() != ConstraintType::EM_BRAKE_OVRD) {
        current_constraint.reset();
    }
}

void OvertakingTask::onEmptyWaypoints() {
    if (!initial_ot_pt.isInitPoint()) {
        waypoints.emplace_front(initial_ot_pt);
    }

}

void OvertakingTask::onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) {
    SubGoalTask::onSubGoalChanged(old_sg, new_sg);
    last_sg = old_sg;
    Point front = car_model->getFrontAxis();

    if (front.distanceTo(initial_ot_pt) < 30) { // || isWaypointReached(front, initial_ot_pt)) {
        initial_ot_pt = calculateNewOtPoint(getCurrentSubGoal());
        dist_travelled += lane_point_distance_m;

        if (!is_ot_mode && goal.isInitPoint()) {
            goal = lane_tracker.poseToPoint(lane_tracker.nextPlanningPoint(global_map, 0.5).pose);
        }
    }

    if (!ot_timer_active && !ot_timer_trigger_pt.isInitPoint() && isWaypointReached(front, ot_timer_trigger_pt)) {
            overtaking_timer = Timer(2.3);
            overtaking_timer.start();
            ot_timer_active = true;
            ot_timer_trigger_pt = Point::Global(0, 0);
    }

    


}

Point OvertakingTask::calculateNewOtPoint(SubGoal *current_sg) {
    Point planning_pt = lane_tracker.poseToPoint(lane_tracker.nextPlanningPoint(global_map, lane_point_distance_m).pose);

    if (is_ot_mode) {
        planning_pt = lane_tracker.poseToPoint(global_map->getWaypointOnOppositeLane(lane_tracker
        .nextPlanningPoint(global_map, lane_point_distance_m), true).pose);
        return planning_pt;
    } else {
        return planning_pt;
    }
}

void OvertakingTask::onStartTask() {
    LaneFollowingPerceptionTask::onStartTask();
    overtaking_timer = Timer(15);
    overtaking_timer.start();
    light_ctrl->setIndicatorLeftEnabled(true);
    em_brake_ctrl->setEmergencyBrakeEnabled(true);
    initial_ot_pt = calculateNewOtPoint(getCurrentSubGoal());
    ot_timer_trigger_pt = initial_ot_pt;
}




OvertakingTask::OvertakingTask(int id, bool exit_on_park_sign, const Point &goal)
        : LaneFollowingPerceptionTask(id, exit_on_park_sign, goal) {}

string OvertakingTask::getName() {
    return "Overtake";
}
