//
// Created by aadc on 04.10.18.
//

#include "SubGoalTask.h"

bool SubGoalTask::isSubGoalReached(const Point &position) {
    //double dist = position.distanceTo(sub_goal.point);
    //bool reached = isPointPassed(sub_goal_check_center_global, sub_goal.point);

    //return dist <= sub_goal_proximity_scaled;
    return isPointPassed(position, sub_goal.point) && position.distanceTo(sub_goal.point) < sub_goal_proximity_scaled;
}

bool SubGoalTask::isSubGoalReached() {
    return isSubGoalReached(sub_goal_check_center_global);
}

void SubGoalTask::onSubGoalReached(const SubGoal &goal) {
    // LOG_INFO("Subgoal reached");
    try {
        setSubGoal(generateSubGoal(getCurrentSubGoal()));
    } catch (const SubGoalCreationException &ex) {
        LOG_INFO("Failed to update subgoal %s", ex.getCause().c_str());
    }
}

bool SubGoalTask::onNoGlobalGoalAvailable() {
    return true;
}

SubGoalTask::SubGoalTask(int id, BaseTask::Type type, int extra, const Point &goal) : BaseTask(id,
                                                                                               type,
                                                                                               extra,
                                                                                               goal) {}



void SubGoalTask::setSubGoal(const SubGoal &goal) {
    SubGoal old = sub_goal;
    sub_goal = goal;
    onSubGoalChanged(old, sub_goal);
}


void SubGoalTask::onBeforeUpdate() {
    BaseTask::onBeforeUpdate();


    if (!isValidSubGoal(getCurrentSubGoal())) {
        try {
            setSubGoal(generateSubGoal(getCurrentSubGoal()));
        } catch (const SubGoalCreationException &ex) {
            LOG_INFO("Failed to set subgoal %s", ex.getCause().c_str());
        }

    }

    if (isValidSubGoal(getCurrentSubGoal()) && isSubGoalReached()) {
        onSubGoalReached(*getCurrentSubGoal());
    }
}

SubGoal *SubGoalTask::getCurrentSubGoal() {
    return &sub_goal;
}

bool SubGoalTask::isValidSubGoal(SubGoal *sg) {
    return sg && !(*sg).point.isInitPoint();
}

void SubGoalTask::onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) {

}


void SubGoalTask::onUpdateProximityCenter(Point new_center) {
    BaseTask::onUpdateProximityCenter(new_center);
    sub_goal_check_center_global = car_model->getBirdseyeAnchor();
}

void SubGoalTask::writeDebugOutputToLocalMap(cv::Mat *local_map_img) {
    // not consistent with current check if subgoal reached
    BaseTask::writeDebugOutputToLocalMap(local_map_img);
    if (!isValidSubGoal(getCurrentSubGoal())) {
        return;
    }

    // subgoal reached checks
    cv::Point sub_goal_check_center_localmap =
        local_map->convertFromGlobalToLocalMapFrame(sub_goal_check_center_global);
        int sgccl_x = sub_goal_check_center_localmap.x;
    int sgccl_y = sub_goal_check_center_localmap.y;
    // cv::circle(*local_map_img,
    //            sub_goal_check_center_localmap,
    //            sub_goal_proximity_scaled,
    //            cv::Scalar(255, 255, 255));
    cv::ellipse(*local_map_img, sub_goal_check_center_localmap,
        cv::Size(sub_goal_proximity_scaled, sub_goal_proximity_scaled),
        0, 180, 360, cv::Scalar(255 , 255, 255), 1);
    cv::line(*local_map_img,
               cv::Point(max(sgccl_x - sub_goal_proximity_scaled, 0), sgccl_y),
               cv::Point(min(sgccl_x + sub_goal_proximity_scaled, local_map_img->size().width - 1), sgccl_y),
               cv::Scalar(255, 255, 255));

    // current subgoal
    cv::circle(*local_map_img,
               local_map->convertFromGlobalToLocalMapFrame(getCurrentSubGoal()->point),
               2,
               cv::Scalar(255, 255, 255));

    // cv::Point2i p = local_map->convertFromGlobalToLocalMapFrame(getCurrentSubGoal()->point);
    // LOG_INFO("subgoal pos [global] %f   %f", getCurrentSubGoal()->point.getX(), -getCurrentSubGoal()->point.getY());
    // LOG_INFO("subgoal pos [local] %d   %d", p.x, p.y);
}

