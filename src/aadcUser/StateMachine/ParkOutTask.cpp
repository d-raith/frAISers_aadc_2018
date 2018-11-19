//
// Created by aadc on 06.10.18.
//

#include "ParkOutTask.h"

ParkOutTask::ParkOutTask(int id, BaseTask::Type type) : BaseTask(id, type) {

}

std::string ParkOutTask::getName() {
    return "ParkOut";
}

Point ParkOutTask::getWaypointProximityCheckCenter() {
    return car_model->getRearAxis().toGlobal(Point::Local(0,20), car_model->getHeading());
}

float ParkOutTask::getWaypointProximity() {
    return WP_PROX;
}

float ParkOutTask::getGoalProximity() {
    return GOAL_PROX;
}


const Point ParkOutTask::getSteeringAnchor() {
    return car_model->getRearAxis().toGlobal(Point::Local(0,20), car_model->getHeading());
}


void ParkOutTask::onGoalReached(Point &goal) {
    BaseTask::onGoalReached(goal);
    lane_tracker.localize(global_map);
}

void ParkOutTask::onCheckStVoConstraints() {
    
}

void ParkOutTask::onStartTask() {
    BaseTask::onStartTask();

    float park_x = car_model->getRearAxis().getX();
    float park_y = car_model->getRearAxis().getY()+40;

    switch (getType()) {
        case BaseTask::Type::pull_out_right:
            waypoints.emplace_back(Point::Global(park_x , park_y-10));
            for (int k = park_x + 60; k <= park_x + 90; k = k + 1) {
                waypoints.emplace_back(Point::Global(k,park_y + 40));

            }
            light_ctrl->setIndicatorRightEnabled(true);
            break;
        case BaseTask::Type::pull_out_left:
            waypoints.emplace_back(Point::Global(park_x , park_y));
            for (int k = park_x - 65; k >= park_x - 75; k = k - 1) {
                waypoints.emplace_back(Point::Global(k,park_y + 85));

            }
            light_ctrl->setIndicatorLeftEnabled(true);
            break;

        default:
            error = true;
            LOG_INFO("Unknown pull out direction type: %d", getType());
            return;
    }

    goal = waypoints.back();
}
