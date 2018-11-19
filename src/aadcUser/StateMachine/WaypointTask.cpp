//
// Created by aadc on 08.10.18.
//

#include "WaypointTask.h"

std::string WaypointTask::getName() {
    return "WP";
}

Point WaypointTask::getWaypointProximityCheckCenter() {
    return car_model->getRearAxis();
}

const Point WaypointTask::getSteeringAnchor() {
    return car_model->getRearAxis();
}

float WaypointTask::getWaypointProximity() {
    return WP_PROX;
}

float WaypointTask::getGoalProximity() {
    return GOAL_PROX;
}

void WaypointTask::onStartTask() {
    BaseTask::onStartTask();
    light_ctrl->setIndicatorLeftEnabled(true);
    em_brake_ctrl->setEmergencyBrakeEnabled(false);
}

float WaypointTask::getDrivingSpeed() {
    return Speed::SLOW;
}

void WaypointTask::onCheckStVoConstraints() {

}