//
// Created by aadc on 08.10.18.
//

#include "ReverseDrivingTask.h"

std::string ReverseDrivingTask::getName() {
    return "Reverse";
}

void ReverseDrivingTask::onStartTask() {
    light_ctrl->setReverseLightsEnabled(true);
    light_ctrl->setIndicatorRightEnabled(true);
    em_brake_ctrl->setEmergencyBrakeEnabled(false);

}

float ReverseDrivingTask::getDrivingSpeed() {
    return Speed::REV_SLOW;
}

const Point ReverseDrivingTask::getSteeringAnchor() {
    return car_model->getRearAxis().toGlobal(Point::Local(0,-15), car_model->getHeading());
}

Point ReverseDrivingTask::getWaypointProximityCheckCenter() {
    return car_model->getRearAxis();
}

bool ReverseDrivingTask::isPointPassed(const Point &origin, const Point &target, bool invertCheck,
        int offset_tolerance) {
    return car_model->toLocal(target).getY() > 0;
}

float ReverseDrivingTask::getWaypointProximity() {
    return WP_PROX;
}

float ReverseDrivingTask::getGoalProximity() {
    return GOAL_PROX;
}

 void ReverseDrivingTask::onBeforeUpdate() {
     BaseTask::onBeforeUpdate();
 }

void ReverseDrivingTask::onCheckStVoConstraints() {

}
