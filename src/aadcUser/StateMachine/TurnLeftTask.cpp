//
// Created by aadc on 23.10.18.
//

#include "TurnLeftTask.h"
//
// Created by aadc on 04.10.18.
//

TurnLeftTask::TurnLeftTask(int id, int extra, const Point &goal) : SubGoalTask(
        id, BaseTask::Type::turn_left, extra, goal) {

}

TurnLeftTask::TurnLeftTask(int id) : SubGoalTask(id, BaseTask::Type::turn_left) {

}

std::string TurnLeftTask::getName() {
    return "T_LEFT";
}


SubGoal TurnLeftTask::generateSubGoal(SubGoal *last) {
    lane_tracker.next(global_map, 1.0);
    if (!lane_tracker.hasWp()) {
        throw SubGoalCreationException("Lane tracker does not have wp");
    }
    return SubGoal(lane_tracker.getMapWaypoint());
}


Point TurnLeftTask::getWaypointProximityCheckCenter() {
    // this shifts the center of the proximity check, must be global coords e.g. -getRearAxis/getFrontAxis
    //car_model->getRearAxis().toGlobal(Point::Local(0,27), car_model->getHeading());
    return car_model->getCameraLocation();
}

const Point TurnLeftTask::getSteeringAnchor() {
    //define pps steering anchor point
    // car_model->getRearAxis().toLocal(Point::Local(0,0), car_model->getHeading());
    //car_model->getRearAxis().toGlobal(Point::Local(0,27), car_model->getHeading());
    return car_model->getCameraLocation();
}

float TurnLeftTask::getWaypointProximity() {
    return WP_PROX;
}

float TurnLeftTask::getGoalProximity() {
    return GOAL_PROX;
}

void TurnLeftTask::onGoalReached(Point &goal) {
    BaseTask::onGoalReached(goal);
    if (needs_relocalize) {
        lane_tracker.localize(global_map);
        needs_relocalize = false;
    }
}


void TurnLeftTask::onStartTask() {
    BaseTask::onStartTask();
    desired_task_speed = Speed::SLOW;
    light_ctrl->setIndicatorLeftEnabled(true);


    if (!lane_tracker.hasWp() && !lane_tracker.isAtPoi()) {
        LOG_ERROR("Attempt to start turn task at non-poi waypoint");
        error = true;
        return;
    }
    generateWaypoints();
    if (env_state->getJunctionMarker()) {
    Marker::MarkerType type = env_state->getJunctionMarker()->getMarkerType();
    if (type == Marker::MarkerType::STOP) {
        doWait(2);
    }
}
}

void TurnLeftTask::onBeforeUpdate() {
    SubGoalTask::onBeforeUpdate();
}

float TurnLeftTask::getDrivingSpeed() {
    return Speed::SLOW;
}




void TurnLeftTask::generateWaypoints() {

    vector<Point> wps;
        if (lane_tracker.isError(lane_tracker.getWaypointsLeftTurn(global_map, &wps))) {
            LOG_ERROR("Obtained waypoints for left turn, but lane tracker status was error");
            error = true;
        }
        //buildWaypointsTurnLeft();


    if (wps.empty()) {
        error = true;
        LOG_ERROR("Failed to set waypoints for turning: no points received");
        return;
    }

    for (auto wp: wps) {
        waypoints.emplace_back(wp);
    }


    goal = waypoints.back();
    Point steering_pt = lane_tracker.poseToPoint(
            lane_tracker.nextPlanningPoint(global_map, 0.8).pose);
    waypoints.emplace_back(steering_pt);

}

void TurnLeftTask::buildWaypointsTurnLeft() {

    Point junction_pt = lane_tracker.getWaypoint();
    if (lane_tracker.isError(lane_tracker.nextLeftTurn(global_map))) {
        LOG_ERROR("left turn delivered wp type error");
        needs_relocalize = true;
        error = true;
        return;
    }

    Point rear = junction_pt;
    float heading = car_model->getHeading();


    if (heading > -0.785398 && heading < 0.785398) {
        waypoints.emplace_back(Point::Global(rear.getX() + 40, rear.getY(), rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() + 85, rear.getY() + 75, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() + 95, rear.getY() + 105, rear.getZ()));

    } else if (heading > 0.785398 && heading < 2.356) {
        waypoints.emplace_back(Point::Global(rear.getX(), rear.getY() + 40, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() - 75, rear.getY() + 85, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() - 105, rear.getY() + 95, rear.getZ()));

    } else if (fabs(heading) > 2.356) {
        waypoints.emplace_back(Point::Global(rear.getX() - 40, rear.getY(), rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() - 85, rear.getY() - 75, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() - 95, rear.getY() - 105, rear.getZ()));

    } else {
        waypoints.emplace_back(Point::Global(rear.getX(), rear.getY() - 40, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() + 75, rear.getY() - 85, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() + 105, rear.getY() - 95, rear.getZ()));
    }
}



