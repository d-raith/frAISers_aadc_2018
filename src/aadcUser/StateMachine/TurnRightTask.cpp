//
// Created by aadc on 04.10.18.
//

#include "TurnRightTask.h"


TurnRightTask::TurnRightTask(int id, int extra, const Point &goal) : SubGoalTask(
        id, BaseTask::Type::turn_right, extra, goal) {

}

TurnRightTask::TurnRightTask(int id) : SubGoalTask(id, BaseTask::Type::turn_right) {

}

SubGoal TurnRightTask::generateSubGoal(SubGoal *last) {
    lane_tracker.next(global_map, 0.2);
    if (!lane_tracker.hasWp()) {
        throw SubGoalCreationException("Lane tracker does not have wp");
    }
    return SubGoal(lane_tracker.getMapWaypoint());
}


Point TurnRightTask::getWaypointProximityCheckCenter() {
    // this shifts the center of the proximity check, must be global coords e.g. -getRearAxis/getFrontAxis
    return car_model->getRearAxis().toGlobal(Point::Local(0,20), car_model->getHeading());
    //return car_model->getRearAxis();
}

const Point TurnRightTask::getSteeringAnchor() {
    //define pps steering anchor point
    // car_model->getRearAxis().toLocal(Point::Local(0,0), car_model->getHeading());
    //car_model->getRearAxis().toGlobal(Point::Local(0,27), car_model->getHeading());
    return car_model->getRearAxis().toGlobal(Point::Local(0,20), car_model->getHeading());
}

float TurnRightTask::getWaypointProximity() {
    return WP_PROX;
}

float TurnRightTask::getGoalProximity() {
    return GOAL_PROX;
}

void TurnRightTask::onGoalReached(Point &goal) {
    BaseTask::onGoalReached(goal);
    if (needs_relocalize) {
        lane_tracker.localize(global_map);
        needs_relocalize = false;
    }
}

std::string TurnRightTask::getName() {
    return "T_RIGHT";
}


void TurnRightTask::onStartTask() {
    BaseTask::onStartTask();
    desired_task_speed = Speed::SLOW;

    light_ctrl->setIndicatorRightEnabled(true);

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

void TurnRightTask::onBeforeUpdate() {
    SubGoalTask::onBeforeUpdate();
}

float TurnRightTask::getDrivingSpeed() {
    return Speed::SLOW;
}




void TurnRightTask::generateWaypoints() {

    vector<Point> wps;


    if (lane_tracker.isError(lane_tracker.getWaypointsRightTurn(global_map, &wps))) {
        LOG_ERROR("Obtained waypoints for right turn, but lane tracker status was error");
        error = true;
    }




    if (wps.empty()) {
        error = true;
        LOG_ERROR("Failed to set waypoints for turning: no points received");
        return;
    }

    for (auto wp: wps) {
        waypoints.emplace_back(wp);
    }


    lane_tracker.next(global_map, 0.2);
    waypoints.emplace_back(lane_tracker.getWaypoint());
    lane_tracker.next(global_map, 0.2);
    waypoints.emplace_back(lane_tracker.getWaypoint());
    lane_tracker.next(global_map, 0.2);
    waypoints.emplace_back(lane_tracker.getWaypoint());
    goal = waypoints.back();
    lane_tracker.next(global_map, 0.2);
    waypoints.emplace_back(lane_tracker.getWaypoint());
}

void TurnRightTask::buildWaypointsTurnRight() {

    Point junction_pt = lane_tracker.getWaypoint();

    if (lane_tracker.isError(lane_tracker.nextRightTurn(global_map))) {
        LOG_INFO("right turn delivered wp type error");
        //needs_relocalize = true;
        error = true;
        return;
    }

    Point rear = junction_pt;
    float heading = car_model->getHeading();
    if (heading > -0.785398 && heading < 0.785398) {
        waypoints.emplace_back(Point::Global(rear.getX() + 15, rear.getY() - 10, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() + 45, rear.getY() - 45, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() + 45, rear.getY() - 90, rear.getZ()));
    } else if (heading > 0.785398 && heading < 2.356) {
        waypoints.emplace_back(Point::Global(rear.getX() + 10, rear.getY() + 15, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() + 45, rear.getY() + 45, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() + 90, rear.getY() + 45, rear.getZ()));

    } else if (fabs(heading) > 2.356) {
        waypoints.emplace_back(Point::Global(rear.getX() - 10, rear.getY() + 15, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() - 45, rear.getY() + 45, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() - 45, rear.getY() + 90, rear.getZ()));

    } else {
        waypoints.emplace_back(Point::Global(rear.getX() - 20, rear.getY(), rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() - 45, rear.getY() - 45, rear.getZ()));
        waypoints.emplace_back(Point::Global(rear.getX() - 90, rear.getY() - 45, rear.getZ()));
    }
}



