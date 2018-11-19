//
// Created by aadc on 08.10.18.
//

#include "DriveStraightTask.h"

DriveStraightTask::DriveStraightTask(int id):BaseTask(id, BaseTask::Type::straight) {

}



void DriveStraightTask::onStartTask() {
    BaseTask::onStartTask();

    vector<Point> wps;

    if (lane_tracker.isError(lane_tracker.getWaypointsStraight(global_map, &wps)) || wps.empty()) {
        error = true;
        LOG_ERROR("Failed to set waypoints for turning: wp type error or no points received");
        return;
    }

    // for (auto wp : wps) {
    //     waypoints.emplace_back(wp);
    // }

    int n = wps.size();
    if (n > 3) {
        int i_1_3 = floor(n / 3);
        int i_2_3 = floor((2 / 3) * n);
        if (i_1_3 >= 0 && i_1_3 < n) {
            waypoints.emplace_back(wps[i_1_3]);
        }
        if (i_2_3 >= 0 && i_2_3 < n) {
            waypoints.emplace_back(wps[i_2_3]);
        }
    }
    waypoints.emplace_back(wps.back());




    /*
    Point junction = lane_tracker.getWaypoint();
    if (lane_tracker.nextStraight(global_map) == mapobjects::WaypointType::ERROR){
        LOG_ERROR("nextStraight resulted in wp type error, aborting");
        error = true;
        return;
    }

    float heading = car_model->getHeading();

    float x = junction.getX();
    float y = junction.getY();

    if (heading >-0.785398 && heading < 0.785398) {
        for (int i=x; i< x+150; i++) {
            waypoints.emplace_back(Point::Global(i, y));
        }
    } else if (heading > 0.785398 && heading < 2.356) {
        for (int i=y; i< y+150; i++) {
            waypoints.emplace_back(Point::Global(x, i));
        }
    } else if (fabs(heading) > 2.356) {
        for (int i=x; i> x-150; i--) {
            waypoints.emplace_back(Point::Global(i, y));
        }
    } else {
        for (int i=y; i> y-150; i--) {
            waypoints.emplace_back(Point::Global(x, i));
        }
    }*/

    goal = waypoints.back();
    if (env_state->getJunctionMarker()) {
    Marker::MarkerType type = env_state->getJunctionMarker()->getMarkerType();
        if (type == Marker::MarkerType::STOP) {
            doWait(2);
        }
    }
}

Point DriveStraightTask::getWaypointProximityCheckCenter() {
    return BaseTask::getWaypointProximityCheckCenter();
}

const Point DriveStraightTask::getSteeringAnchor() {
    return car_model->getRearAxis();;
}

float DriveStraightTask::getWaypointProximity() {
    return WP_PROX;
}

float DriveStraightTask::getGoalProximity() {
    return GOAL_PROX;
}


std::string DriveStraightTask::getName() {
    return "DriveStraight";
}