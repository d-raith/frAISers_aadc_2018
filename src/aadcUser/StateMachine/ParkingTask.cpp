//
// Created by aadc on 06.10.18.
//

#include "ParkingTask.h"

ParkingTask::ParkingTask(int id, int extra) : SubGoalTask(id, BaseTask::Type::cross_parking,
                                                          extra) {

}

std::string ParkingTask::getName() {
    return "Park";
}

void ParkingTask::onStartTask() {
    BaseTask::onStartTask();
    generatePath();
}

SubGoal ParkingTask::generateSubGoal(SubGoal *last) {
    if (sub_goals.empty()) {
        throw SubGoalCreationException("Parking task has reached all sub goals");
    }

    is_reverse_movement = sub_goals.front().extra == ID_REVERSE_MOVE;
    if (sub_goals.front().extra == ID_REVERSE_MOVE) {
        waypoints.clear();
        for (auto &pt : reverse_pts) {
            waypoints.emplace_back(pt);
        }
    }
    LOG_INFO("generate subgoal: %f %f", sub_goals.front().point.getX(), sub_goals.front().point.getY());
    return sub_goals.front();
}

void ParkingTask::onBeforeUpdate() {
    SubGoalTask::onBeforeUpdate();
}

void ParkingTask::onParkSign(Marker *marker, float dist_m) {
    BaseTask::onParkSign(marker, dist_m);
}

void ParkingTask::generatePath() {
    const ParkingSpace *pspace = RoadSignInfo::getInstance()->getParkingSpace(getExtra());

    if (!pspace) {
        error = true;
        LOG_INFO("Unable to obtain parking lot with id: %d", getExtra());
        return;
    }

    int park_id = getExtra();

    float park_y = pspace->f32Y;
    float park_x = pspace->f32X;
    park_x*=100;
    park_y*=100;


    if (park_id > 4) {
            //TODO: change 690 to 700 sth
        for (int k = 250; k <= park_y -30 ; k=k+2) {
            waypoints.emplace_back(Point::Global(park_x -25, k));
        }
        //waypoints.emplace_back(Point::Global(park_x-25, park_y+10));
        //waypoints.emplace_back(Point::Global(park_x-25, park_y+15));

    } else {
        for (int j = 200; j <= park_y - 30; j = j + 1) {
            waypoints.emplace_back(Point::Global(park_x - 25, j));
        }

        //waypoints.emplace_back(Point::Global(park_x - 50, park_y - 10));
        //waypoints.emplace_back(Point::Global(park_x - 60, park_y - 0));

    }

    int offset = 0 ;
    if(park_id ==7 ){
        offset = 0;
    }

    for (int d = park_x-25 ; d <= park_x + 80; d = d + 1) {
        reverse_pts.emplace_back(Point::Global(d,park_y -offset));
    }

    SubGoal front_move;
    front_move.point = waypoints.back();
    front_move.extra = ID_FRONT_MOVE;
    sub_goals.emplace_back(front_move);

    for (auto &wp : waypoints) {
        LOG_INFO("parking wp: %f %f", wp.getX(), wp.getY());
    }

    for (auto &wp : reverse_pts) {
        LOG_INFO("rev parking wp: %f %f", wp.getX(), wp.getY());
    }

    LOG_INFO("parking sub goal move forward %f %f", front_move.point.getX(), front_move.point.getY());

    SubGoal rev_move;
    rev_move.point = reverse_pts.back();
    rev_move.extra = ID_REVERSE_MOVE;
    sub_goals.emplace_back(rev_move);

    LOG_INFO("parking sub goal move reverse %f %f", rev_move.point.getX(), rev_move.point.getY());


}


float ParkingTask::getDrivingSpeed() {
    LOG_INFO("is reverse: %d", is_reverse_movement);
    int sign = is_reverse_movement ? -1 : 1;
    return sign * BaseTask::getDrivingSpeed();
}

bool ParkingTask::isSubGoalReached(const Point &position) {
    return SubGoalTask::isSubGoalReached(position);
}

void ParkingTask::onSubGoalReached(const SubGoal &goal) {
    if (!sub_goals.empty()) {
        sub_goals.pop_front();
    }

    goal_reached = sub_goals.empty();
}

Point ParkingTask::getWaypointProximityCheckCenter() {
    sub_goal_check_center_global = car_model->getRearAxis();
    return car_model->getRearAxis();
}

float ParkingTask::getWaypointProximity() {
    return WP_PROX;
}

float ParkingTask::getGoalProximity() {
    return GOAL_PROX;
}

const Point ParkingTask::getSteeringAnchor() {
    return car_model->getRearAxis();
}
