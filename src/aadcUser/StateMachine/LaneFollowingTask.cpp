//
// Created by aadc on 03.10.18.
//

#include "LaneFollowingTask.h"


LaneFollowingTask::LaneFollowingTask(int id, bool exit_on_parking, Point goal) :
SubGoalTask(id, BaseTask::Type::straight, -1, goal), exit_on_parking(exit_on_parking) {
}

void LaneFollowingTask::logDebug() {

}
void LaneFollowingTask::onSubGoalReached(const SubGoal &goal) {
    if (lane_tracker.isAtPoi()) {
        goal_reached = true;
    } else {
        SubGoalTask::onSubGoalReached(goal);
    }
}

void LaneFollowingTask::onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) {
    SubGoalTask::onSubGoalChanged(old_sg, new_sg);

    if (old_sg.getMapObj().lp.lane_info.is_ramp_up != new_sg.getMapObj().lp.lane_info.is_ramp_up) {
        bool enable_em_brake = !new_sg.getMapObj().lp.lane_info.is_ramp_up;
        em_brake_ctrl->setEmergencyBrakeEnabled(enable_em_brake);
        LOG_INFO("Emergency brake enabled: %d", enable_em_brake);
    }
}


SubGoal LaneFollowingTask::generateSubGoal(SubGoal *last) {
    if (!lane_tracker.hasWp()) {
       lane_tracker.localize(global_map);
    } else {
        lane_tracker.next(global_map, .5);
    }



    SubGoal g = SubGoal(lane_tracker.getMapWaypoint());
    LOG_INFO("Subgoal generated: %f %f (type: %d)", g.point.getX(), g.point.getY(), g.getMapObj().type);
    return g;
}

void LaneFollowingTask::updateCachedLanes(Lane *left, Lane *right) {


}


bool LaneFollowingTask::onNoGlobalGoalAvailable(){
    return true;
}

void LaneFollowingTask::onBeforeUpdate() {
    SubGoalTask::onBeforeUpdate();
}


void LaneFollowingTask::onEmptyWaypoints() {
    SubGoalTask::onEmptyWaypoints();
}




void LaneFollowingTask::onLaneDataAvailable(vector<Lane> &laneData) {

}


void LaneFollowingTask::addPerceptionAsWaypoints(vector<Point> *src, Point globalConversionAnchor, bool scale){

    bool do_conversion = !globalConversionAnchor.isInitPoint();

    for (auto elem : *src) {


        if (scale) {
            elem.scaleBy(Coordinate::Type::GLOBAL);
        }


        if (do_conversion) {
            //LOG_INFO("Local: %f, %f", elem.getX(), elem.getY());
            globalConversionAnchor.toGlobalNoCopy(elem, car_model->getHeading());
            //LOG_INFO("Global Perc.: %f, %f", elem.getX(), elem.getY());
            waypoints.push_front(elem);
        } else {
            waypoints.push_front(elem);
        }
    }
}


void LaneFollowingTask::onLanePointDataAvailable(std::vector<Point> *input) {
    BaseTask::onLanePointDataAvailable(input);

    waypoints.clear();

    Point front = car_model->getBirdseyeAnchor();
    addPerceptionAsWaypoints(input, front, true);
}

float LaneFollowingTask::getDrivingSpeed() {
    if (!zebra_timer.isDone()) {
        light_ctrl->setHazardLightsEnabled(true);
        LOG_INFO("Zebra detection running");
        if (!em_brake_ctrl->getLastTriggerWatch()->didSecondsPass(5)) {
            LOG_INFO("EM brake was triggered, stopping");
            em_brake_ctrl->getLastTriggerWatch()->print_measurement("ls em brake trigger");
            return Speed::STOP;
        }
        return SPEED_CROSSWALK;
    }
    return SPEED_NORMAL;
}


void LaneFollowingTask::onObstacleDetected(Obstacle &obstacle) {
    SubGoalTask::onObstacleDetected(obstacle);
}


void LaneFollowingTask::onStartTask() {
    SubGoalTask::onStartTask();

    addMarkerTrigger(Marker::MarkerType::ZEBRA, 1.0);
    LOG_INFO("Lane following starting");

}


void LaneFollowingTask::onParkSign(Marker *marker, float dist_m) {
    BaseTask::onParkSign(marker, dist_m);
    if (exit_on_parking) {
        LOG_INFO("Parking sign detected, finishing lane following");
        goal_reached = true;
    }

}

void LaneFollowingTask::onStraightOnly(Marker *marker, float dist) {
    SubGoalTask::onStraightOnly(marker, dist);
}

void LaneFollowingTask::onStopSign(Marker *marker, float dist) {
    SubGoalTask::onStopSign(marker, dist);
}

void LaneFollowingTask::onCrossingSign(Marker *marker, float dist) {
    SubGoalTask::onCrossingSign(marker, dist);
}


void LaneFollowingTask::writeDebugOutputToLocalMap(cv::Mat *local_map_img) {
    if (!waypoints.empty()) {
        //MatUtils::interpolatePoints(localMapImg, &waypoints);
        for (auto wp : waypoints) {

            wp = local_map->convertToLocalFrame(wp);

            if (MatUtils::inBoundaries(*local_map_img, wp)) {
                int x = static_cast<int>(ceil(wp.getX()));
                int y = static_cast<int>(ceil(wp.getY()));
                local_map_img->at<cv::Vec3b>(x, y) = cv::Vec3b(255, 165, 0);
            }

        }

        Point goal = local_map->convertToLocalFrame(goal);
        if (MatUtils::inBoundaries(*local_map_img, goal)) {
            int x = static_cast<int>(ceil(goal.getX()));
            int y = static_cast<int>(ceil(goal.getY()));
            local_map_img->at<cv::Vec3b>(x, y) = cv::Vec3b(0, 255, 0);
        } else {
            /*cout << "WARN: goal not in local map (visualization): " << goal << "(global: "
                 << waypoints[waypoints.size() - 1] <<
                 ")" << endl;*/
        }
    }
}

void LaneFollowingTask::onZebraCrossingSign(Marker *marker, float dist_m) {
    BaseTask::onZebraCrossingSign(marker, dist_m);
    zebra_timer.start();
    //LOG_INFO("Zebra detected: %d", zebra_timer.isActive());
}


const Point LaneFollowingTask::getSteeringAnchor() {
    return car_model->toGlobal(0, 36);
}

void LaneFollowingTask::onUpdateProximityCenter(Point new_center) {
    SubGoalTask::onUpdateProximityCenter(new_center);
}

Point LaneFollowingTask::getWaypointProximityCheckCenter() {
    //return car_model->getFrontAxis().toGlobal(Point::Local(0, 12), car_model->getHeading());
    return car_model->getFrontAxis().toGlobal(Point::Local(0, 12), car_model->getHeading());
}

float LaneFollowingTask::getWaypointProximity() {
    return WP_PROX;
}

float LaneFollowingTask::getGoalProximity() {
    return GOAL_PROX;
}

std::string LaneFollowingTask::getName() {
    return "LF (Old)";
}