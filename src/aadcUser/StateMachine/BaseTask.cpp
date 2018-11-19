#include "BaseTask.h"



void BaseTask::logDebug() {
    if (current_constraint && current_constraint->isActive()) {
        LOG_INFO("Task: constraint active since: %f | remaining ms: %f ",
                 current_constraint->getTimer().getElapsedMs()/1000, current_constraint->getTimer()
                 .getRemainingMs());
        LOG_INFO("Speed: %f", current_constraint->getDrivingSpeed());
    }
}


Point &BaseTask::getCurrentWaypoint() {
    if (waypoints.empty()) {
        throw WaypointsEmptyException();
    }
    return waypoints.front();
}


bool BaseTask::isPointPassed(const Point &target) {
    return isPointPassed(target, is_reverse_movement, point_passed_offset_tolerance);
}

bool BaseTask::isPointPassed(const Point &origin, const Point &target) {
    return isPointPassed(origin, target, is_reverse_movement, point_passed_offset_tolerance);
}

bool BaseTask::isPointPassed(const Point &target, bool invertCheck,
                             int offset_tolerance) {
    return isPointPassed(car_model->getRearAxis(), target, invertCheck, offset_tolerance);
}

bool BaseTask::isPointPassed(const Point &origin, const Point &target, bool invertCheck,
                             int offset_tolerance) {
    Point local = origin.toLocal(target, car_model->getHeading());

    if (invertCheck) {
        return local.getY() > offset_tolerance;
    }

    return local.getY() < offset_tolerance;
}

/** State checking**/

bool BaseTask::isGoalReached(Point &position) {
    return isValidGoal(goal) &&
           (position.distanceTo(goal) < getGoalProximity() || isPointPassed(goal,
                                                                            is_reverse_movement,
                                                                            point_passed_offset_tolerance));
}

bool BaseTask::isGoalReached() {
    return isGoalReached(wp_check_center_global);
}

bool BaseTask::isWaypointReached(Point &position, Point &waypoint) {
    return position.distanceTo(waypoint) < getWaypointProximity()
           || isPointPassed(waypoint, is_reverse_movement, point_passed_offset_tolerance);
}

bool BaseTask::isWaypointReached(Point &waypoint) {
    return isWaypointReached(wp_check_center_global, waypoint);
}

bool BaseTask::isValidGoal(const Point &goal) {
    return !goal.isInitPoint();
}

Point BaseTask::getCurrentGoalPoint() {
    return goal;
}

void BaseTask::onBeforeUpdate() {

}

void BaseTask::writeDebugOutputToLocalMap(cv::Mat *local_map_img) {

}


/**Override this to provide custom proxyimity checking (e.g. with offset)*/
Point BaseTask::getWaypointProximityCheckCenter() {
    return car_model->getRearAxis();
}


void BaseTask::onUpdateProximityCenter(Point new_center) {
    this->wp_check_center_global = new_center;
}


const Point BaseTask::getSteeringAnchor() {

    return car_model->toGlobal(0, 36);
}


/**Control loop**/


void BaseTask::update(const float *current_speed) {
    if (!car_model->isInitialized()) {
        cout << "position pointer or position itself not valid " << endl;
    }


    if (!local_map->isInitialized()) {
        cout << "Local map not initialized" << endl;
        return;
    }

    onUpdateProximityCenter(getWaypointProximityCheckCenter());

    if (goal_reached) {
        // wait for state machine to detect finish and switch tasks
        LOG_INFO("Goal reached, stopping");
        setVelocity(Speed::STOP);
        return;
    }

    if (goal.isInitPoint() && !onNoGlobalGoalAvailable()) {
        LOG_INFO("no global goal set, returning");
        return;
    }


    if (!goal_reached && isGoalReached()) {
        onGoalReached(goal);
        return;
    }

    onBeforeUpdate();

    while (!waypoints.empty() && isWaypointReached(getCurrentWaypoint())) {
        onWaypointReached(getCurrentWaypoint());
        waypoints.pop_front();
    }


    desired_task_speed = getDrivingSpeed();
    if (verifyDrivingConstraintsExists() && applyDrivingConstraints()) {
        return;
    }

    if (isWaiting()) {
        // override speed if waiting timer is active
        desired_task_speed = Speed::STOP;
    }


    if (!waypoints.empty()) {
        setNextWaypoint(getCurrentWaypoint());
        setVelocity(desired_task_speed);
    } else {
        setVelocity(Speed::SLOW);
        onEmptyWaypoints();
        LOG_INFO("No more waypoints, stopping");
    }
}


/**Lifecycle / Callback methods **/

void BaseTask::onStartTask() {

}


bool BaseTask::verifyDrivingConstraintsExists() {
    if (current_constraint && current_constraint->isActive()) {
        //LOG_INFO("Active constraint %s exists", current_constraint->getSourceTypeReadable().c_str());
        return true;
    }
    onCheckStVoConstraints();
    /*if (!current_constraint->isActive()) {
        onCheckStVoConstraints(current_constraint);
    }
    if(!current_constraint->isActive()){
        current_constraint.reset();
    }*/
    if (current_constraint) {
        //LOG_INFO("Current constraint: %s", current_constraint->getSourceTypeReadable().c_str());
        return true;
    }
    return false;
}


void BaseTask::onCheckStVoConstraints() {

    // move this to individual tasks if logic gets more complex
    bool at_junction = getType() == BaseTask::Type::turn_right || getType()
            == BaseTask::Type::turn_left || getType() == BaseTask::Type::straight;
    std::string task_name = getName();
    at_junction = at_junction
    && task_name != "LFP"
    && task_name != "Overtake"
    && task_name != "Merge"
    && task_name != "Reverse"
    && task_name != "WP";

    BaseTask::Type task_type = getType();
    at_junction = at_junction
    && task_type != BaseTask::Type::pull_out_left
    && task_type != BaseTask::Type::pull_out_right
    && task_type != BaseTask::Type::cross_parking
    && task_type != BaseTask::Type::parallel_parking;

    stVo.updateConstraint(*env_state, this, at_junction, current_constraint);
}

bool BaseTask::applyDrivingConstraints() {
   return current_constraint->apply(&desired_task_speed, &waypoints, this);
    // waypoints = ...
}


bool BaseTask::onNoGlobalGoalAvailable() {
    // true indicates ok, continue update loop;
    return false;
}


void BaseTask::onGoalReached(Point &goal) {
    LOG_INFO("Global goal reached: %f, %f", goal.getX(), goal.getY());
    goal_reached = true;
    waypoints.clear();
}

void BaseTask::onWaypointReached(Point &wp) {
    // LOG_INFO("Waypoint reached: %f %f", wp.getX(), wp.getY());
}


void BaseTask::onEmptyWaypoints() {

}

void BaseTask::onMarkerDetected(Marker *marker) {

    if (!marker) {
        // reset
        detected_marker = nullptr;
        return;
    }


    bool trigger_always = true;


    if (detected_marker && (*detected_marker) == (*marker) && !trigger_always) {
        // marker seen already
        return;
    }


    this->detected_marker = marker;


    auto iter_found = marker_trigger_map.find(marker->getMarkerType());
    if (iter_found == marker_trigger_map.end()) {

        LOG_DUMP("No registration for marker type");
        return;
    }

    //float trigger_dist = iter_found->second;

    auto distance = static_cast<float>(car_model->getRearAxis().distanceTo(*marker)
                                       / Coordinate::GLOBAL);
    // distance check not working atm
    //cout << "dist to marker: " << distance << endl;
    /*if (distance > trigger_dist) {
        LOG_DUMP("Marker too far away to trigger, dist: %f | %f", distance, trigger_dist);
        return;
    }*/

    switch (detected_marker->getMarkerType()) {

        case Marker::CROSSING:
            onCrossingSign(marker, distance);
            break;
        case Marker::STOP:
            onStopSign(marker, distance);
            break;
        case Marker::PARKING_AHEAD:
            onParkSign(marker, distance);
            break;
        case Marker::RIGHT_OF_WAY:
            onRoWSign(marker, distance);
            break;
        case Marker::STRAIGHT_ONLY:
            onStraightOnly(marker, distance);
            break;
        case Marker::GIVE_WAY:
            onGiveWaySign(marker, distance);
            break;
        case Marker::ZEBRA:
            onZebraCrossingSign(marker, distance);
            break;
        case Marker::ROUNDABOUT:
            onRoundaboutSign(marker, distance);
            break;
        case Marker::NO_OVERTAKING:
            onNoOvertakingSign(marker, distance);
            break;
        case Marker::NO_ENTERING:
            onNoEnteringSign(marker, distance);
            break;
        case Marker::ONE_WAY:
            onOneWaySign(marker, distance);
            break;
        case Marker::CONSTRUCTION:
            onConstructionSign(marker, distance);
            break;
        case Marker::SPEED_LIMIT_50:
            onSpeedLimitSign(marker, distance);
            break;
        case Marker::SPEED_LIMIT_100:
            onSpeedLimitSign(marker, distance);
            break;
        case Marker::POSITIONING:
            onPositioningSign(marker, distance);
            break;
    }
}


void BaseTask::onLaneDataAvailable(vector<Lane> &laneData) {

}

void BaseTask::onObstacleDetected(Obstacle &obstacle) {

}


float BaseTask::computeCurvature(const Point &waypoint) {
    return pps_steering.getCurvature(getSteeringAnchor(), waypoint);
    /*
    float lookahead = 0.5;

    Point anchor = getSteeringAnchor();
    Point steer_target = Point::Global(0, 0);

    for (auto &wp : waypoints) {
        if (anchor.distanceTo(steer_target) > lookahead) {
            return pps_steering.getCurvature(anchor, wp);
        }
    }

    if (!waypoints.empty()) {
        LOG_INFO("No waypoint found outside lookahead, using last");
        return pps_steering.getCurvature(anchor, waypoints.back());
    }

    LOG_INFO("No waypoints, steering straight");
    return pps_steering.getCurvature(anchor, anchor.toGlobal(Point::Local(0, 10), car_model->getHeading()));
    */
}


bool BaseTask::isOvertakeRequested() {
    return is_overtake_required;
}

void BaseTask::requestOvertake() {
    is_overtake_required = true;
}

void BaseTask::doReverseEmBrakeRecovery() {
    desired_task_speed = -0.4f;
    setVelocity(desired_task_speed);
    setNextWaypoint(car_model->getFrontAxis().toGlobal(Point::Local(0, 10), car_model->getHeading
    ()));
}



bool BaseTask::setNextWaypoint(const Point &waypoint) {
    return steer_ctrl->setCurvature(computeCurvature(waypoint));
}

bool BaseTask::setVelocity(const float speed) {
    return speed_ctrl->setVehicleSpeed(speed);
}


/**Setter methods invoked once**/

void BaseTask::setLocalMap(shared_ptr<LocalMap> &localMap) {
    local_map = localMap;
    if (local_map) {
        onLocalMapAvailable(local_map);
    }
}

void BaseTask::setGlobalMap(IGlobalMap *map) {
    global_map = map;
    if (global_map) {
        onGlobalMapAvailable(global_map);
    }
}


void BaseTask::setPlannerInstance(IPlanner *planner) {
    this->planner = planner;

    if (this->planner) {
        onPlannerAvailable(this->planner);
    }
}

void BaseTask::setCarModel(std::shared_ptr<CarModel> &carModel) {
    ICarModelDependent::setCarModel(carModel);
    lane_tracker.setCarModel(carModel);
    pps_steering.setCarModel(carModel);
}

void BaseTask::setEnvironmentState(const shared_ptr<EnvironmentState> &state) {
    env_state = state;
}

float BaseTask::getDrivingSpeed() {
    return Speed::SLOW;
}

shared_ptr<DrivingConstraint> BaseTask::getCurrentDrivingConstraint() {
    return current_constraint;
}









