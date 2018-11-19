#include "PlanningTask.h"


void PlanningTask::logDebug() {
    BaseTask::logDebug();

}


/** State checking**/

std::string PlanningTask::getName() {
    return "Planning";
}

bool PlanningTask::isLocalGoalReached(Point &position) {
    return position.distanceTo(getCurrentGoalPoint()) < getGoalProximity();
}

bool PlanningTask::isLocalGoalReached() {
    return isLocalGoalReached(wp_check_center_global);
}


bool PlanningTask::onNoGlobalGoalAvailable() {
    return true;
}

Point PlanningTask::getCurrentGoalPoint() {
    if (local_goals.empty()) {
        return Point::Global(0, 0);
    }
    return local_goals.front().point;
}


SubGoal PlanningTask::getCurrentLocalGoal() {
    if (local_goals.empty()) {
        return SubGoal();
    }
    return local_goals.front();
}


void PlanningTask::onComputeLocalGoal() {
    if (!global_map) {
        LOG_INFO("No global map available");
    }
    SubGoal local_g = getCurrentLocalGoal();
    updateLocalGoal(&local_g, nullptr);

}

void PlanningTask::onBeforeUpdate() {
    BaseTask::onBeforeUpdate();


    if (is_waiting_timer_in_progress) {
        if (waiting_timer_turn.didSecondsPass(num_secs_wait_before_turn)) {
            desired_task_speed = Speed::SLOW;
            is_waiting_timer_in_progress = false;
        } else {
            desired_task_speed = Speed::STOP;
            cout << "waiting" << endl;
        }

    }

    if (!isValidGoal(getCurrentGoalPoint()) || local_goals.empty()) {
        onComputeLocalGoal();
    }

    if (!isValidGoal(getCurrentGoalPoint())) {
        LOG_INFO("Unable to compute local goal points");
        return;
    }


    if (isValidGoal(getCurrentGoalPoint()) && isLocalGoalReached()) {
        onLocalGoalReached(getCurrentLocalGoal());
    }


    if (bgPlanner.solutionAvailable()) {
        updateFromPlan(bgPlanner.getLastRequest());
    }
    if (bgPlanner.openRequestCount() > 1) {
        cout << "BgPlanner open request count: " << bgPlanner.openRequestCount() << endl;
    }

    if (!bgPlanner.hasOpenRequests() && update_plan_watch.didSecondsPass(1)) {
        requestNewPlan();
    }

}



/**Lifecycle / Callback methods **/

void PlanningTask::onGoalReached(Point &goal) {
    BaseTask::onGoalReached(goal);
}


void PlanningTask::onStartTask() {
    BaseTask::onStartTask();
    addMarkerTrigger(Marker::MarkerType::STOP, 1.2);
    addMarkerTrigger(Marker::MarkerType::CROSSING, 1.2);
    addMarkerTrigger(Marker::MarkerType::STRAIGHT_ONLY, 0.7);
    bgPlanner.start();
    update_plan_watch.beginMeasure();
}


void PlanningTask::onWaypointReached(Point &wp) {
    BaseTask::onWaypointReached(wp);
}


mapobjects::GlobalWaypoint getGoalByTurnType(BaseTask::Type type, IGlobalMap *map,
                                             const mapobjects::GlobalWaypoint &lastLp) {

    mapobjects::GlobalWaypoint next_wp(mapobjects::WaypointType::ERROR);
    switch (type) {
        case BaseTask::Type::turn_left:
            next_wp = map->getWaypointTurnLeft(lastLp.lp);

            break;
        case BaseTask::Type::turn_right:
            next_wp = map->getWaypointTurnRight(lastLp.lp);

            break;
        case BaseTask::Type::straight:
            next_wp = map->getWaypointDriveStraight(lastLp.lp);
            break;
        case BaseTask::Type::merge_left:
            throw std::runtime_error("merge not implemented yet");
        default:
            throw std::runtime_error("task type not supported");
    }

    if (next_wp.type == mapobjects::WaypointType::ERROR) {
        throw mapobjects::WaypointException::NO_OUTGOING_CONNECTIONS;
    }
    return next_wp;
}

void PlanningTask::generateLocalGoals(SubGoal *last) {
    LOG_INFO("Computing local goals");

    local_goals.clear();
    mapobjects::Pose pose;
    Point front = car_model->getFrontAxis();


    pose.setPose(front.getX(), -front.getY(), front.getZ(),
                 -car_model->getHeading());
    // double lookahead_dist = 1.5 * Coordinate::GLOBAL;

    for (auto &goal : local_goals) {
        mapobjects::GlobalWaypoint wp = goal.getMapObj();
        goal.allows_update = false;
        LOG_INFO("Local goal point: %f, %f | type: %d", wp.pose.getX(), wp.pose.getY(), wp.type);
    }

}

void PlanningTask::updateLocalGoal(SubGoal *l_goal, Marker *marker) {

    if (l_goal && !l_goal->allows_update && !marker) {
        cout << "Fixed local goal: " << l_goal->point << endl;
        return;
    }


    if (marker && false) {
        SubGoal testGoal;
        testGoal.allows_update = false;
        if (getType() == BaseTask::Type::turn_left) {
            testGoal.point = car_model->toGlobal(Point::Local(-120, 120));
            goal = testGoal.point;

            desired_task_speed = Speed::STOP;

            waiting_timer_turn.reset();
            is_waiting_timer_in_progress = true;

            LOG_INFO("Adding turning gobal goal: %f, %f", goal.getX(), goal.getY());
        }

        if (!testGoal.point.isInitPoint()) {
            (*l_goal) = testGoal;
        }

    }







    /*   if (isValidGoal(this->goal) && MatUtils::inBoundaries(local_map->channel_perception,
                                                             local_map->convertToLocalFrame(
                                                                     this->goal))) {

           if ((*l_goal).point == this->goal) {
               cout << "Global goal in range & already local goal, nothing to do" << endl;
           } else {
               cout << "Global goal in range, forcing to be local goal" << endl;
               SubGoal global_goal;
               global_goal.point = this->goal;
               global_goal.allows_update = false;
               local_goals.push_front(global_goal);

           }
           return;
       }
   */


    generateLocalGoals(l_goal);
}


void PlanningTask::requestNewPlan() {


    if (local_goals.empty()) {
        cout << "No local goals available" << "\r";
        return;
    }

    SubGoal localGoal = local_goals.front();


    cout << "Subgoal global: " << localGoal.point << endl;
    Point local_plan_start = local_map->convertToLocalFrame(car_model->getFrontAxis().toGlobal(Point::Local(0, 12), car_model->getHeading()));
    //Point local_plan_start = local_map->convertToLocalFrame(car_model->getRearAxis());

    PlanRequest request;
    request.start = local_plan_start;
    request.heading = car_model->getHeading();
    request.goal = localGoal;
    request.goal.point = local_map->convertToLocalFrame(localGoal.point);
    request.local_map = local_map;

    bgPlanner.requestPlan(request);
}


void PlanningTask::updateFromPlan(PlanRequest solvedRequest) {


    if (!solvedRequest.isSuccess()) {
        cout << "Cannot update using plan: request status failed" << endl;
        return;
    }

    if (hasWaypoints()) {
        Point next = getCurrentWaypoint();
        waypoints.clear();
        waypoints.emplace_back(next);
    }
    /*
    if (!solvedRequest.result.empty()) {
        Point last_wp = local_map->convertToGlobalFrame(solvedRequest.result.back());
        cout << "Last wp reached: " << last_wp << endl;
        if (isWaypointReached(last_wp) || isPointPassed(last_wp)) {
            onSubGoalReached(solvedRequest.goal);
            return;
        }
    }*/


    int wp_count = 0;
    Point rear = car_model->getRearAxis();
    for (auto &wp : solvedRequest.result) {
        wp_count++;

        Point globalWp = local_map->convertToGlobalFrame(wp);
        if (isWaypointReached(globalWp)  || isPointPassed(rear, globalWp) || wp_count%2 == 0) {
            continue;
        }

        waypoints.emplace_back(globalWp);
    }

    LOG_INFO("Waypoint count: %d | Available: %d", waypoints.size(), solvedRequest.result.size());
    if (!hasWaypoints()) {
        onComputeLocalGoal();
    }

    update_plan_watch.beginMeasure();

}

void PlanningTask::onLocalGoalReached(const SubGoal &goal) {
    LOG_INFO("Local goal reached: %f, %f", goal.point.getX(), goal.point.getY());
    if (!local_goals.empty()) {
        local_goals.pop_front();
    }
    if (local_goals.empty()) {
        SubGoal last = goal;
        updateLocalGoal(&last, nullptr);
    }

}


void PlanningTask::onCrossingSign(Marker *marker, float dist_m) {
    BaseTask::onCrossingSign(marker, dist_m);


}

void PlanningTask::onStopSign(Marker *marker, float dist_m) {
    BaseTask::onStopSign(marker, dist_m);
}


void PlanningTask::onStraightOnly(Marker *marker, float dist_m) {
    BaseTask::onStraightOnly(marker, dist_m);
}


/**Intercepts to parent class*/

void PlanningTask::onEmptyWaypoints() {
    BaseTask::onEmptyWaypoints();

}

void PlanningTask::onLaneDataAvailable(vector<Lane> &laneData) {
    BaseTask::onLaneDataAvailable(laneData);
}

void PlanningTask::onObstacleDetected(Obstacle &obstacle) {
    BaseTask::onObstacleDetected(obstacle);

}


void PlanningTask::onLocalMapAvailable(shared_ptr<LocalMap> &local_map) {
    BaseTask::onLocalMapAvailable(local_map);
    bgPlanner.setLocalMap(local_map);
}

void PlanningTask::onGlobalMapAvailable(IGlobalMap *global_map) {
    BaseTask::onGlobalMapAvailable(global_map);
}

void PlanningTask::onPlannerAvailable(IPlanner *planner) {
    BaseTask::onPlannerAvailable(planner);
    bgPlanner.setPlanner(planner);
}

Point PlanningTask::getWaypointProximityCheckCenter() {
    return car_model->getRearAxis();
}

float PlanningTask::getWaypointProximity() {
    return WP_PROX;
}

float PlanningTask::getGoalProximity() {
    return GOAL_PROX;
}














