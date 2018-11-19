
#include "LaneFollowingPerceptionTask.h"

float cvPoint2fDistance(cv::Point2f a, cv::Point2f b) {
    return sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
}

int cvPoint2iDistance(cv::Point2i a, cv::Point2i b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

LaneFollowingPerceptionTask::LaneFollowingPerceptionTask(int id, bool exit_on_parking, Point goal) :
SubGoalTask(id, BaseTask::Type::straight, -1, goal), exit_on_parking(exit_on_parking) {
    this->goalpoint_extractor =
        DijkstraGoalpointExtractor(SEGMENTATION_CHANNEL,
                                   COST_CHANNEL,
                                   PLANNING_GOAL_RADIUS,
                                   START_POSITION_Y,
                                   SELECT_EACH_XTH_WAYPOINT,
                                   INTERSECTION_REGION,
                                   INTERSECTION_THRESHOLD,
                                   DISTANCE_TO_INTERSECTION,
                                   POINT_SHIFT_X);
}

void LaneFollowingPerceptionTask::logDebug() {

}
void LaneFollowingPerceptionTask::onSubGoalReached(const SubGoal &goal) {
    if (lane_tracker.isAtPoi()) {
        goal_reached = true;
    } else {
        // plan to next subgoal here
        // planWaypoints(car_model->getRearAxis(), goal, true);

        SubGoalTask::onSubGoalReached(goal);
    }
}

SubGoal LaneFollowingPerceptionTask::generateSubGoal(SubGoal *last) {
    if (!lane_tracker.hasWp()) {
       lane_tracker.localize(global_map);
    } else {
        lane_tracker.next(global_map, .1);
    }

    SubGoal g = SubGoal(lane_tracker.getMapWaypoint(), true);
    // LOG_INFO("Subgoal generated: %f %f (type: %d)", g.point.getX(), g.point.getY(), g.getMapObj().type);

    // planWaypointsWrapper();
    return g;
}

void LaneFollowingPerceptionTask::updateCachedLanes(Lane *left, Lane *right) {


}

void LaneFollowingPerceptionTask::doReverseEmBrakeRecovery() {
    waypoints.clear();
    BaseTask::doReverseEmBrakeRecovery();
}


bool LaneFollowingPerceptionTask::onNoGlobalGoalAvailable(){
    return true;
}

void LaneFollowingPerceptionTask::onBeforeUpdate() {
    SubGoalTask::onBeforeUpdate();
}


void LaneFollowingPerceptionTask::onEmptyWaypoints() {
    SubGoalTask::onEmptyWaypoints();
}




void LaneFollowingPerceptionTask::onLaneDataAvailable(vector<Lane> &laneData) {

}


void LaneFollowingPerceptionTask::planWaypointsWrapper() {
    Point globalConversionAnchor = car_model->getRearAxis();
    SubGoal* current_subgoal = getCurrentSubGoal();
    float planner_lookahead = 1.0;
    auto planner_goal_map= lane_tracker.nextPlanningPoint(global_map, planner_lookahead);
    if (planner_goal_map.type == mapobjects::WaypointType::ERROR) {
        LOG_ERROR("NextPlanningPoint returned type error");
        return;
    }
    SubGoal planner_goal_global = SubGoal(planner_goal_map, true);

    if (!isValidSubGoal(current_subgoal)) {
        LOG_INFO("subgoal is not valid");
        planWaypoints(globalConversionAnchor, planner_goal_global, false, false);
    } else {
        // cv::Point2f planning_start = cv::Point2f(local_map->localCarPosition.getX(),
        //                                                 local_map->localCarPosition.getY())
        //                         + car_model->getLocalBeyeAnchor(Coordinate::GLOBAL) + cv::Point2f(0, START_POSITION_Y);
        // if (planner_goal_map.type == mapobjects::WaypointType::JUNCTION
        // && cvPoint2fDistance(local_map->convertFromGlobalToLocalMapFrame(planner_goal_global.point),
        //                      planning_start) < JUNCTION_FALLBACK_DISTANCE) {
        if (planner_goal_map.type == mapobjects::WaypointType::JUNCTION
                && cvPoint2fDistance(local_map->convertFromGlobalToLocalMapFrame(planner_goal_global.point),
                                     local_map->convertFromGlobalToLocalMapFrame(car_model->getFrontAxis()))
                < PPS_LOOKAHEAD_RADIUS + PLANNING_GOAL_RADIUS + 1) {
            disable_goal_radius = true;
        } else {
            disable_goal_radius = false;
        }
        // plan to global goal
        planWaypoints(globalConversionAnchor, planner_goal_global, true, disable_goal_radius);
    }
}


/* trigger waypoint generation via planner here */
void LaneFollowingPerceptionTask::planWaypoints(
    Point globalConversionAnchor, SubGoal goal, bool plan_to_goal, bool disable_goal_radius) {

    // convert goal position to local_map pixel frame
    this->planning_goal = local_map->convertFromGlobalToLocalMapFrame(goal.point);
    this->planning_goal_subgoal = goal;

    // LOG_INFO("goal point [global] %f %f", goal.point.getX(), goal.point.getY());
    // LOG_INFO("goal point [local] %d %d", planning_goal.x, planning_goal.y);

    vector<cv::Point> new_waypoints;
    cv::Mat internal_planner_image = cv::Mat(
        local_map->channel_perception.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    float pixel_per_cm = round(PIXEL_PER_METER / 100);
    cv::Point2f local_car_position = cv::Point2f(local_map->localCarPosition.getX(),
                                                    local_map->localCarPosition.getY());
    cv::Point2f beye_anchor_local = local_car_position \
        + car_model->getLocalBeyeAnchor(PIXEL_PER_METER);

    // if (planning_goal.y < (beye_anchor_local.y + START_POSITION_Y + DISCARD_WAYPOINT_RADIUS)) {  // TODO remove this block and this condition
    //     // generate junction waypoints (go straight)
    //     for (int i = 0; i < 10; i++) {
    //         new_waypoints.push_back(cv::Point(0, i * 10) + cv::Point(beye_anchor_local) + cv::Point(0, START_POSITION_Y));
    //     }
    // } else {
    this->fallback_planning = goalpoint_extractor.getGoalPointsToGoal(
                                            local_map->channel_perception,
                                            local_map->channel_segmentation,
                                            &new_waypoints,
                                            true,
                                            &internal_planner_image,
                                            0,
                                            beye_anchor_local,
                                            planning_goal,
                                            plan_to_goal,
                                            disable_goal_radius);
    // }

    // planner_watch.print_measurement("planner runtime");

    // cv::Mat debug_perception;
    // local_map->channel_perception.convertTo(debug_perception, CV_8UC1, 255);
    // cv::imwrite("/home/aadc/Desktop/debug_perception.png", debug_perception);
    // cv::imwrite("/home/aadc/Desktop/debug_image_planner.png", internal_planner_image);
    waypoints.clear();
    for (cv::Point elem : new_waypoints) {
        // convert waypoints to global frame
        cv::Point2f elem_local = cv::Point2f(static_cast<float>(elem.x),
                                            static_cast<float>(elem.y));

        cv::Point2f planning_start_local = beye_anchor_local +
                                            cv::Point2f(0, START_POSITION_Y);
        if (sqrtf(powf(elem_local.x - planning_start_local.x, 2) + powf(elem_local.y - planning_start_local.y, 2)) < DISCARD_WAYPOINT_RADIUS) {
            continue;
        }

        elem_local -= local_car_position;  // reference: rear axis middle -> global position anchor
        elem_local /= pixel_per_cm;  // px -> cm (global unit)
        //LOG_INFO("waypoint [local map frame]: %f, %f", elem_local.x, elem_local.y);
        //elem_local.y *= -1;

        cv::Point2f car_local = cv::Point2f(-elem_local.x, elem_local.y);
        //LOG_INFO("waypoint [car frame]: %f, %f", car_local.x, car_local.y);

        globalConversionAnchor.toGlobalCV(car_local, car_model->getHeading());
        //LOG_INFO("waypoint [global]: %f, %f", car_local.x, car_local.y);

        Point elem_global = Point::Global(car_local.x, car_local.y);


        waypoints.push_front(elem_global);
        //
        // Filter waypoints close to start, add to waypoint list
        //
        // version 1:
        // if (elem_global.distanceTo(car_model->getBirdseyeAnchor()) > DISCARD_WAYPOINT_RADIUS
        //        || goal.getMapObj().type == mapobjects::WaypointType::JUNCTION) {
        //     waypoints.push_front(elem_global);
        // }
        // version 2: see a few lines earlier continue
    }

    local_map->planner_image = internal_planner_image;
}

void LaneFollowingPerceptionTask::addPerceptionAsWaypoints(vector<Point> *src,
                                                           Point globalConversionAnchor,
                                                           bool scale) {
    // bool do_conversion = !globalConversionAnchor.isInitPoint();

    // for (auto elem : *src) {


    //     if (scale) {
    //         elem.scaleBy(Coordinate::Type::GLOBAL);
    //     }


    //     if (do_conversion) {
    //         //LOG_INFO("Local: %f, %f", elem.getX(), elem.getY());
    //         globalConversionAnchor.toGlobalNoCopy(elem, car_model->getHeading());
    //         //LOG_INFO("Global Perc.: %f, %f", elem.getX(), elem.getY());
    //         waypoints.push_front(elem);
    //     } else {
    //         waypoints.push_front(elem);
    //     }
    // }
}


void LaneFollowingPerceptionTask::onLanePointDataAvailable(std::vector<Point> *input) {
    BaseTask::onLanePointDataAvailable(input);

    // waypoints.clear();

    // Point back = car_model->getRearAxis();
    // addPerceptionAsWaypoints(input, back, true);
    planWaypointsWrapper();
}

float LaneFollowingPerceptionTask::getDrivingSpeed() {
    return Speed::SLOW;
}


void LaneFollowingPerceptionTask::onObstacleDetected(Obstacle &obstacle) {
    SubGoalTask::onObstacleDetected(obstacle);
}


void LaneFollowingPerceptionTask::onStartTask() {
    SubGoalTask::onStartTask();

    // point_passed_offset_tolerance = -10;

    // addMarkerTrigger(Marker::MarkerType::ZEBRA, 1.0);
    setSubGoalProximity(SUBGOAL_PROXIMITY);
    LOG_INFO("Lane following starting");

}


void LaneFollowingPerceptionTask::onParkSign(Marker *marker, float dist_m) {
    BaseTask::onParkSign(marker, dist_m);
    if (exit_on_parking) {
        LOG_INFO("Parking sign detected, finishing lane following");
        goal_reached = true;
    }
}

void LaneFollowingPerceptionTask::onStraightOnly(Marker *marker, float dist) {
    SubGoalTask::onStraightOnly(marker, dist);
}

void LaneFollowingPerceptionTask::onStopSign(Marker *marker, float dist) {
    SubGoalTask::onStopSign(marker, dist);
}

void LaneFollowingPerceptionTask::onCrossingSign(Marker *marker, float dist) {
    SubGoalTask::onCrossingSign(marker, dist);
}


Point &LaneFollowingPerceptionTask::getCurrentWaypoint() {
    if (waypoints.empty()) {
        throw WaypointsEmptyException();
    }
    int i = waypoints.size() - 1;
    Point* pps_goal_point = &waypoints[i];
    while (pps_goal_point->distanceTo(car_model->getFrontAxis()) > PPS_LOOKAHEAD_RADIUS) {
        i--;
        if (i >= 0) {
            pps_goal_point = &waypoints[i];
        } else {
            break;
        }
    }
    m_pps_goalpoint = *pps_goal_point;
    return *pps_goal_point;
}


void LaneFollowingPerceptionTask::writeDebugOutputToLocalMap(cv::Mat *local_map_img) {
    SubGoalTask::writeDebugOutputToLocalMap(local_map_img);

    // planning goal radius
    if (!disable_goal_radius) {
        cv::circle(*local_map_img, this->planning_goal, PLANNING_GOAL_RADIUS, cv::Scalar(255, 0, 0));
    }
    // planning point
    if (!(planning_goal_subgoal.getMapObj().type == mapobjects::WaypointType::JUNCTION)) {
        cv::circle(*local_map_img, this->planning_goal, 2, cv::Scalar(255, 0, 0), -1);
    } else {
        cv::circle(*local_map_img, this->planning_goal, 2, cv::Scalar(255, 100, 0), -1);
    }

    // planning start
    cv::Point2f planning_start = cv::Point2f(local_map->localCarPosition.getX(),
                                                    local_map->localCarPosition.getY())
                            + car_model->getLocalBeyeAnchor(Coordinate::GLOBAL) + cv::Point2f(0, START_POSITION_Y);

    // waypoint discard circle (when new waypoints are generated, NOT isWaypointReached)
    cv::circle(*local_map_img, planning_start, DISCARD_WAYPOINT_RADIUS, cv::Scalar(255, 100, 0));

    // // lane following fallback  // commented out for better visibility
    // cv::circle(*local_map_img,
    //             planning_start,
    //             JUNCTION_FALLBACK_DISTANCE,
    //             cv::Scalar(0, 150, 0));

    // pps_goalpoint
    cv::Point center = local_map->convertFromGlobalToLocalMapFrame(car_model->getFrontAxis());
    cv::circle(*local_map_img, center, PPS_LOOKAHEAD_RADIUS, cv::Scalar(0, 255, 0));
    cv::Point cv_m_pps_goalpoint = local_map->convertFromGlobalToLocalMapFrame(m_pps_goalpoint);
    cv::circle(*local_map_img, cv_m_pps_goalpoint, 2, cv::Scalar(0, 255, 0));

    if (fallback_planning) {
        cv::rectangle(
            *local_map_img,
            cv::Point2i(305, 265),
            cv::Point2i(395, 280),
            cv::Scalar(255, 0, 0, 255),
            -1);
        cv::putText(*local_map_img,
            cv::String("! LF-Fallback"),
            cv::Point2f(305, 278),
            cv::FONT_HERSHEY_PLAIN,
            0.8,
            cv::Scalar(0, 0, 0, 255));
    }
}


const Point LaneFollowingPerceptionTask::getSteeringAnchor() {
    return car_model->getRearAxis().toGlobal(Point::Local(0, 13), car_model->getHeading());  // was 18
    // return car_model->getRearAxis();
    // return car_model->getFrontAxis();
}

void LaneFollowingPerceptionTask::onUpdateProximityCenter(Point new_center) {
    BaseTask::onUpdateProximityCenter(new_center);
    // if (isValidSubGoal(getCurrentSubGoal())
    //     && getCurrentSubGoal()->getMapObj().type
    //     == mapobjects::WaypointType::JUNCTION) {

    //     sub_goal_check_center_global = car_model->getRearAxis();
    //     return;
    // }
    sub_goal_check_center_global = new_center;
}

Point LaneFollowingPerceptionTask::getWaypointProximityCheckCenter() {
    // return car_model->getFrontAxis().toGlobal(Point::Local(0, 12), car_model->getHeading());
    return car_model->getBirdseyeAnchor();
}

float LaneFollowingPerceptionTask::getWaypointProximity() {
    return sub_goal_proximity_scaled;
}

float LaneFollowingPerceptionTask::getGoalProximity() {
    return GOAL_PROX;
}

// waypoint check
bool LaneFollowingPerceptionTask::isWaypointReached(Point &position, Point &waypoint) {
    // currently not used as we always plan new, filter wrong points
    // and make sure that points are never reached.
    // this condition influences steering behaviour because it influences
    // the distance between steering-anchor and first waypoint.
    auto heading = car_model->getHeading();
    Point local = position.toLocal(waypoint, heading);
    auto wp_proximity = getWaypointProximity();
    return position.distanceTo(waypoint) < wp_proximity && local.getY() < 0;
}

// subgoal check
bool LaneFollowingPerceptionTask::isSubGoalReached(const Point &position) {
    Point local = position.toLocal(getCurrentSubGoal()->point, car_model->getHeading());
    bool subgoal_reached = position.distanceTo(getCurrentSubGoal()->point) < sub_goal_proximity_scaled
        && local.getY() < 0;
    return subgoal_reached;
}


std::string LaneFollowingPerceptionTask::getName() {
    return "LFP";
}

void LaneFollowingPerceptionTask::onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) {
    SubGoalTask::onSubGoalChanged(old_sg, new_sg);

    bool new_sg_is_ramp_up = new_sg.getMapObj().lp.lane_info.is_ramp_up;
    bool new_sg_is_ramp_down = new_sg.getMapObj().lp.lane_info.is_ramp_down;
    bool new_sg_is_on_ramp = !new_sg.getMapObj().lp.lane_info.is_ground;

    /*if (old_sg.getMapObj().lp.lane_info.is_ramp_up != new_sg.getMapObj().lp.lane_info.is_ramp_up) {
        bool enable_em_brake = !new_sg.getMapObj().lp.lane_info.is_ramp_up;
        em_brake_ctrl->setEmergencyBrakeEnabled(enable_em_brake);
        LOG_INFO("Emergency brake enabled: %d", enable_em_brake);
    }*/

    if (new_sg_is_ramp_up || new_sg_is_ramp_down || new_sg_is_on_ramp) {
        if (em_brake_ctrl->isEnabled()) {
            LOG_INFO("Ramp mode, em brake disabled");
            em_brake_ctrl->setEmergencyBrakeEnabled(false);
        }
    } else if (!em_brake_ctrl->isEnabled()) {
        LOG_INFO("Enabled em brake");
        em_brake_ctrl->setEmergencyBrakeEnabled(true);
    }
}