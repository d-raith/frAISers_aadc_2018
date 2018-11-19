#include <sstream>

#include "StateMachineCtrl.h"

StateMachineCtrl::StateMachineCtrl(IControlOutput *output, IGlobalMap *map,
                                   ILightCtrl *lightCtrl) :
        map(map),
        local_map(std::make_shared<LocalMap>(3, 3)),

        filter_output(output),
        light_ctrl(lightCtrl) {
    local_map->init();
    planner = new AStarPlanner(10, *local_map, 0.05);

    pool = std::make_shared<ThreadPool>(1);
    pool->init();
}

StateMachineCtrl::~StateMachineCtrl() {
    pool->shutdown();
    pool.reset();
    local_map.reset();
}

void StateMachineCtrl::configure() {
    max_speed_override.reset(new MaxSpeedOverride(MAX_SPEED));
}

/** Provide required interfaces to new active task **/

void StateMachineCtrl::setTask(shared_ptr<BaseTask> &task) {
    if (current_task) {
        task->setLaneTracker(current_task->getLaneTracker());
    }

    current_task = task;
    current_task->setSteerCtrl(this);
    current_task->setVelocityCtrl(this);
    current_task->setLightCtrl(light_ctrl);
    current_task->setEmergencyBrakeCtrl(em_brake_override.get());
    current_task->setGlobalMap(map);
    current_task->setLocalMap(local_map);
    current_task->setCarModel(car_model);
    current_task->setPlannerInstance(planner);
    current_task->setEnvironmentState(environment_state);
    task_changed = true;
}

void resetLights(ILightCtrl *l_ctrl) {
    l_ctrl->setHazardLightsEnabled(false);
    l_ctrl->setIndicatorLeftEnabled(false);
    l_ctrl->setIndicatorRightEnabled(false);
    l_ctrl->setBrakeLightsEnabled(false);
}

/** Control loop executed upon position update, delegates control to underlying task. **/
void StateMachineCtrl::run() {

    update_count++;


    if (task_changed) {
        //resetLights(light_ctrl);
        em_brake_override->setEmergencyBrakeEnabled(true);
        task_changed = false;
        LOG_INFO("task changed: type: %d", current_task->getType());
        current_task->onStartTask();
        filter_output->taskStarted(current_task.get());
        local_map_debug_timer.beginMeasure();
    }

    if (current_task->isError()) {
        LOG_INFO("Task id %d completed with status error", current_task->getId());
        setVehicleSpeed(Speed::STOP);
        filter_output->taskFailed(current_task.get());
        return;
    }


    if (current_task->isOvertakeRequested()) {
        LOG_INFO("Task id %d requested overtaking maneuver", current_task->getId());
        filter_output->requestOvertake(current_task.get());
    } else if (current_task->isFinished()) {
        LOG_INFO("Task id %d finished", current_task->getId());
        filter_output->taskFinished(current_task.get());
        return;
    } else {
        current_task->logDebug();
        current_task->update(&current_speed);
    }

    // additional call to ensure overrides are notified
    setVehicleSpeed(current_speed);
    if (local_map_debug_timer.didMillisecondsPass(50)) {
        if (filter_output->isLocalMapBufferEmpty() && (!pool->hasJobs())) {
            // StateMachineCtrl::writeDebugOutputLocalMap();
            auto wps = current_task->getWaypoints();
            auto func = std::bind(&StateMachineCtrl::writeDebugOutputLocalMap, this, wps,
                    *car_model.get());
            pool->submit(func);
            local_map_debug_timer.beginMeasure();
        }
    }

}


/** Outgoing control signals **/

bool StateMachineCtrl::setVehicleSpeed(const float speed) {


    float desired_speed = speed;
    if (em_brake_override->onOverrideSpeed(&desired_speed)) {
        //LOG_INFO("EM_BRAKE_OVERRIDE_SPEED");
    }
    if (max_speed_override->onOverrideSpeed(&desired_speed)) {
        // LOG_INFO("MAX_SPEED_OVERRIDE_SPEED");
    }
    if (desired_speed != speed) {
        //LOG_INFO("Speed override detected: old/new: %f | %f", current_speed, desired_speed);
    }
    if (desired_speed != current_speed) {
        LOG_INFO("Speed changed: curr/new: %f | %f", current_speed, desired_speed);
        if (desired_speed < current_speed
            || desired_speed == Speed::STOP) {
            light_ctrl->setBrakeLightsEnabled(true);
        } else {
            light_ctrl->setBrakeLightsEnabled(false);
        }
        if (filter_output->transmitSpeedSignal(desired_speed)) {
            current_speed = desired_speed;
            return true;
        }
    }

    return false;

}

bool StateMachineCtrl::setCurvature(float curv_value) {
    /*if (lane following){
        steering = -50.96 * curv_value ;
    }else{
        steering = -64.84738*curv_value + 3.581326 *curv_value*curv_value*curv_value;
    } */
    em_brake_override->updateSteeringValue(curv_value);
    environment_state->updateSteeringValue(curv_value);
    return filter_output->transmitCurvature(curv_value);
}

template<typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}


void draw_rotated_rectangle(cv::Mat* img,
                            cv::RotatedRect rRect,
                            cv::Scalar color,
                            int thickness = 1) {
    cv::Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++) {
        cv::line(*img, vertices[i], vertices[(i+1)%4], color, thickness);
    }
}

void StateMachineCtrl::writeDebugOutputLocalMap(const deque<Point> &waypoints, CarModel car) {
    // cv::Mat localMapImg = cv::Mat(local_map->window_height, local_map->window_width,
    //                                          CV_8UC3,
    //                                          cv::Scalar(0, 0, 0));
    cv::Mat localMapImg = local_map->toSoftMaxedCellProbImgMatrix(  // not really a softmax any more
            current_task->getWaypointProximity());

    const float pixel_per_meter = 100;

    // get car anchors
    cv::Point2f local_car_position = cv::Point2f(local_map->localCarPosition.getX(),
                                                 local_map->localCarPosition.getY());
    cv::Point2f rear_axis = local_car_position;
    cv::Point2f front_axis = rear_axis + car.getLocalFrontAxis(pixel_per_meter);
    cv::Point2f camera = rear_axis + car.getLocalCamera(pixel_per_meter);
    cv::Point2f wheel_left = rear_axis + car.getLocalFrontAxis(pixel_per_meter) \
 + car.getLocalWheelLeft(pixel_per_meter);
    cv::Point2f wheel_right = rear_axis + car.getLocalFrontAxis(pixel_per_meter) \
 + car.getLocalWheelRight(pixel_per_meter);
    cv::Point2f beye_anchor_local = rear_axis + car.getLocalBeyeAnchor(pixel_per_meter);


    const tUInt8 INTERSECTION = 9;
    for (int i = 0; i != local_map->channel_segmentation.size().height; i++) {
        for (int j = 0; j != local_map->channel_segmentation.size().width; j++) {
            if (local_map->channel_segmentation.at<tUInt8>(cv::Point(j, i)) == INTERSECTION) {
                localMapImg.at<cv::Vec3b>(cv::Point(j, i)) = INTERSECTION_COLOR;
            }
        }
    }

    // raw image
    if (!birdseye_data.empty()) {
        // std::unique_lock<std::mutex> lock(birdseye_data_mutex);
        try {
            // top mid of the incoming image should end up here
            int target_roi_offset_width = beye_anchor_local.x - birdseye_data.size().width / 2;
            int target_roi_offset_height = beye_anchor_local.y;
            int target_roi_width = birdseye_data.size().width;
            int target_roi_height = birdseye_data.size().height;

            // std::cout << "___ROI check___\no_w: " << target_roi_offset_width << " o_h: "
            //     << target_roi_offset_height << " w: " << target_roi_width << " h: "
            //     << target_roi_height << std::endl;

            // make sure that roi is not out of bound!
            cv::Rect target_roi = cv::Rect(target_roi_offset_width,
                                           target_roi_offset_height,
                                           target_roi_width,
                                           target_roi_height);
            cv::Mat birdseye_fullsize = cv::Mat(localMapImg.size(), CV_8UC3, cv::Scalar(0, 0, 0));
            if (birdseye_data.size() != birdseye_fullsize(target_roi).size()) {
                std::cout << "StateMachineCtrl wirteDebugOutput: sizes dont match before copyTo. "
                          << std::endl;
                std::cout << "birdseye_data.size()=" << birdseye_data.size() <<
                          " birdseye_fullsize(target_roi).size()="
                          << birdseye_fullsize(target_roi).size() << std::endl;
            }
            birdseye_data.copyTo(birdseye_fullsize(target_roi));

            cv::addWeighted(birdseye_fullsize, 0.5,
                            localMapImg, 0.5,
                            0,
                            localMapImg);
        } catch (cv::Exception ex) {
            cout << "catched cv exception while printing debug output" << endl << ex.what() << endl;
        }
        // lock.unlock();
    }



    // cv::Point2f wp_prox_local = local_map->convertFromGlobalToLocalMapFrame(current_task->getProximityCheckCenterGlobal());
    // cv::circle(localMapImg, wp_prox_local, static_cast<int>(ceil
    //                    (current_task->getWaypointProximity())),
    //            cv::Scalar(255, 0, 0));

    // cv::ellipse(localMapImage, sub_goal_check_center_localmap,
    //     cv::Size(sub_goal_proximity_scaled, sub_goal_proximity_scaled),
    //     0, 180, 360, cv::Scalar(255 , 255, 255), 1);

    Point goal = current_task->getCurrentGoalPoint();

    // print the waypoints
    if (!waypoints.empty()) {
        // MatUtils::interpolatePoints(localMapImg, &waypoints);
        for (auto wp : waypoints) {
            cv::Point wp_local = local_map->convertFromGlobalToLocalMapFrame(wp);

            if (MatUtils::inBoundaries(localMapImg, wp_local.x, wp_local.y)) {
                // localMapImg.at<cv::Vec3b>(wp_local) = LOCALMAP_WAYPOINT_COLOR;
                cv::circle(localMapImg, wp_local, 1, LOCALMAP_WAYPOINT_COLOR, -1);
            } else {
                LOG_INFO("WP: %d %d not in boundaries", wp_local.x, wp_local.y);
            }
        }

        cv::Point goal_local = local_map->convertFromGlobalToLocalMapFrame(goal);
        if (MatUtils::inBoundaries(localMapImg, goal_local.x, goal_local.y)) {
            cv::circle(localMapImg, goal_local, 2, cv::Scalar(0, 255, 0), -1);
            // localMapImg.at<cv::Vec3b>(goal_local) = cv::Vec3b(0, 255, 0);
        }
    }

    // draw car
    cv::circle(localMapImg, local_car_position, 2, CAR_COLOR);
    cv::circle(localMapImg, front_axis, 2, CAR_COLOR);
    cv::circle(localMapImg, camera, 2, CAR_COLOR);
    cv::circle(localMapImg, wheel_left, 2, CAR_COLOR);
    cv::circle(localMapImg, wheel_right, 2, CAR_COLOR);


    // plain laser scan points
    std::vector<Point> laser_scan = local_map->get_plainLS_obstacle_list();
    for (unsigned int i = 0; i < laser_scan.size(); i++) {
        Point laser_scan_point = laser_scan[i];
        cv::Point local_laser_scan_point =
            local_map->convertFromGlobalToLocalMapFrame(laser_scan_point);
        if (0 <= local_laser_scan_point.x
                && localMapImg.size().width >  local_laser_scan_point.x
                && 0 <= local_laser_scan_point.y
                && localMapImg.size().width >  local_laser_scan_point.y) {
            localMapImg.at<cv::Vec3b>(local_laser_scan_point)
                = cv::Vec3b(255, 255, 255);
        }
    }


    // obstacles
    std::vector<Obstacle> obstacle_list;
    environment_state->getObstacles(4, &obstacle_list);  // cars
    for (unsigned int i = 0; i < obstacle_list.size(); i++) {
        Obstacle obstacle = obstacle_list[i];
        cv::Point local_obstacle =
            local_map->convertFromGlobalToLocalMapFrame(obstacle);
        if (0 <= local_obstacle.x
                && localMapImg.size().width >  local_obstacle.x
                && 0 <= local_obstacle.y
                && localMapImg.size().width >  local_obstacle.y) {
            // object width and height are unfortunatly not comparable before and after birdseye.
            // cv::RotatedRect rect =
            //     cv::RotatedRect(cv::Point2f(local_obstacle.x
            //                                     + (obstacle.getExpY() / 2)
            //                                     * (-1.0) * sin(obstacle.getHeading() * DEG2RAD),
            //                                 local_obstacle.y
            //                                     + (obstacle.getExpY() / 2)
            //                                     * cos(obstacle.getHeading() * DEG2RAD)),
            //                     cv::Size2f(obstacle.getExpX(),
            //                                obstacle.getExpY()),
            //                     obstacle.getHeading());
            // draw_rotated_rectangle(&localMapImg, rect, cv::Scalar(255, 0, 0));
            cv::circle(localMapImg, local_obstacle, 2, cv::Scalar(255, 0, 0), 2);
        }
    }
    obstacle_list.clear();
    environment_state->getObstacles(13, &obstacle_list);  // child
    for (unsigned int i = 0; i < obstacle_list.size(); i++) {
        Obstacle obstacle = obstacle_list[i];
        cv::Point local_obstacle =
            local_map->convertFromGlobalToLocalMapFrame(obstacle);
        if (0 <= local_obstacle.x
                && localMapImg.size().width >  local_obstacle.x
                && 0 <= local_obstacle.y
                && localMapImg.size().width >  local_obstacle.y) {
            // object width and height are unfortunatly not comparable before and after birdseye.
            // cv::RotatedRect rect =
            //     cv::RotatedRect(cv::Point2f(local_obstacle.x
            //                                     + (obstacle.getExpY() / 2)
            //                                     * (-1.0) * sin(obstacle.getHeading() * DEG2RAD),
            //                                 local_obstacle.y
            //                                     + (obstacle.getExpY() / 2)
            //                                     * cos(obstacle.getHeading() * DEG2RAD)),
            //                     cv::Size2f(obstacle.getExpX(),
            //                                obstacle.getExpY()),
            //                     obstacle.getHeading());
            // draw_rotated_rectangle(&localMapImg, rect, cv::Scalar(255, 255, 0));
            cv::circle(localMapImg, local_obstacle, 2, cv::Scalar(255, 255, 0), 2);
        }
    }
    obstacle_list.clear();
    environment_state->getObstacles(20, &obstacle_list);  // adult
    for (unsigned int i = 0; i < obstacle_list.size(); i++) {
        Obstacle obstacle = obstacle_list[i];
        cv::Point local_obstacle =
            local_map->convertFromGlobalToLocalMapFrame(obstacle);
        if (0 <= local_obstacle.x
                && localMapImg.size().width >  local_obstacle.x
                && 0 <= local_obstacle.y
                && localMapImg.size().width >  local_obstacle.y) {
            // object width and height are unfortunatly not comparable before and after birdseye.
            // cv::RotatedRect rect =
            //     cv::RotatedRect(cv::Point2f(local_obstacle.x
            //                                     + (obstacle.getExpY() / 2)
            //                                     * (-1.0) * sin(obstacle.getHeading() * DEG2RAD),
            //                                 local_obstacle.y
            //                                     + (obstacle.getExpY() / 2)
            //                                     * cos(obstacle.getHeading() * DEG2RAD)),
            //                     cv::Size2f(obstacle.getExpX(),
            //                                obstacle.getExpY()),
            //                     obstacle.getHeading());
            // draw_rotated_rectangle(&localMapImg, rect, cv::Scalar(150, 150, 0));
            cv::circle(localMapImg, local_obstacle, 2, cv::Scalar(255, 165, 0), 2);
        }
    }
    // draw dividor and range limit for car-left/car-right/car-ahead
    int obstacle_detection_distance = 200;
    cv::Point hitcount_dividor_start =
        local_map->convertFromGlobalToLocalMapFrame(car.getFrontAxis());
    cv::Point hitcount_dividor_leftend =
        hitcount_dividor_start + cv::Point(sin(15 * DEG2RAD) * obstacle_detection_distance,
                                           cos(15 * DEG2RAD) * obstacle_detection_distance);
    cv::Point hitcount_dividor_rightend =
        hitcount_dividor_start + cv::Point(sin(-15 * DEG2RAD) * obstacle_detection_distance,
                                           cos(-15 * DEG2RAD) * obstacle_detection_distance);
    cv::line(localMapImg,
             hitcount_dividor_start,
             hitcount_dividor_leftend,
             cv::Scalar(150, 150, 150),
             1);
    cv::line(localMapImg,
             hitcount_dividor_start,
             hitcount_dividor_rightend,
             cv::Scalar(150, 150, 150),
             1);
    cv::circle(localMapImg,
            hitcount_dividor_start,
            obstacle_detection_distance,
            cv::Scalar(150, 150, 150),
            1);


    //overtaking
    cv::circle(localMapImg,
               local_map->convertFromGlobalToLocalMapFrame(car.getFrontAxis()),
               100,
               cv::Scalar(0, 150, 150),
               1);


    // emergency break  (optional also used for overtaking)
    if (em_brake_override) {
        // emergency break condition (circle arc) (non adaptive version)
        // cv::Point em_brake_center =
        // local_map->convertFromGlobalToLocalMapFrame(car.getFrontAxis());
        // float em_brake_distance = em_brake_override->getBreakDistanceCM();
        // cv::ellipse(localMapImg,
        //             em_brake_center,
        //             cv::Size(em_brake_distance,
        //                      em_brake_distance),
        //             0,
        //             65,
        //             115,
        //             cv::Scalar(255 , 255, 255),
        //             1);
        // cv::Point em_brake_leftend =
        //     em_brake_center + cv::Point(sin(25 * DEG2RAD) * em_brake_distance,
        //                                 cos(25 * DEG2RAD) * em_brake_distance);
        // cv::Point em_brake_rightend =
        //     em_brake_center + cv::Point(sin(-25 * DEG2RAD) * em_brake_distance,
        //                                 cos(-25 * DEG2RAD) * em_brake_distance);
        // cv::line(localMapImg,
        //         em_brake_center,
        //         em_brake_leftend,
        //         cv::Scalar(255, 255, 255),
        //         1);
        // cv::line(localMapImg,
        //         em_brake_center,
        //         em_brake_rightend,
        //         cv::Scalar(255, 255, 255),
        //         1);
        // end old version

        float em_brake_corridor_width = 20;
        float em_brake_distance_frontaxis = em_brake_override->getBreakDistanceCM();
        float steering_curvature = em_brake_override->getCurvatureSteering();
        cv::circle(localMapImg,
                   local_map->convertFromGlobalToLocalMapFrame(car.getFrontAxis()),
                   em_brake_distance_frontaxis,
                   cv::Scalar(255, 255, 255),
                   1);
        if (abs(steering_curvature) > 0.001) {
            float steering_radius = (1.0 / steering_curvature) * 100.0;
            Point steering_anchor = car.getFrontAxis();
            if (current_task) {
                steering_anchor = current_task->getSteeringAnchor();
            }

            cv::Point steering_center = local_map->convertFromGlobalToLocalMapFrame(steering_anchor)
                                            + cv::Point2i(steering_radius, 0);
            float dist_steeringanchor_frontaxis =
                steering_anchor.distanceTo(car.getFrontAxis());
            float em_brake_distance_steering = 100 + dist_steeringanchor_frontaxis;  // use a higher value just to make the viz nicer, correct is:  em_brake_distance_frontaxis + dist_steeringanchor_frontaxis;
            float circumfence = 2 * M_PI * abs(steering_radius);
            float em_brake_angle = (em_brake_distance_steering / circumfence) * 360;
            float ellipse_start_angle = 0;
            float ellipse_end_angle = 0;
            if (steering_radius < 0) {
                ellipse_start_angle = 0;
                ellipse_end_angle = em_brake_angle;
            } else {
                ellipse_start_angle = 180 - em_brake_angle;
                ellipse_end_angle = 180;
            }
            cv::ellipse(localMapImg,
                        steering_center,
                        cv::Size(abs(static_cast<int>(steering_radius)) - em_brake_corridor_width,
                                abs(static_cast<int>(steering_radius)) - em_brake_corridor_width),
                        0,
                        ellipse_start_angle,
                        ellipse_end_angle,
                        cv::Scalar(0, 0, 255),
                        1);
            cv::ellipse(localMapImg,
                        steering_center,
                        cv::Size(abs(static_cast<int>(steering_radius)) + em_brake_corridor_width,
                                abs(static_cast<int>(steering_radius)) + em_brake_corridor_width),
                        0,
                        ellipse_start_angle,
                        ellipse_end_angle,
                        cv::Scalar(0, 0, 255),
                        1);
        } else {
            Point steering_anchor = car.getFrontAxis();
            if (current_task) {
                steering_anchor = current_task->getSteeringAnchor();
            }
            float dist_steeringanchor_frontaxis =
                steering_anchor.distanceTo(car.getFrontAxis());
            cv::Point steering_anchor_local = local_map->convertFromGlobalToLocalMapFrame(steering_anchor);
            float em_brake_distance_steering = 100 + dist_steeringanchor_frontaxis;  // use a higher value just to make the viz nicer, correct is:  em_brake_distance_frontaxis + dist_steeringanchor_frontaxis;
            cv::line(localMapImg,
                     steering_anchor_local
                        + cv::Point(-em_brake_corridor_width, 0),
                     steering_anchor_local
                        + cv::Point(-em_brake_corridor_width, em_brake_distance_steering),
                    cv::Scalar(0, 0, 255),
                    1);
            cv::line(localMapImg,
                     steering_anchor_local
                        + cv::Point(+em_brake_corridor_width, 0),
                     steering_anchor_local
                        + cv::Point(+em_brake_corridor_width, em_brake_distance_steering),
                    cv::Scalar(0, 0, 255),
                    1);
        }
    }


    // new calibration pattern
    // cv::circle(localMapImg, front_axis + cv::Point2f(6, 29.4), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(6, 58.7), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(6, 88.2), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(6, 117.6), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(6, 147.0), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(6, 176.7), 2, cv::Scalar(0, 0, 255));

    // cv::circle(localMapImg, front_axis + cv::Point2f(-15, 29.4), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(-15, 58.7), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(-15, 88.2), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(-15, 117.6), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(-15, 147.0), 2, cv::Scalar(0, 0, 255));

    // cv::circle(localMapImg, front_axis + cv::Point2f(-36, 88.2), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(-36, 117.6), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(-36, 147.0), 2, cv::Scalar(0, 0, 255));

    // cv::circle(localMapImg, front_axis + cv::Point2f(27, 29.4), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(27, 58.7), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(27, 88.2), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(27, 117.6), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(27, 147.0), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(27, 176.7), 2, cv::Scalar(0, 0, 255));

    // cv::circle(localMapImg, front_axis + cv::Point2f(48, 117.6), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(48, 147.0), 2, cv::Scalar(0, 0, 255));
    // cv::circle(localMapImg, front_axis + cv::Point2f(48, 176.7), 2, cv::Scalar(0, 0, 255));
    // end new calibration pattern


    // birdseye anchor calibration helpers
    // cv::Point2f beye_calib_50 = front_axis +
    //                             cv::Point2f(0, 0.5 * pixel_per_meter);
    // cv::Point2f beye_calib_100 = front_axis +
    //                             cv::Point2f(0, 1.0 * pixel_per_meter);
    // cv::Point2f beye_calib_150 = front_axis +
    //                             cv::Point2f(0, 1.5 * pixel_per_meter);
    // cv::circle(localMapImg, beye_calib_50, 1, cv::Scalar(255, 255, 0));
    // cv::circle(localMapImg, beye_calib_100, 1, cv::Scalar(255, 255, 0));
    // cv::circle(localMapImg, beye_calib_150, 1, cv::Scalar(255, 255, 0));


    // frame border
    cv::rectangle(localMapImg, cv::Point(0, 0),
                  cv::Point(localMapImg.rows - 1, localMapImg.cols - 1),
                  cv::Scalar(255, 255, 255));

    //cv::resize(localMapImg, localMapImg, localMapImg.size()*100);

    //
    // increase the size of localMapImg to make space for more debug output
    //
    cv::Mat localMapImg_large = cv::Mat(localMapImg.size().height,
                                        localMapImg.size().width + 100,
                                        CV_8UC3,
                                        cv::Scalar(0, 0, 0));
    localMapImg.copyTo(localMapImg_large(cv::Rect(cv::Point(0, 0), localMapImg.size())));

    //
    // write position debug
    //
    std::string debug_pos_x =
            "x: " + to_string_with_precision(car.getRearAxis().getX() / 100, 3) + " m";
    std::string debug_pos_y =
            "y: " + to_string_with_precision(car.getRearAxis().getY() / 100, 3) + " m";
    std::string debug_heading =
            "r: " + to_string_with_precision(car.getHeading() * RAD2DEG, 0) + " deg";

    cv::putText(localMapImg_large,
                cv::String(debug_pos_x),
                cv::Point2f(localMapImg.size().width + 5, 15),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    cv::putText(localMapImg_large,
                cv::String(debug_pos_y),
                cv::Point2f(localMapImg.size().width + 5, 30),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    cv::putText(localMapImg_large,
                cv::String(debug_heading),
                cv::Point2f(localMapImg.size().width + 5, 45),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));

    //
    // write the active manouver on the image
    //
    std::string manouver_type_string = "Man: ";
    switch (current_task->getType()) {
        case (BaseTask::Type::undefined): {
            manouver_type_string += "undefined";
            break;
        }
        case (BaseTask::Type::turn_left): {
            manouver_type_string += "t_left";
            break;
        }
        case (BaseTask::Type::turn_right): {
            manouver_type_string += "t_right";
            break;
        }
        case (BaseTask::Type::straight): {
            manouver_type_string += "straight";
            break;
        }
        case (BaseTask::Type::parallel_parking): {
            manouver_type_string += "park_parallel";
            break;
        }
        case (BaseTask::Type::cross_parking): {
            manouver_type_string += "park_cross";
            break;
        }
        case (BaseTask::Type::pull_out_left): {
            manouver_type_string += "p_out_left";
            break;
        }
        case (BaseTask::Type::pull_out_right): {
            manouver_type_string += "p_out_right";
            break;
        }
        case (BaseTask::Type::merge_left): {
            manouver_type_string += "merge_left";
            break;
        }
        case (BaseTask::Type::merge_right): {
            manouver_type_string += "merge_right";
            break;
        }
        default: {
            manouver_type_string += "UNKNOWN ENUM";
            LOG_WARNING(
                    "unknown enum value for BaseTask::type in StateMachineCtrl.writeDebugOutput.");
        }
    }
    cv::putText(localMapImg_large,
                cv::String(manouver_type_string),
                cv::Point2f(localMapImg.size().width + 5, 70),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    //
    // write the active task on the image
    //
    std::string task_type_string = "Task: " + current_task->getName();
    cv::putText(localMapImg_large,
                cv::String(task_type_string),
                cv::Point2f(localMapImg.size().width + 5, 85),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));

    //
    // write the current sector and task id
    //
    std::string sector_type_string = "Sec: " +
        std::to_string(filter_output->getSectorFromTaskId(current_task->getId()));
    cv::putText(localMapImg_large,
                cv::String(sector_type_string),
                cv::Point2f(localMapImg.size().width + 5, 100),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    std::string id_type_string = "Id: " + std::to_string(current_task->getId());
    cv::putText(localMapImg_large,
                cv::String(id_type_string),
                cv::Point2f(localMapImg.size().width + 5, 115),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));


    //
    // add detection probabilities, junction sign and active stvo constraint
    //
    std::string debug_env_car_header = "Car";

    std::string debug_env_car_l = "< " +
            to_string_with_precision(environment_state->getDetInfo().f32pCarLeft, 2);
    std::string debug_env_car_a = "^ " + to_string_with_precision(environment_state->getDetInfo().f32pCarCenter, 2);
    // std::string debug_env_car_r = "> " + to_string_with_precision(environment_state->getDetInfo().f32pCarRight, 2);
    std::string debug_env_car_r = "^+ " + to_string_with_precision(environment_state->getCarInPlannedPathProb(), 2);

    std::string debug_env_person_adult = "Adult " + to_string_with_precision(environment_state->getDetInfo().f32pPerson, 2);
    std::string debug_env_person_child = "Child " + to_string_with_precision(environment_state->getDetInfo().f32pChild,
            2);
    cv::putText(localMapImg_large,
                cv::String(debug_env_car_header),
                cv::Point2f(localMapImg.size().width + 5, 145),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    cv::putText(localMapImg_large,
                cv::String(debug_env_car_l),
                cv::Point2f(localMapImg.size().width + 5, 160),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    cv::putText(localMapImg_large,
                cv::String(debug_env_car_a),
                cv::Point2f(localMapImg.size().width + 5, 175),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    cv::putText(localMapImg_large,
                cv::String(debug_env_car_r),
                cv::Point2f(localMapImg.size().width + 5, 190),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    cv::putText(localMapImg_large,
                cv::String(debug_env_person_adult),
                cv::Point2f(localMapImg.size().width + 5, 205),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));
    cv::putText(localMapImg_large,
                cv::String(debug_env_person_child),
                cv::Point2f(localMapImg.size().width + 5, 220),
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(255, 255, 255, 255));

    if (current_task->getCurrentDrivingConstraint()
            && current_task->getCurrentDrivingConstraint()->isActive()) {
        std::string debug_env_stvo_constraint = current_task->getCurrentDrivingConstraint()->getSourceTypeReadable();

        cv::putText(localMapImg_large,
                    cv::String(debug_env_stvo_constraint),
                    cv::Point2f(localMapImg.size().width + 5, 235),
                    cv::FONT_HERSHEY_PLAIN,
                    0.8,
                    cv::Scalar(255, 255, 255, 255));
    }

    if (environment_state->getJunctionMarker()) {
        std::string debug_env_junction_marker = "sgn: " + environment_state->getJunctionMarker()
                ->getReadableType();

        cv::putText(localMapImg_large,
                    cv::String(debug_env_junction_marker),
                    cv::Point2f(localMapImg.size().width + 5, 250),
                    cv::FONT_HERSHEY_PLAIN,
                    0.8,
                    cv::Scalar(255, 255, 255, 255));
    }

    // emergency break active
    if (environment_state->isEmergencyBrakeActive()) {
        cv::putText(localMapImg_large,
            cv::String("Em-Break!"),
            cv::Point2f(localMapImg.size().width + 5, 265),
            cv::FONT_HERSHEY_PLAIN,
            0.8,
            cv::Scalar(255, 255, 255, 255));
    }


    // let the tasks add their own debug output
    if (current_task) {
        current_task->writeDebugOutputToLocalMap(&localMapImg_large);
    }

    // rotate the local map 180 degrees such that the car drives upwards in the local map
    cv::Rect small_mapimg_roi = cv::Rect(0,
                                         0,
                                         localMapImg.size().width,
                                         localMapImg.size().height);
    cv::rotate(localMapImg_large(small_mapimg_roi),
               localMapImg_large(small_mapimg_roi),
               cv::ROTATE_180);

    filter_output->transmitLocalMap(&localMapImg_large);
    filter_output->transmitPlanner(&local_map->planner_image);
}


/** Sensor data callbacks **/

void StateMachineCtrl::onCarPositionUpdate(DeltaPosition position_update) {
    local_map->onUpdateGlobalPosition(*map);
}

void StateMachineCtrl::onUpdateDetectionInfo(tDetectionInfo obs_data, vector<tLaserSegStruct>
*laserSegData) {
    environment_state->update(obs_data,
                              laserSegData,
                              current_task->getWaypoints(),
                              current_task->getSteeringAnchor());
    environment_state->setEmergencyBrakeState(em_brake_override->isActive());
}

void StateMachineCtrl::onLaserScannerUpdate(const std::vector<tPolarCoordiante>& ls_polar,
                                  const std::vector<Point>& ls_global_cartesian) {
    local_map->onLaserScannerUpdate(ls_polar, ls_global_cartesian);
}

void StateMachineCtrl::onObstaclesDetected(const std::vector<Obstacle> *const obstacles,
                                           DataSource source) {
    // inactive due to no use of data, reactivate within StateMachineFilter
    // local_map->onObstaclesDetected(obstacles, source);
    // em_brake_override->onObstaclesDetected(obstacles, source);
}


void StateMachineCtrl::laserscannerPolarToGlobalCartesian(const std::vector<tPolarCoordiante>& input,
                                                          std::vector<Point>* output) {
    if (car_model) {
        for (const tPolarCoordiante &ls_pt : input) {
            tPolarCoordiante coord = ls_pt;
            coord.f32Radius /= 1000;

            if (coord.f32Angle >= 0 && coord.f32Angle < 90) {
                coord.f32Angle = 90 - coord.f32Angle;
            } else {
                coord.f32Angle = 450 - coord.f32Angle;
            }

            float x = (coord.f32Radius) *
                        cos(coord.f32Angle * static_cast<float>(DEG2RAD));
            float y = (coord.f32Radius) *
                        sin(coord.f32Angle * static_cast<float>(DEG2RAD));

            Point lsDataPt = Point::Local(x, y, 0.0);

            lsDataPt.scaleBy(Coordinate::Type::GLOBAL);
            Point front_axis = car_model->getLaserScanner();
            front_axis.toGlobalNoCopy(lsDataPt, car_model->getHeading(),
                        0.0 * Coordinate::GLOBAL, 0);
            output->push_back(lsDataPt);
        }
    }
}

void StateMachineCtrl::onPerceptionDataAvailable(const cv::Mat *const data) {
    local_map->onPerceptionDataAvailable(data);
    current_task->onLanePointDataAvailable(nullptr);
}

void StateMachineCtrl::onLaneDataAvailable(std::vector<Lane> &input) {
    current_task->onLaneDataAvailable(input);
    if (!car_model->isInitialized()) {
        return;
    }
    local_map->addLanes(&input);
}

void StateMachineCtrl::onLanePointDataAvailable(std::vector<Point> *input) {
    current_task->onLanePointDataAvailable(input);
}

void StateMachineCtrl::onRoadSignDetected(int *sign_id) {
    unique_ptr<Marker> ptr;
    if (sign_id) {
        const RoadSign *sign = RoadSignInfo::getInstance()->getSign(*sign_id, car_model.get());
        if (sign) {


            ptr = make_unique<Marker>(sign->u16Id, sign->f32X, sign->f32Y, sign->f32Direction);
            ptr->scaleBy(Coordinate::Type::GLOBAL);
            environment_state->onMarkerDetected(ptr.get());
            //environment_state->debugLog();
        }
    }

    current_task->onMarkerDetected(ptr.get());

}

void StateMachineCtrl::reset() {
    current_task.reset();
    local_map->init();
    current_speed = Speed::STOP;
    environment_state = make_shared<EnvironmentState>();
    environment_state->setCarModel(car_model);
}

void StateMachineCtrl::setCarModel(std::shared_ptr<CarModel> &carModel) {
    ICarModelDependent::setCarModel(carModel);
    local_map->setCarModel(carModel);
    environment_state->setCarModel(carModel);
}

void StateMachineCtrl::setBirdsEyeImage(const cv::Mat &be_img) {
    std::lock_guard<std::mutex> lock(birdseye_data_mutex);
    birdseye_data = be_img.clone();
    // cv::resize(birdseye_data, birdseye_data, birdseye_data.size() * 2);
}





