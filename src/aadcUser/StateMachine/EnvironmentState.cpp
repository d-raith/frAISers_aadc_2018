//
// Created by aadc on 23.10.18.
//

#include "EnvironmentState.h"


bool EnvironmentState::isCarAhead() const {
    return detection_info.f32pCarCenter >= p_threshold_car;
}

bool EnvironmentState::isPersonDetected() const {
    return detection_info.f32pPerson >= p_threshold_person;
}

bool EnvironmentState::isCarLeft() const {
    return detection_info.f32pCarLeft >= p_threshold_car;
}

bool EnvironmentState::isCarRight() const {
    return detection_info.f32pCarRight >= p_threshold_car;
}


bool EnvironmentState::isUsObstacleLeft() const{
    return detection_info.f32pObsLeft>= p_threshold_us_detection;
}

bool EnvironmentState::isUsObstacleRight() const{
    return detection_info.f32pObsRight>= p_threshold_us_detection;
}

bool EnvironmentState::isUsObstacleRear() const{
    return detection_info.f32pObsRear>= p_threshold_us_detection;
}


bool EnvironmentState::isCrossWalkDetected() const {
    bool marker_valid = isMarkerValid(crosswalk_marker, crosswalk_keep_alive_seconds);
    if (!marker_valid) {
        return false;
    }
    bool marker_in_range;
    if (crosswalk_marker->getElapsedMilliseconds() > after_marker_timer) {
        marker_in_range = crosswalk_marker->getElement()->distanceTo(car_model->getFrontAxis())
                                < after_crosswalk_distance;
        // LOG_INFO("before marker, marker_in_range=%d", marker_in_range);
    } else {
        marker_in_range = crosswalk_marker->getElement()->distanceTo(car_model->getFrontAxis())
                                < crosswalk_detection_distance;
        // LOG_INFO("after marker, marker_in_range=%d", marker_in_range);
    }
    return marker_in_range;
}


bool EnvironmentState::isChildDetected() const {
    return detection_info.f32pChild >= p_threshold_person;
}

const Marker *EnvironmentState::getJunctionMarker() const {
    if (junction_marker) {
        return junction_marker->getElement();
    }
    return nullptr;
}

bool EnvironmentState::isJunctionMarkerValid(int age_seconds) const {
    return isMarkerValid(junction_marker, age_seconds);
}

bool
EnvironmentState::isMarkerValid(const std::unique_ptr<TimedEvent<fraisers::models::Marker>> &marker,
                                int
                                age_seconds) const {

    return marker && !marker->getTimer().didSecondsPass(age_seconds);
}

float EnvironmentState::getCurvatureSteering() {
    return steering_curvature;
}

void EnvironmentState::updateSteeringValue(float curv_value) {
    steering_curvature = curv_value;
}

// only forward declaration because already defined somewhere else
// float cvPoint2fDistance(cv::Point2f a, cv::Point2f b) {
//     return sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
// }
// int cvPoint2iDistance(cv::Point2i a, cv::Point2i b) {
//     return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
// }
int cvPoint2iDistance(cv::Point2i a, cv::Point2i b);

void EnvironmentState::update_with_adaptive_region(std::vector<Obstacle> obstacles,
                                                   Point steering_anchor) {
    // StopWatch watch;
    float em_brake_corridor_width = 20;
    float detection_distance_frontaxis = 100;
    float steering_curvature = getCurvatureSteering();
    if (abs(steering_curvature) > 0.001) {
        float steering_radius = (1.0 / steering_curvature) * 100.0;
        Point local_steer_anchor = car_model->getRearAxis().toLocal(steering_anchor, car_model->getHeading());

        cv::Point steering_center = cv::Point2i(local_steer_anchor.getX(), local_steer_anchor.getY())
                                        - cv::Point2i(steering_radius, 0);

        for (const Obstacle obstacle : obstacles) {
            Point local_scan_pt = car_model->getRearAxis().toLocal(obstacle, car_model->getHeading());
            cv::Point2i local_obstacle = cv::Point2i(local_scan_pt.getX(), local_scan_pt.getY());
            int dist_steeringcenter_obstacle = cvPoint2iDistance(steering_center, local_obstacle);
            bool inCorridor = (dist_steeringcenter_obstacle
                                > abs(steering_radius) - em_brake_corridor_width)
                                    && (dist_steeringcenter_obstacle
                                        < abs(steering_radius) + em_brake_corridor_width);


            // if (inCorridor) {
            // LOG_INFO("scanpoint.x=%f, scanpoint.y=%f", scanpoint.getX(), scanpoint.getY());
            // LOG_INFO("inCorrdor=%i", inCorridor);
            // LOG_INFO("car_model->getFrontAxis().distanceTo(scanpoint)=%f", car_model->getFrontAxis().distanceTo(scanpoint));
            // }
            if (inCorridor
                    && car_model->getFrontAxis().distanceTo(obstacle)
                        < detection_distance_frontaxis) {
                car_in_planned_path_hits++;
                return;
            }
        }
    } else {
        // LOG_INFO("debug embrake: straight case");
        // check for straight
        for (const Obstacle obstacle : obstacles) {
            // std::cout << "scanpoint" << scanpoint << std::endl;
            Point local_scan_pt = car_model->getRearAxis().toLocal(obstacle, car_model->getHeading());
            // std::cout << "local_scan_pt" << local_scan_pt << std::endl;
            cv::Point2i local_obstacle = cv::Point2i(local_scan_pt.getX(), local_scan_pt.getY());
            float dist = car_model->getFrontAxis().distanceTo(obstacle);
            // std::cout << "dist" << dist << std::endl;
            if (dist
                        < detection_distance_frontaxis
                    && local_obstacle.x
                        > - em_brake_corridor_width
                    && local_obstacle.x
                        <  em_brake_corridor_width) {
                car_in_planned_path_hits++;
                return;
            }
        }
    }
    car_in_planned_path_misses++;
    // watch.print_measurement("env-state update for overtaking");
}

void EnvironmentState::update(tDetectionInfo det_info,
                              vector<tLaserSegStruct> *laserSegData,
                              std::deque<Point> waypoints,
                              Point steering_anchor) {
    detection_info = det_info;
    this->laser_seg_data = *laserSegData;

    //    version with region that is adapted to steering
    std::vector<Obstacle> obstacles;
    getObstacles(4, &obstacles);
    update_with_adaptive_region(obstacles, steering_anchor);
    if (car_in_planned_path_hits + car_in_planned_path_misses > 10) {
        car_in_planned_path_hits /= 2;
        car_in_planned_path_misses /= 2;
    }
    //    end version with region that is adapted to steering

    // //    version with distance to waypoints
    // std::vector<Obstacle> obstacles;
    // getObstacles(4, &obstacles);
    // bool car_in_planned_path = false;
    // for (Obstacle obstacle : obstacles) {
    //     for (Point waypoint : waypoints) {
    //         if (waypoint.distanceTo(obstacle) < 15) {
    //             car_in_planned_path = true;
    //         }
    //     }
    // }
    // if (car_in_planned_path) {
    //     car_in_planned_path_hits++;
    // } else {
    //     car_in_planned_path_misses++;
    // }
    // if (car_in_planned_path_hits + car_in_planned_path_misses > 10) {
    //     car_in_planned_path_hits /= 2;
    //     car_in_planned_path_misses /= 2;
    // }
    // //   end version with distance to waypoints

    //LOG_INFO("Read: %f %f %f %f %f", obs_data.f32pCarLeft, obs_data.f32pCarCenter, obs_data.f32pCarRight, obs_data.f32pPerson, obs_data.f32pChild);
}

void EnvironmentState::onCrossWalkSignDetected(Marker *marker) {
    if (crosswalk_marker) {
        crosswalk_marker->refresh(*marker);
    } else {
        crosswalk_marker = std::make_unique<TimedEvent<Marker>>(*marker);
    }
}

void EnvironmentState::onJunctionSignDetected(Marker *marker) {
    if (debug_output) {LOG_INFO("Junction detected");}
    if (junction_marker) {
        junction_marker->refresh(*marker);
        if (debug_output) {LOG_INFO("Marker refreshed");}
    } else {
        if (debug_output) {LOG_INFO("Marker set");}
        junction_marker = std::make_unique<TimedEvent<Marker>>(*marker);
    }
}


void EnvironmentState::onMarkerDetected(fraisers::models::Marker *marker) {
    if (!marker) {
        // no marker currently detected
        if (!isMarkerValid(junction_marker, marker_keep_alive_seconds)) {
            // remove the marker if its last detection exceeds the maximum age
            if (debug_output) {LOG_INFO("Junction marker not valid anymore, resetting");}
            junction_marker.reset();
        }

        if (!isMarkerValid(crosswalk_marker, crosswalk_keep_alive_seconds)) {
            // remove the marker if its last detection exceeds the maximum age
            if (debug_output) {LOG_INFO("Junction marker not valid anymore, resetting");}
            crosswalk_marker.reset();
        }
        return;
    }

    switch (marker->marker_type) {
        case Marker::MarkerType::ZEBRA:
            onCrossWalkSignDetected(marker);
            break;
        case Marker::MarkerType::STRAIGHT_ONLY:
            onJunctionSignDetected(marker);
            break;
        case Marker::MarkerType::STOP:
            onJunctionSignDetected(marker);
            break;
        case Marker::MarkerType::GIVE_WAY:
            onJunctionSignDetected(marker);
            break;
        case Marker::MarkerType::CROSSING:
            onJunctionSignDetected(marker);
            break;
        case Marker::MarkerType::RIGHT_OF_WAY:
            onJunctionSignDetected(marker);
            break;
        default:
            break;
    }
}

bool EnvironmentState::isCarInPlannedPath() const {
    return getCarInPlannedPathProb() > 0.2;
}

float EnvironmentState::getCarInPlannedPathProb() const {
    if (car_in_planned_path_hits + car_in_planned_path_misses <= 0) {
        return 0.0;
    }
    return (static_cast<float>(car_in_planned_path_hits)
        / (static_cast<float>(car_in_planned_path_hits)
            + static_cast<float>(car_in_planned_path_misses)));
}

const tDetectionInfo& EnvironmentState::getDetInfo() const {
    return detection_info;
}

void EnvironmentState::debugLog() {
    if (!debug_output) {
        return;
    }
    // LOG_INFO("EnvState:");
    // LOG_INFO("Car: l: %f, r: %f, a: %f |cw: %d| person: %f | child: %f", detection_info
    // .f32pCarLeft,
    //         detection_info.f32pCarRight, detection_info.f32pCarCenter, isCrossWalkDetected(),
    //          detection_info.f32pPerson, detection_info.f32pChild);
    if (isMarkerValid(junction_marker, marker_keep_alive_seconds)) {
        LOG_INFO("Junction: %s", junction_marker->getElement()->getReadableType().c_str());
        LOG_INFO("detected %f ms ago", junction_marker->getElapsedMilliseconds());
    } else {
        LOG_INFO("No junction");
    }
}

bool EnvironmentState::getObstacles(int classId, vector<Obstacle> *obs_out) const {
    for (auto &det: laser_seg_data) {
        if (det.i16Class == classId) {
            obs_out->emplace_back(obs_processor.processLaserSegStruct(det));
        }
    }
    return !obs_out->empty();
}

void EnvironmentState::setCarModel(std::shared_ptr<CarModel> &carModel) {
    ICarModelDependent::setCarModel(carModel);
    obs_processor.setCarModel(carModel);
}

CarModel* EnvironmentState::getCarModel() const {
    return car_model.get();
}

void EnvironmentState::setEmergencyBrakeState(bool isActive) {
    emergency_brake_active = isActive;
}

bool EnvironmentState::isEmergencyBrakeActive() const {
    return emergency_brake_active;
}

float EnvironmentState::getProbCarAhead() const {
    return detection_info.f32pCarCenter;
}

float EnvironmentState::getProbCarRight() const {
    return detection_info.f32pCarRight;
}

float EnvironmentState::getProbCarLeft() const {
    return detection_info.f32pCarLeft;
}


float EnvironmentState::getProbPerson() const {
    return detection_info.f32pPerson;
}


float EnvironmentState::getProbChild() const {
    return detection_info.f32pChild;
}

float EnvironmentState::getProbObsLeft() const {
    return detection_info.f32pObsLeft;
}

float EnvironmentState::getProbObsRight() const {
    return detection_info.f32pObsRight;
}

float EnvironmentState::getProbObsRear() const {
    return detection_info.f32pObsRear;
}





