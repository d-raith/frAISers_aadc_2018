//
// Created by aadc on 04.10.18.
//

#include "LaneTracker.h"


using namespace mapobjects;

bool ignore_stoplines = true;

bool ignore_crosswalks = true;
bool ignore_parking = true;


void LaneTracker::setWaypointFromMap(const GlobalWaypoint &wp) {
    // LOG_INFO("Set gwp with (%f, %f) type %d", wp.pose.getX(), wp.pose.getY(), wp.type);
    map_wp_current = wp;
    if(wp.type == mapobjects::WaypointType::ERROR){ 
        throw runtime_error("wp type error set");
    }
}


bool LaneTracker::isCurrentWpOfType(mapobjects::WaypointType type) {
    return map_wp_current.type == type;
}


const GlobalWaypoint &LaneTracker::obtainNextWp(IGlobalMap *map, float distance) {
    try {
        setWaypointFromMap(computeNextWp(map, map_wp_current, distance));
    } catch (WaypointException ex) {
        LOG_WARNING("Map returned waypoint exception: %d", ex);
    }

    return map_wp_current;
}

GlobalWaypoint LaneTracker::computeNextWp(IGlobalMap *map, const GlobalWaypoint &current, float
distance) {
    if (!validWp(current)) {
        Pose pose = buildPose();
        LOG_INFO("Request waypoint using pose(%f ,%f) %f", pose.getX(), pose.getY(), pose.getT());
        GlobalWaypoint wp = map->getNextGlobalWaypoint(pose, distance * Coordinate::Type::GLOBAL,
                                          ignore_stoplines,
                                          ignore_crosswalks,
                                          ignore_parking);
        return wp;
    } else {
        //LOG_INFO("Request waypoint using lp");
        GlobalWaypoint wp = map->getNextGlobalWaypoint(current.lp, distance * Coordinate::Type::GLOBAL,
                                          ignore_stoplines,
                                          ignore_crosswalks, ignore_parking);
        return wp;
    }
}

GlobalWaypoint LaneTracker::next(IGlobalMap *map, const GlobalWaypoint &current, float distance) {
    try {
        return computeNextWp(map, current, distance);
    } catch (WaypointException ex) {
        LOG_INFO("next() failed: Waypoint exception type %d caught", ex);
        return GlobalWaypoint(WaypointType::ERROR);
    }
}


bool LaneTracker::validWp(const GlobalWaypoint &target) {
    return target.type != WaypointType::ERROR;
}


const GlobalWaypoint &LaneTracker::getMapWaypoint() {
    return map_wp_current;
}

Point LaneTracker::getWaypoint() {
    if (map_wp_current.type == WaypointType::ERROR) {
        LOG_INFO("map waypoint is not valid");
    }
    return poseToPoint(map_wp_current.pose);
}

bool LaneTracker::hasWp() {
    return validWp(map_wp_current);
}


bool LaneTracker::isPoi(const GlobalWaypoint &point) {
    return point.type == WaypointType::JUNCTION ||
           point.type == WaypointType::STOPLINE ||
           point.type == WaypointType::CROSSWALK ||
           point.type == WaypointType::PARKING;
}

WaypointType LaneTracker::next(IGlobalMap *map, float distance_m) {
    try {
        return obtainNextWp(map, distance_m).type;
    } catch (WaypointException ex) {
        LOG_INFO("Waypoint ex: %d", ex);
        throw runtime_error("delocalized");
        return WaypointType::ERROR;
    }
}

GlobalWaypoint LaneTracker::nextPlanningPoint(IGlobalMap *map, float distance_m) {
    try {
        return computeNextWp(map, map_wp_current, distance_m);
    } catch (WaypointException ex) {
        LOG_WARNING("Map returned waypoint exception: %d", ex);
        throw runtime_error("planning point exception");
        //return GlobalWaypoint(WaypointType::ERROR);
    }
}


bool LaneTracker::computePathToNextPoi(IGlobalMap *map, const GlobalWaypoint &current, float
step_distance_m, vector<GlobalWaypoint>
                                       *out) {

    auto wp = current;

    while (validWp(wp) && !isPoi(wp)) {
        wp = next(map, wp, step_distance_m);
        out->emplace_back(wp);
    }
    return validWp(wp) && isPoi(wp);
}


Point LaneTracker::getGlobalPosition() {
    return car_model->getFrontAxis();
}


Pose LaneTracker::pointToPose(const Point &src, float heading, bool invertFrame) {
    if (!invertFrame) {
        return Pose(src.getX(), src.getY(), src.getZ(), heading);
    }
    return Pose(src.getX(), -src.getY(), src.getZ(), heading);

}

Point LaneTracker::poseToPoint(const Pose &pose, bool invertFrame) {
    if (!invertFrame) {
        return Point::Global(
                static_cast<float>(pose.getX()),
                static_cast<float>(pose.getY()),
                static_cast<float>(pose.getZ()));
    }
    return Point::Global(static_cast<float>(pose.getX()),
                         static_cast<float>(-pose.getY()),
                         static_cast<float>(pose.getZ()));
}

Pose LaneTracker::buildPose(bool invertFrame) {
    if (!invertFrame) {
        return pointToPose(getGlobalPosition(), car_model->getHeading(), invertFrame);
    }
    return pointToPose(getGlobalPosition(), -car_model->getHeading(), invertFrame);
}

bool LaneTracker::localize(IGlobalMap *map) {

    try {
        setWaypointFromMap(computeNextWp(map, GlobalWaypoint(WaypointType::ERROR), 0.1));
        return map_wp_current.type != WaypointType::ERROR;
    } catch (WaypointException ex) {
        return false;
    }
}

bool LaneTracker::isAtPoi() {
    return isPoi(map_wp_current);
}

mapobjects::WaypointType LaneTracker::nextLeftTurn(IGlobalMap *map) {
    try {
        setWaypointFromMap(map->getWaypointTurnLeft(map_wp_current.lp));
    } catch (WaypointException ex) {
        return WaypointType::ERROR;
    }
    return map_wp_current.type;
}

mapobjects::WaypointType LaneTracker::nextRightTurn(IGlobalMap *map) {
    try {
        setWaypointFromMap(map->getWaypointTurnRight(map_wp_current.lp));
    } catch (WaypointException ex) {
        return WaypointType::ERROR;
    }

    return map_wp_current.type;
}


mapobjects::WaypointType LaneTracker::nextStraight(IGlobalMap *map) {
    try {
        setWaypointFromMap(map->getWaypointDriveStraight(map_wp_current.lp));
    } catch (WaypointException ex) {
        return WaypointType::ERROR;
    }

    return map_wp_current.type;
}


mapobjects::WaypointType LaneTracker::getWaypointsLeftTurn(IGlobalMap *map, vector<Point> *wp_out) {
    try {
        auto wps = map->getWaypointsTurnLeft(map_wp_current.lp);

        if (!wps.empty()) {
            for (auto wp: wps) {
                wp_out->emplace_back(poseToPoint(wp.pose));
            }

            setWaypointFromMap(wps.back());
        }

    } catch (WaypointException ex) {
        return WaypointType::ERROR;
    }

    return map_wp_current.type;
}

bool LaneTracker::isLocalized() {
    return true;
}

mapobjects::WaypointType
LaneTracker::getWaypointsRightTurn(IGlobalMap *map, vector<Point> *wp_out) {
    try {
        auto wps = map->getWaypointsTurnRight(map_wp_current.lp);

        if (!wps.empty()) {
            for (auto wp: wps) {
                wp_out->emplace_back(poseToPoint(wp.pose));
            }

            setWaypointFromMap(wps.back());
        }

    } catch (WaypointException ex) {
        return WaypointType::ERROR;
    }

    return map_wp_current.type;
}

mapobjects::WaypointType LaneTracker::getWaypointsStraight(IGlobalMap *map, vector<Point> *wp_out) {
    try {
        auto wps = map->getWaypointsDriveStraight(map_wp_current.lp);

        if (!wps.empty()) {
            for (auto wp: wps) {
                wp_out->emplace_back(poseToPoint(wp.pose));
            }

            setWaypointFromMap(wps.back());
        }

    } catch (WaypointException ex) {
        return WaypointType::ERROR;
    }

    return map_wp_current.type;
}

bool LaneTracker::isError(mapobjects::WaypointType type) {
    return type == mapobjects::WaypointType::ERROR;

}

