#pragma once

#include <opencv2/core/core.hpp>
#include "MapObjects.h"


enum MapStatus {
    UNINITIALIZED = 1,
    LOADED_FROM_FILE = 2,
    UPDATED_FROM_REMOTE = 3
};

class IGlobalMap {

public:
    virtual cv::Mat getCenteredMapWindow(
            int window_width, int window_height, double car_x, double car_y) = 0;
    virtual cv::Mat getShiftedMapWindow(
            int window_width, int window_height, \
        double car_x, double car_y, \
        double offset_x, double offset_y) = 0;


    virtual mapobjects::GlobalWaypoint getNextGlobalWaypoint(
            const mapobjects::Pose& pose,
            double waypoint_distance,
            bool ignoreStoplines,
            bool ignoreCrosswalks,
            bool ignoreParking) = 0;
    virtual mapobjects::GlobalWaypoint getNextGlobalWaypoint(
            const mapobjects::Pose& pose,
            const mapobjects::LanePoint3D& lp,
            double waypoint_distance,
            bool ignoreStoplines,
            bool ignoreCrosswalks,
            bool ignoreParking) {
        throw std::runtime_error("not implemented");
    };
    virtual mapobjects::GlobalWaypoint getNextGlobalWaypoint(
            const mapobjects::LanePoint3D& lp,
            double waypoint_distance,
            bool ignoreStoplines,
            bool ignoreCrosswalks,
            bool ignoreParking) = 0;
    virtual mapobjects::GlobalWaypoint getWaypointTurnLeft(mapobjects::LanePoint3D lp) = 0;
    virtual mapobjects::GlobalWaypoint getWaypointDriveStraight(mapobjects::LanePoint3D lp) = 0;
    virtual mapobjects::GlobalWaypoint getWaypointTurnRight(mapobjects::LanePoint3D lp) = 0;

    virtual std::vector<mapobjects::GlobalWaypoint> getWaypointsTurnLeft(mapobjects::LanePoint3D
    lp) = 0;
    virtual std::vector<mapobjects::GlobalWaypoint> getWaypointsDriveStraight
    (mapobjects::LanePoint3D lp)
    = 0;
    virtual std::vector<mapobjects::GlobalWaypoint> getWaypointsTurnRight(mapobjects::LanePoint3D
    lp) = 0;


    virtual mapobjects::GlobalWaypoint getWaypointOnOppositeLane(
            const mapobjects::GlobalWaypoint& wp, bool invertHeading) = 0;

    virtual bool isInitialized(){
        return getMapStatus()!=MapStatus::UNINITIALIZED;
    }

    virtual MapStatus getMapStatus() = 0;
};



