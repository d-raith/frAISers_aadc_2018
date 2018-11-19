//
// Created by aadc on 04.10.18.
//

#ifndef AADC_USER_LANETRACKER_H
#define AADC_USER_LANETRACKER_H

#include "CarModel.h"
#include "MapObjects.h"
#include "IGlobalMap.h"


class LaneTracker : public ICarModelDependent {

    mapobjects::GlobalWaypoint map_wp_current = mapobjects::GlobalWaypoint(mapobjects::WaypointType::ERROR);


    void setWaypointFromMap(const mapobjects::GlobalWaypoint &wp);


    const mapobjects::GlobalWaypoint &obtainNextWp(IGlobalMap *map, float distance);

    mapobjects::GlobalWaypoint computeNextWp(IGlobalMap *map, const mapobjects::GlobalWaypoint &current, float
    distance);

    mapobjects::GlobalWaypoint next(IGlobalMap *map, const mapobjects::GlobalWaypoint &current, float distance);

    


    bool validWp(const mapobjects::GlobalWaypoint &target);

public:
    LaneTracker() = default;


    const mapobjects::GlobalWaypoint &getMapWaypoint();

    Point getWaypoint();

    bool hasWp();


    bool isLocalized();

    bool localize(IGlobalMap *map);

    bool isPoi(const mapobjects::GlobalWaypoint &point);

    bool isAtPoi();

    bool isError(mapobjects::WaypointType type);


    bool isCurrentWpOfType(mapobjects::WaypointType type);

    mapobjects::WaypointType next(IGlobalMap *map, float distance_m);
    mapobjects::GlobalWaypoint nextPlanningPoint(IGlobalMap *map, float distance_m);


    mapobjects::WaypointType nextLeftTurn(IGlobalMap *map);
    mapobjects::WaypointType nextRightTurn(IGlobalMap *map);
    mapobjects::WaypointType nextStraight(IGlobalMap *map);

    mapobjects::WaypointType getWaypointsLeftTurn(IGlobalMap *map,
            std::vector<fraisers::models::Point> *wp_out);
    mapobjects::WaypointType getWaypointsRightTurn(IGlobalMap *map,
            std::vector<fraisers::models::Point> *wp_out);
    mapobjects::WaypointType getWaypointsStraight(IGlobalMap *map,
            std::vector<fraisers::models::Point> *wp_out);



    bool computePathToNextPoi(IGlobalMap *map, const mapobjects::GlobalWaypoint &current, float
    step_distance_m, vector<mapobjects::GlobalWaypoint>
    *out);


    Point getGlobalPosition();


    mapobjects::Pose pointToPose(const Point &src, float heading, bool invertFrame = true);

    Point poseToPoint(const mapobjects::Pose &pose, bool invertFrame = true);

    mapobjects::Pose buildPose(bool invertFrame = true);


};


#endif //AADC_USER_LANETRACKER_H
