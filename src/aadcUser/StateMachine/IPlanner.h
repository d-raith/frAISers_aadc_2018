

#ifndef AADC_USER_IPLANNER_H
#define AADC_USER_IPLANNER_H
#pragma once
#include <vector>
#include "Point.h"
#include "LocalMap.h"
#include <opencv2/core/mat.hpp>


using namespace std;
using namespace fraisers::models;


struct SubGoal {
private:

    mapobjects::GlobalWaypoint map_obj;
    bool map_obj_set = false;


public:
    Point point = Point::Global(0,0);
    float val = 0;
    int extra = -1;
    bool allows_update = true;

    bool hasMapObj(){
        return map_obj_set;
    }

    mapobjects::GlobalWaypoint getMapObj() const {
        return map_obj;
    }


    void setMapObj(const mapobjects::GlobalWaypoint &wp){
        map_obj = wp;
        map_obj_set = true;
    }

    SubGoal()=default;
    explicit SubGoal(const mapobjects::GlobalWaypoint &wp, bool invert_y = false): map_obj(wp),
    map_obj_set(map_obj.type != mapobjects::WaypointType::ERROR),
        point(
            Point::Global(
                static_cast<float>(wp.pose.getX()),
                static_cast<float>(invert_y ? -wp.pose.getY() : wp.pose.getY()),
                static_cast<float>(wp.pose.getZ()))) {
    }
};

typedef fraisers::models::Point Waypoint;

struct PlanRequest{
    enum Status {
        PENDING,
        PROCESSING,
        SOLVED,
        FAILED
    };
    Point start = Point::Global(0, 0);
    float heading;
    SubGoal goal;
    shared_ptr<LocalMap> local_map;

    vector<Waypoint> result;

    Status status = PENDING;

    bool isSuccess(){
        return status == SOLVED;
    }

};

class IPlanner {

public:


    virtual void
    getPlan(LocalMap &local_map, PlanRequest *request, bool applyHeadingCosts) = 0;

};

#endif //AADC_USER_IPLANNER_H