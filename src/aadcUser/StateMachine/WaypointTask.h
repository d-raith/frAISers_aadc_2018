
#pragma once
#ifndef AADC_USER_WPTASK_H
#define AADC_USER_WPTASK_H

#include "Point.h"

#include "LocalMap.h"
#include "Map.h"
#include "../PinClasses/StopWatch.h"
#include "BackgroundPlanner.h"
#include "ICarCtrl.h"
#include <vector>
#include "Astar.h"
#include "ICarCtrl.h"
#include "Point.h"
#include "IGlobalMap.h"
#include "Lane.h"
#include "deque"
#include "BaseTask.h"

using namespace fraisers::models;

class WaypointTask : public BaseTask {


    static constexpr float WP_PROX=0.3 * Coordinate::GLOBAL;
    static constexpr float GOAL_PROX=0.3* Coordinate::GLOBAL;

public:

    std::string getName() override;

    WaypointTask(int id, vector<Point> &wps)
    : BaseTask(id, BaseTask::Type::straight, -1,  wps.back()) {
        for (auto &wp : wps) {
            waypoints.emplace_back(wp);
        }
    }

    WaypointTask(int id, BaseTask::Type type)
            : BaseTask(id, type) {
    }

    void logDebug() override {
        BaseTask::logDebug();
    }

protected:

    float getDrivingSpeed() override;

    void onCheckStVoConstraints() override;

public:
    Point getWaypointProximityCheckCenter() override;

    void onStartTask() override;

protected:
    const Point getSteeringAnchor() override;

public:
    float getWaypointProximity() override;

    float getGoalProximity() override;


};


#endif //AADC_USER_WPTASK_H
