#pragma once
#ifndef AADC_USER_RVDRVTASK_H
#define AADC_USER_RVDRVTASK_H

#include "Point.h"
#include "BaseTask.h"
#include "../PinClasses/StopWatch.h"

using namespace fraisers::models;

class ReverseDrivingTask : public BaseTask {
    static constexpr float WP_PROX=0.3 * Coordinate::GLOBAL;
    static constexpr float GOAL_PROX=0.3* Coordinate::GLOBAL;


public:

    ReverseDrivingTask(int id, vector<Point> &wps)
    : BaseTask(1, BaseTask::Type::straight, -1, wps.back()) {
        for (auto &wp : wps) {
            waypoints.emplace_back(wp);
            cout << "adding waypoint: " << wp << endl;
        }
    }


    ReverseDrivingTask(int id, BaseTask::Type type)
            : BaseTask(1, BaseTask::Type::straight) {
    }

    void logDebug() override {
        BaseTask::logDebug();
    }

public:

    std::string getName();

    bool setNextWaypoint(const Point &waypoint) override{
        return BaseTask::setNextWaypoint(waypoint);
    }

    void onStartTask() override;

protected:

    const Point getSteeringAnchor() override;
    Point getWaypointProximityCheckCenter() override;
    float getDrivingSpeed() override;

    bool isPointPassed(const Point &origin, const Point &target, bool invertCheck,
        int offset_tolerance) override;

    void onBeforeUpdate() override;
    void onCheckStVoConstraints() override;


public:


    float getWaypointProximity() override;

    float getGoalProximity() override;

};


#endif //AADC_USER_RVDRVTASK_H
