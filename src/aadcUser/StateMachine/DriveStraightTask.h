//
// Created by aadc on 08.10.18.
//

#ifndef AADC_USER_DRIVESTRAIGHTTASK_H
#define AADC_USER_DRIVESTRAIGHTTASK_H

#include "LaneFollowingTask.h"

class DriveStraightTask: public BaseTask {

static constexpr float WP_PROX=0.3 * Coordinate::GLOBAL;
static constexpr float GOAL_PROX=0.3* Coordinate::GLOBAL;

public:
    void onStartTask() override;

    DriveStraightTask(int id);

    std::string getName();

protected:
    Point getWaypointProximityCheckCenter() override;

    const Point getSteeringAnchor() override;

    float getWaypointProximity() override;

    float getGoalProximity() override;
};


#endif //AADC_USER_DRIVESTRAIGHTTASK_H
