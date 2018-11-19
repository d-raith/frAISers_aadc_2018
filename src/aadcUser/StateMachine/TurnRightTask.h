//
// Created by aadc on 04.10.18.
//

#ifndef AADC_USER_TURNRIGHTTASK_H
#define AADC_USER_TURNRIGHTTASK_H
#include "SubGoalTask.h"
#include "../PinClasses/StopWatch.h"

class TurnRightTask: public SubGoalTask {



    static constexpr float WP_PROX=0.25 * Coordinate::GLOBAL;
    static constexpr float GOAL_PROX=0.2* Coordinate::GLOBAL;

    SubGoal generateSubGoal(SubGoal *last) override;

    bool needs_relocalize = false;

    void generateWaypoints();

    void buildWaypointsTurnRight();

public:

    std::string getName() override;

    void onStartTask() override;

    float getDrivingSpeed() override;

    TurnRightTask(int id, int extra, const Point &goal);

    TurnRightTask(int id);

protected:
    float getWaypointProximity() override;

    float getGoalProximity() override;

    void onGoalReached(Point &goal) override;

public:
    void onBeforeUpdate() override;

protected:
    Point getWaypointProximityCheckCenter() override;

protected:
    const Point getSteeringAnchor() override;
};


#endif //AADC_USER_TURNRIGHTTASK_H
