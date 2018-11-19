//
// Created by aadc on 06.10.18.
//

#ifndef AADC_USER_PARKOUTTASK_H
#define AADC_USER_PARKOUTTASK_H

#include "BaseTask.h"

class ParkOutTask: public BaseTask {

    static constexpr float WP_PROX=0.3 * Coordinate::GLOBAL;
    static constexpr float GOAL_PROX=0.3* Coordinate::GLOBAL;


public:

    std::string getName() override;

    ParkOutTask(int id, BaseTask::Type type);

    void onStartTask() override;

protected:

    void onCheckStVoConstraints() override;
    void onGoalReached(Point &goal) override;

    Point getWaypointProximityCheckCenter() override;

    float getWaypointProximity() override;

    float getGoalProximity() override;

    const Point getSteeringAnchor() override;

};


#endif //AADC_USER_PARKOUTTASK_H
