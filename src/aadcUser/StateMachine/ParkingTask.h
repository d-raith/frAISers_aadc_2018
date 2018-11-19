//
// Created by aadc on 06.10.18.
//

#ifndef AADC_USER_PARKINGTASK_H
#define AADC_USER_PARKINGTASK_H

#include "SubGoalTask.h"
#include "RoadSignInfo.h"
#include "../PinClasses/RoadSignsMapPin.h"

class ParkingTask: public SubGoalTask {

    int ID_FRONT_MOVE=100;
    int ID_REVERSE_MOVE=200;

    static constexpr float WP_PROX=0.3 * Coordinate::GLOBAL;
    static constexpr float GOAL_PROX=0.3* Coordinate::GLOBAL;


    vector<Point> reverse_pts;

    deque<SubGoal> sub_goals;

public:

    explicit ParkingTask(int id, int extra);
    void onStartTask() override;

    bool isSubGoalReached(const Point &position) override;

    void generatePath();

    SubGoal generateSubGoal(SubGoal *last) override;

    void onBeforeUpdate() override;

    void onSubGoalReached(const SubGoal &goal) override;

    float getDrivingSpeed() override;

    std::string getName() override;

protected:


    const Point getSteeringAnchor() override;
    Point getWaypointProximityCheckCenter() override;

    float getWaypointProximity() override;

    float getGoalProximity() override;

protected:
    void onParkSign(Marker *marker, float dist_m) override;
};


#endif //AADC_USER_PARKINGTASK_H
