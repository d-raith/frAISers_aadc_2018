//
// Created by aadc on 03.10.18.
//

#ifndef AADC_USER_LANEFOLLOWINGTASK_H
#define AADC_USER_LANEFOLLOWINGTASK_H


#include "SubGoalTask.h"
#include "vector"
#include "../PinClasses/StopWatch.h"

class LaneFollowingTask : public SubGoalTask {

    static constexpr float WP_PROX=0.1 * Coordinate::GLOBAL;
    static constexpr float GOAL_PROX=0.3* Coordinate::GLOBAL;

    static constexpr float SPEED_NORMAL=Speed::SLOW;
    static constexpr float SPEED_CROSSWALK=Speed::SLOW*0.5;

    bool exit_on_parking = false;


    float driving_speed = SPEED_NORMAL;

    Timer zebra_timer = Timer(5);


public:
    explicit LaneFollowingTask(int id, bool exit_on_park_sign=false, Point goal = Point::Global(0,0));

    LaneFollowingTask(const LaneFollowingTask &task):SubGoalTask(task) {
        driving_speed = task.driving_speed;
        exit_on_parking = task.exit_on_parking;
        zebra_timer = task.zebra_timer;
    }

    ~LaneFollowingTask() = default;

    std::string getName() override;

    bool onNoGlobalGoalAvailable() override;
    void onObstacleDetected(Obstacle &obstacle) override;


    SubGoal generateSubGoal(SubGoal *last) override;

    void onSubGoalReached(const SubGoal &goal)  override;


    void onCrossingSign(Marker *marker, float dist) override;

    void onStopSign(Marker *marker, float dist) override;

    void onStraightOnly(Marker *marker, float dist) override;


protected:
    float getWaypointProximity() override;

    float getGoalProximity() override;

    void onUpdateProximityCenter(Point new_center) override;

    Point getWaypointProximityCheckCenter() override;

    const Point getSteeringAnchor() override;

public:


    void onLaneDataAvailable(vector<Lane> &laneData) override;

    void logDebug() override;



public:
    void updateCachedLanes(Lane *left, Lane *right);

    void onStartTask() override;

    void onLanePointDataAvailable(std::vector<Point> *input) override;
    void writeDebugOutputToLocalMap(cv::Mat *local_map_img) override;

    float getDrivingSpeed() override;

    void onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) override;

protected:
    void onBeforeUpdate() override;

    void onEmptyWaypoints() override;

    //float computeCurvature(const Point &waypoint) override;

    void addPerceptionAsWaypoints(vector<Point> *src, Point globalConversionAnchor = Point::Global
            (0,0), bool scale =
            true);

    void onParkSign(Marker *marker, float dist_m) override;

    void onZebraCrossingSign(Marker *marker, float dist_m) override;
};


#endif //AADC_USER_LANEFOLLOWINGTASK_H
