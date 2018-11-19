//
// Created by aadc on 03.10.18.
//

#ifndef AADC_USER_LANEFOLLOWINGPERCEPTIONTASK_H
#define AADC_USER_LANEFOLLOWINGPERCEPTIONTASK_H


#include "SubGoalTask.h"
#include "vector"
#include "../PinClasses/StopWatch.h"
#include "DijkstraGoalpointExtractor.h"

class LaneFollowingPerceptionTask : public SubGoalTask {
    // planner constants
    const int SEGMENTATION_CHANNEL = 0;
    const int COST_CHANNEL = 2;
    const unsigned int SELECT_EACH_XTH_WAYPOINT = 10;
    const unsigned int INTERSECTION_REGION = 10;
    const float INTERSECTION_THRESHOLD = 0.75;
    const int POINT_SHIFT_X = 0;

    // mixed constants
    const int PPS_LOOKAHEAD_RADIUS = 50;  // cm
    const int DISTANCE_TO_INTERSECTION = 13;    // cm
    const float PIXEL_PER_METER = 100;
    const float PIXEL_PER_CM = round(PIXEL_PER_METER / 100);

    // goal checks etc.
    const int DISCARD_WAYPOINT_RADIUS = 12;     // cm
    const int PLANNING_GOAL_RADIUS = 20;        // cm
    const int JUNCTION_FALLBACK_DISTANCE = DISCARD_WAYPOINT_RADIUS + PLANNING_GOAL_RADIUS + 1;
    const int START_POSITION_Y = -DISCARD_WAYPOINT_RADIUS;            // cm
    const float SUBGOAL_PROXIMITY = 1.50;  // in m, catch more points behind the car already passed
    static constexpr float GOAL_PROX = 0.3* Coordinate::GLOBAL;  // not subgoal, not really used (?)

    bool exit_on_parking = false;

    Point m_pps_goalpoint = Point::Local(0, 0);
    DijkstraGoalpointExtractor goalpoint_extractor;
    cv::Point2i planning_goal;
    SubGoal planning_goal_subgoal;
    bool disable_goal_radius = false;
    bool fallback_planning = false;


 public:

    explicit LaneFollowingPerceptionTask(
        int id, bool exit_on_park_sign = false, Point goal = Point::Global(0, 0));

    LaneFollowingPerceptionTask(
        const LaneFollowingPerceptionTask &task):SubGoalTask(task) {
        exit_on_parking = task.exit_on_parking;
    }

    ~LaneFollowingPerceptionTask() = default;

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

    bool isWaypointReached(Point &position, Point &waypoint);

    bool isSubGoalReached(const Point &position);

    const Point getSteeringAnchor() override;

 public:
    void onLaneDataAvailable(vector<Lane> &laneData) override;

    void logDebug() override;



 public:


    void doReverseEmBrakeRecovery();
    
    void updateCachedLanes(Lane *left, Lane *right);

    void onStartTask() override;

    Point &getCurrentWaypoint() override;

    void onLanePointDataAvailable(std::vector<Point> *input) override;
    void writeDebugOutputToLocalMap(cv::Mat *local_map_img) override;

    float getDrivingSpeed() override;

    void onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg) override;

 protected:
    void onBeforeUpdate() override;

    void onEmptyWaypoints() override;

    // float computeCurvature(const Point &waypoint) override;

    void addPerceptionAsWaypoints(vector<Point> *src, Point globalConversionAnchor = Point::Global
            (0, 0), bool scale = true);

    void onParkSign(Marker *marker, float dist_m) override;

    // void planWaypoints(Point globalConversionAnchor);

    void planWaypointsWrapper();
    virtual void planWaypoints(Point globalConversionAnchor, SubGoal goal, bool plan_to_goal, bool
    disable_goal_radius);
};


#endif //AADC_USER_LANEFOLLOWINGPERCEPTIONTASK_H
