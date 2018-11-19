#ifndef AADC_USER_BASETASK_H
#define AADC_USER_BASETASK_H

#include <vector>
#include <string>

#include "Point.h"
#include "LocalMap.h"
#include "Map.h"
#include "IGlobalMap.h"
#include "LaneTracker.h"
#include "../PinClasses/StopWatch.h"
#include "BackgroundPlanner.h"
#include "ICarCtrl.h"
#include "Astar.h"
#include "ICarCtrl.h"
#include "Point.h"
#include "Lane.h"
#include "deque"
#include "CarModel.h"
#include "PurePursuit.h"
#include "EnvironmentState.h"
#include "StVO.h"


class WaypointsEmptyException {
};

struct EnumClassHash
{
    template <typename T>
    std::size_t operator()(T t) const
    {
        return static_cast<std::size_t>(t);
    }
};

class BaseTask: public ICarModelDependent {
public:
    enum class Type {
        undefined = 0,
        turn_left = 1,
        turn_right = 2,
        straight = 4,
        parallel_parking = 5,
        cross_parking = 6,
        pull_out_left = 7,
        pull_out_right = 8,
        merge_left = 9,
        merge_right = 10
    };


private:

    bool report_to_jury = true;
    int id;
    Type type;
    int extra = -1;
    ISteerCtrl *steer_ctrl = nullptr;

    IVelocityCtrl *speed_ctrl = nullptr;

    std::unordered_map<Marker::MarkerType, float, EnumClassHash> marker_trigger_map;


    //goal / wp detection on rear axis position by default
    Point wp_check_center_local = Point::Local(0, 0);

    bool verifyDrivingConstraintsExists();
    bool applyDrivingConstraints();


protected:

    ILightCtrl *light_ctrl = nullptr;
    float desired_task_speed = Speed::STOP;
    IGlobalMap *global_map = nullptr;
    IPlanner *planner = nullptr;
    Point current_wp = Point::Global(0, 0);
    LaneTracker lane_tracker;
    IEmergencyBrakeCtrl *em_brake_ctrl = nullptr;

    bool is_reverse_movement = false;

    bool is_overtake_required = false;

// TODO: offset needs to be inverted for reversing
    int point_passed_offset_tolerance = 0;


    PurePursuit pps_steering;

    Marker *detected_marker = nullptr;
    bool error = false;
    bool goal_reached = false;
    deque<Point> waypoints;
    shared_ptr<LocalMap> local_map = nullptr;
    Point goal = Point::Global(0,0);

    Timer wait_timer = Timer(3);

    bool is_waiting = false;

    shared_ptr<EnvironmentState> env_state;

    StVO stVo;
    shared_ptr<DrivingConstraint> current_constraint;


    virtual bool isPointPassed(const Point &target, bool invertCheck,
                       int offset_tolerance);
    virtual bool isPointPassed(const Point &origin, const Point &target, bool invertCheck,
                       int offset_tolerance);
    virtual void onUpdateProximityCenter(Point new_center);




    virtual bool isPointPassed(const Point &target);
    virtual bool isPointPassed(const Point &origin, const Point &target);

    virtual bool isWaypointReached(Point &position, Point &waypoint);

    virtual bool isWaypointReached(Point &waypoint);

    bool isGoalReached(Point &position);

    bool isGoalReached();


    virtual float computeCurvature(const Point &waypoint);

    virtual bool setNextWaypoint(const Point &waypoint);

    virtual bool setVelocity(const float speed);

    int waypointCount() {
        return static_cast<int>(waypoints.size());
    }

    bool hasWaypoints() {
        return waypointCount() > 0;
    }


    virtual void onCheckStVoConstraints();

    virtual bool onNoGlobalGoalAvailable();

    virtual bool isValidGoal(const Point &goal);

    virtual void onBeforeUpdate();


    virtual void onWaypointReached(Point &wp);

    virtual void onEmptyWaypoints();

    virtual void onGoalReached(Point &goal);



    virtual void onCrossingSign(Marker *marker, float dist_m){};
    virtual void onStopSign(Marker *marker, float dist_m){};
    virtual void onParkSign(Marker *marker, float dist_m){};
    virtual void onRoWSign(Marker *marker, float dist_m){};
    virtual void onGiveWaySign(Marker *marker, float dist_m){};
    virtual void onZebraCrossingSign(Marker *marker, float dist_m){};
    virtual void onRoundaboutSign(Marker *marker, float dist_m){};
    virtual void onNoOvertakingSign(Marker *marker, float dist_m){};
    virtual void onNoEnteringSign(Marker *marker, float dist_m){};
    virtual void onOneWaySign(Marker *marker, float dist_m){};
    virtual void onConstructionSign(Marker *marker, float dist_m){};
    virtual void onStraightOnly(Marker *marker, float dist_m){};
    virtual void onSpeedLimitSign(Marker *marker, float dist_m){};
    virtual void onPositioningSign(Marker *marker, float dist_m){};

    Point wp_check_center_global = Point::Global(0, 0);
public:

    virtual const Point getSteeringAnchor();

    BaseTask(int id, Type type, int
    extra=-1, const Point &goal= Point::Global(0,0)) :
                                  id(id), type(type),
                                  extra(extra), goal(goal)  {
    }

    BaseTask(const BaseTask &task) : ICarModelDependent(task) {

        id = task.id;
        type = task.type;
        extra = task.extra;

        steer_ctrl = task.steer_ctrl;
        light_ctrl = task.light_ctrl;
        speed_ctrl = task.speed_ctrl;
        em_brake_ctrl = task.em_brake_ctrl;

        global_map = task.global_map;
        planner = task.planner;
        car_model = task.car_model;
        current_wp = task.current_wp;
        wp_check_center_local = task.wp_check_center_local;
        wp_check_center_global = task.wp_check_center_global;
        detected_marker = task.detected_marker;
        error = task.error;
        goal_reached = task.goal_reached;
        waypoints = task.waypoints;
        local_map = task.local_map;
        goal = task.goal;
    }

    ~BaseTask() {
        local_map.reset();
    };


    void addMarkerTrigger(Marker::MarkerType type, float trigger_threshold_m){
        marker_trigger_map.insert({type, trigger_threshold_m});
    }

    virtual void writeDebugOutputToLocalMap(cv::Mat *local_map_img);

    virtual std::string getName() = 0;


    virtual void setGlobalMap(IGlobalMap *map);

    virtual void setLocalMap(shared_ptr<LocalMap> &localMap);

    virtual void setPlannerInstance(IPlanner *planner);


    const Point& getProximityCheckCenterGlobal() {
        return wp_check_center_global;
    }

    virtual Point getCurrentGoalPoint();

    virtual Point &getCurrentWaypoint();

    virtual void onStartTask();

    virtual void update(const float *current_speed);

    virtual float getDrivingSpeed();

    virtual Point getWaypointProximityCheckCenter();

    virtual float getWaypointProximity(){
        throw runtime_error("Call of getWaypointProximity of BaseTask not allowed!");
    }

    virtual float getGoalProximity(){
        throw runtime_error("Call of getGoalProximity of BaseTask not allowed!");
    }


    void doWait(float seconds) {
        wait_timer = Timer(seconds);
        wait_timer.start();
    }


    bool isWaiting() {
        return !wait_timer.isDone();
    }


    void setSteerCtrl(ISteerCtrl *steerCtrl) {
        this->steer_ctrl = steerCtrl;
    }

    void setLightCtrl(ILightCtrl *lightCtrl) {
        this->light_ctrl = lightCtrl;
    }

    void setVelocityCtrl(IVelocityCtrl *velocityCtrl) {
        this->speed_ctrl = velocityCtrl;
    }

    void setEmergencyBrakeCtrl(IEmergencyBrakeCtrl *em_brake_ctrl) {
        this->em_brake_ctrl = em_brake_ctrl;
    }

    IEmergencyBrakeCtrl * getEmergencyBrakeCtrl() {
        return this->em_brake_ctrl;
    }


    void setProximityCheckPositionOffset(const Point &rear_axis_offset) {
        wp_check_center_local = rear_axis_offset;
    };


    bool doReportToJury(){
        return report_to_jury;
    }

    void setReportToJury(bool report){
        this->report_to_jury = report;
    }


    deque<Point> getWaypoints() {
        return waypoints;
    };

    virtual void logDebug();

    bool isFinished() { return goal_reached; }

    bool isError() { return error; }

    int getId() { return id; }

    Type getType() { return type; }


    bool isOvertakeRequested();

    void requestOvertake();

    void doReverseEmBrakeRecovery();

    shared_ptr<DrivingConstraint> getCurrentDrivingConstraint();

    int getExtra(){
        return extra;
    }

    void setExtra(int extra) {
        this->extra = extra;
    }


    void setLaneTracker(const LaneTracker &tracker) {
        lane_tracker = tracker;
    }

    const LaneTracker& getLaneTracker() const {
        return lane_tracker;
    }


    void setCarModel(std::shared_ptr<CarModel> &carModel) override;

    void setEnvironmentState(const shared_ptr<EnvironmentState> &state);


    virtual void onLaneDataAvailable(std::vector<Lane> &laneData);



    virtual void onObstacleDetected(Obstacle &obstacle);

    virtual void onMarkerDetected(Marker *marker);

    virtual void onLanePointDataAvailable(std::vector<Point> *input) {

    }

    virtual void onLocalMapAvailable(shared_ptr<LocalMap> &localMap){

    }

    virtual void onGlobalMapAvailable(IGlobalMap *global_map){

    }

    virtual void onPlannerAvailable(IPlanner *planner){

    }


};


#endif //AADC_USER_BASETASK_H
