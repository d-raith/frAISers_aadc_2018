


#ifndef AADC_USER_PLANNINGTASK_H
#define AADC_USER_PLANNINGTASK_H
#include "BaseTask.h"
#include "Point.h"
#include "BackgroundPlanner.h"
#include "LocalMap.h"
#include "ICarCtrl.h"

using namespace fraisers::models;

class PlanningTask : public BaseTask {

    static constexpr float WP_PROX=0.3 * Coordinate::GLOBAL;
    static constexpr float GOAL_PROX=0.3* Coordinate::GLOBAL;

    // not copyable
    BackgroundPlanner bgPlanner;
    StopWatch update_plan_watch;
    StopWatch waiting_timer_turn;

    deque<SubGoal> local_goals;

    bool is_waiting_timer_in_progress = false;

    int num_secs_wait_before_turn = 3;

protected:

    bool isLocalGoalReached(Point &position);

    bool isLocalGoalReached();

    virtual void generateLocalGoals(SubGoal *last);
    virtual void onComputeLocalGoal();

    void onLocalGoalReached(const SubGoal &goal);

    bool onNoGlobalGoalAvailable() override;


    SubGoal getCurrentLocalGoal();


    void requestNewPlan();

    void updateFromPlan(PlanRequest solvedRequest);

    void updateLocalGoal(SubGoal *l_goal, Marker *marker);




    Point getCurrentGoalPoint() override;

    void onBeforeUpdate() override;

    void onWaypointReached(Point &wp) override;

    void onEmptyWaypoints() override;

    void onGoalReached(Point &goal) override;



    void onCrossingSign(Marker *marker, float dist) override;

    void onStopSign(Marker *marker, float dist) override;

    void onStraightOnly(Marker *marker, float dist) override;


    Point getWaypointProximityCheckCenter() override;

    float getWaypointProximity() override;

    float getGoalProximity() override;


public:


    PlanningTask(int id, BaseTask::Type type,int extra = -1, const Point &goal = Point::Global
            (0,0)) : BaseTask(id, type, extra, goal) {
    }

    PlanningTask(const PlanningTask &task):BaseTask(task) {
        local_goals = task.local_goals;
        update_plan_watch = task.update_plan_watch;
        cout << "Planning task copied" <<endl;
    }
    ~PlanningTask() = default;

    std::string getName() override;


    void onStartTask() override;

    void logDebug() override;

    void onLaneDataAvailable(vector<Lane> &laneData) override;

    void onObstacleDetected(Obstacle &obstacle) override;


    void onLocalMapAvailable(shared_ptr<LocalMap> &localMap) override;

    void onGlobalMapAvailable(IGlobalMap *global_map) override;

    void onPlannerAvailable(IPlanner *planner) override;


};


#endif //AADC_USER_PLANNINGTASK_H
