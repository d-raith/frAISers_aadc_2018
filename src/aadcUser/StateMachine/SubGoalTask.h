#include <utility>

//
// Created by aadc on 04.10.18.
//

#ifndef AADC_USER_MAPSUBGOALTASK_H
#define AADC_USER_MAPSUBGOALTASK_H


#include "BaseTask.h"


class SubGoalCreationException {

    std::string reason;
public:
    SubGoalCreationException(std::string reason) : reason(std::move(reason)) {

    };

    std::string getCause() const {
        return reason;
    }

};

class SubGoalTask : public BaseTask {

protected:
    SubGoal sub_goal;

    int sub_goal_proximity_scaled = 50;
    


protected:

    Point sub_goal_check_center_global = Point::Global(0, 0);

    void onUpdateProximityCenter(Point new_center) override;

public:

    SubGoalTask(int id, Type type, int extra = -1, const Point &goal = Point::Global(0, 0));

    SubGoalTask(const SubGoalTask &task):BaseTask(task) {
        sub_goal_proximity_scaled = task.sub_goal_proximity_scaled;
        sub_goal_check_center_global = task.sub_goal_check_center_global;
    }


    virtual std::string getName() = 0;

    virtual bool isSubGoalReached(const Point &position);

    bool isSubGoalReached();


    void setSubGoalProximity(float sg_prox_m) {
        sub_goal_proximity_scaled = sg_prox_m*Coordinate::GLOBAL;
    }

    void setSubGoal(const SubGoal &goal);

    SubGoal *getCurrentSubGoal();


    virtual SubGoal generateSubGoal(SubGoal *last) = 0;

    virtual bool isValidSubGoal(SubGoal *sg);

    virtual void onSubGoalReached(const SubGoal &goal);

    virtual void onSubGoalChanged(const SubGoal &old_sg, const SubGoal &new_sg);




    bool onNoGlobalGoalAvailable() override;

    void onBeforeUpdate() override;

    void writeDebugOutputToLocalMap(cv::Mat *local_map_img) override;
};


#endif //AADC_USER_MAPSUBGOALTASK_H
