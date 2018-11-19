//
// Created by aadc on 23.10.18.
//

#ifndef AADC_USER_STVO_H
#define AADC_USER_STVO_H

#include "EnvironmentState.h"
#include "ICarCtrl.h"

class BaseTask;

enum ConstraintType {
    NONE = -1,
    PERSON = 1,
    CROSSWALK = 2,
    JUNCTION_SIGN = 3,
    OVERTAKE = 4,
    EM_BRAKE_OVRD = 5
};


class DrivingConstraint;

class StVORule {
public:
    virtual bool appliesInState(const EnvironmentState &state, BaseTask *task, bool
    at_junction) const = 0;

    virtual bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) = 0;

    virtual void onInitConstraint(shared_ptr<DrivingConstraint> constraint,
                                  const EnvironmentState &state,
                                  BaseTask *task) = 0;

    virtual void onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                                const EnvironmentState &state,
                                BaseTask *task) = 0;


    virtual ConstraintType getType() const = 0;
};

class ConstraintOverride {
public:
    enum Type {
        SLOW_MOVE,
        INIT_OVERTAKE,
        REVERSE
    };

    Type overrideType;
    explicit ConstraintOverride(Type type):overrideType(type) {

    }
    ConstraintOverride() : overrideType(SLOW_MOVE) {

    }

    bool apply(float *speed, deque<Point> *wps, BaseTask* task);
};

class DrivingConstraint {

    std::vector<Point> waypoints;
    Timer duration = Timer(0);
    bool is_active = false;
    shared_ptr<StVORule> src_rule;
    float max_duration_seconds = -1;
    ConstraintOverride max_wait_override;


public:
     DrivingConstraint() = delete;

    explicit DrivingConstraint (shared_ptr<StVORule> src) {
         src_rule = src;
     }
    bool apply(float *speed, deque<Point> *wps, BaseTask* task) {
        if (isMaxWaitTimeExceeded()) {
            return max_wait_override.apply(speed, wps, task);
            LOG_INFO("wait time exceeded, apply override");
        } else {
            *speed = getDrivingSpeed();
            return false;
        }

    }

    float speed = Speed::STOP;
    void activate(float speed, float duration_s) {
        if (!isActive()) {
            setDuration(duration_s);
            duration.start();
        }

        is_active = !duration.isDone();
        this->speed = speed;
        LOG_INFO("Activate %s activated", getSourceTypeReadable().c_str());
    }

    void extend() {
        this->extend(duration.getDuration());
    }

    void extend(float seconds) {
        this->extend(getDrivingSpeed(), seconds);
    }

    void extend(float speed, float duration_s){
        duration.addDuration(duration_s);
        is_active = !duration.isDone();
        this->speed = speed;
        LOG_INFO("Constraint extended, duration is now: %f | elapsed: %f", duration.getDuration(), duration.getElapsedMs()/1000);
    }


    void updateFrom(const DrivingConstraint& other) {
        speed = other.speed;
        src_rule = other.src_rule;
        max_duration_seconds = other.max_duration_seconds;

        // reactivate if updated timer is not elapsed
        duration.addDuration(other.getTimer().getDuration());
        duration.start();
        is_active = !duration.isDone();
    }


    bool isMaxWaitTimeExceeded() {
        return max_duration_seconds > 0 && isActive()
            && duration.isElapsed(max_duration_seconds);
    }

    void setDuration(float seconds) {
        duration = Timer(seconds);
    }

    void setMaxDuration(float seconds) {
        max_duration_seconds = seconds;
    }

    void setMaxWaitOverride(ConstraintOverride ovrde) {
        max_wait_override = ovrde;
    }


    shared_ptr<StVORule> getRule() {
        return src_rule;
    }


    float getDrivingSpeed() {

        /*if (max_duration_seconds > 0 && isActive()
                && duration.isElapsed(max_duration_seconds)) {
            // slowly drive, ignoring constraint
            LOG_INFO("Constraint override, max duration elapsed");
            return static_cast<float>(Speed::SLOW * 0.8);
        }*/

        return speed;
    }

    const Timer &getTimer() const {
        return duration;
    }

    ConstraintType getSourceType() {
        if(!src_rule){
            return ConstraintType::NONE;
        }
        return src_rule->getType();
    }

    bool isActive() {
        return is_active && !duration.isDone();
    }


    std::string getSourceTypeReadable() {
        switch (getSourceType()) {
            case NONE:
                return "None";
            case PERSON:return "Person";
            case CROSSWALK:return "Crosswalk";
            case JUNCTION_SIGN:return "Junction";
            case OVERTAKE:return "Overtake";
            case EM_BRAKE_OVRD: return "Reverse";
        }
        return "Unknown";
    }
};


class SignRule : public StVORule {
    Marker::MarkerType sign_type;

public:
    SignRule() = delete;

    explicit SignRule(Marker::MarkerType type) : sign_type(type) {

    }


    bool appliesInState(const EnvironmentState &state, BaseTask *task,
                        bool at_junction) const override {
        return at_junction && state.getJunctionMarker() && state.getJunctionMarker()
                                                                   ->marker_type == sign_type;
    }


    void onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                          BaseTask *task) override {
        LOG_INFO("Activate sign rule");
        constraint->activate(Speed::STOP, 1);
        constraint->setMaxDuration(12);
        constraint->setMaxWaitOverride(ConstraintOverride(ConstraintOverride::Type::SLOW_MOVE));
    }

    void onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                            const EnvironmentState &state,
                            BaseTask *task) override {
        constraint->extend(0.5);
    }

    const Marker::MarkerType &getSignType() {
        return sign_type;
    }

    ConstraintType getType() const override {
        return JUNCTION_SIGN;
    }
};


class Crossing : public SignRule {
public:

    Crossing() : SignRule(Marker::MarkerType::CROSSING) {

    }

    bool appliesInState(const EnvironmentState &state, BaseTask *task,
                        bool at_junction) const override;

    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;
};


class RightOfWay : public SignRule {
public:

    RightOfWay() : SignRule(Marker::MarkerType::RIGHT_OF_WAY) {

    }

    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;
};


class StraightOnly : public SignRule {
public:

    StraightOnly() : SignRule(Marker::MarkerType::STRAIGHT_ONLY) {

    }

    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;
};

class GiveWay : public SignRule {
public:

    GiveWay() : SignRule(Marker::MarkerType::GIVE_WAY) {

    }

    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;
};

class Stop : public SignRule {
public:

    Stop() : SignRule(Marker::MarkerType::STOP) {

    }

    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;
};

class CrossWalk : public StVORule {
    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;

    void onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                          BaseTask *task) override;

    void onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                            const EnvironmentState &state,
                            BaseTask *task)override;

    bool appliesInState(const EnvironmentState &state, BaseTask *task, bool
    at_junction) const override;

    ConstraintType getType() const override {
        return CROSSWALK;
    }

};

class Person : public StVORule {
    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;

    void onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                          BaseTask *task) override;
    void onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                            const EnvironmentState &state,
                            BaseTask *task) override;

    bool appliesInState(const EnvironmentState &state, BaseTask *task, bool
    at_junction) const override;

    ConstraintType getType() const override {
        return PERSON;
    }
};

class Child : public StVORule {
    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;

    void onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                          BaseTask *task) override;
    void onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                            const EnvironmentState &state,
                            BaseTask *task) override;

    bool appliesInState(const EnvironmentState &state, BaseTask *task, bool
    at_junction) const override;

    ConstraintType getType() const override {
        return PERSON;
    }
};


class Overtake: public StVORule {

    static constexpr float MIN_DIST_RULE_INIT = 180;
    static constexpr float MIN_DIST_INIT_OVERTAKE = 120;
    static constexpr float MIN_PROB_CAR_AHEAD = 0.5;

    Obstacle* getClosestCar(const EnvironmentState &state) const;

    // bool isOvertakeTargetInLocalWindow(const EnvironmentState &state, Obstacle* closest_car) const;

    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;

    void onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                          BaseTask *task) override;
    void onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                            const EnvironmentState &state,
                            BaseTask *task) override;

    bool appliesInState(const EnvironmentState &state, BaseTask *task, bool
    at_junction) const override;

    bool isConstraintValid(const EnvironmentState &state) const;

    ConstraintType getType() const override {
        return OVERTAKE;
    }
};

class EmergencyBrakeRecovery: public StVORule {
    bool isConstraintRequired(const EnvironmentState &state, BaseTask *task) override;

    void onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                          BaseTask *task) override;
    void onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                            const EnvironmentState &state,
                            BaseTask *task) override;

    bool appliesInState(const EnvironmentState &state, BaseTask *task, bool
    at_junction) const override;

    ConstraintType getType() const override {
        return EM_BRAKE_OVRD;
    }
};


class StVO {

    std::vector<std::shared_ptr<StVORule>> ruleset;

    shared_ptr<StVORule> findRule(const EnvironmentState &state, BaseTask *task, bool at_junction,
            shared_ptr<DrivingConstraint> current)
    const;

public:
    StVO();

    void updateConstraint(const EnvironmentState &state, BaseTask *task, bool
    at_junction, shared_ptr<DrivingConstraint> &current) const;
};


#endif //AADC_USER_STVO_H
