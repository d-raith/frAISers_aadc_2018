//
// Created by aadc on 23.10.18.
//

#include "StVO.h"
#include "BaseTask.h"


/**
 * Checks if the provided constraint still applies, updates it if required.
 * Checks other rules if not. Resets pointer to nullptr if no constraint applies.
 * */
void StVO::updateConstraint(const EnvironmentState &state, BaseTask *task, bool
at_junction, shared_ptr<DrivingConstraint>& current) const {

    // if constraint exists, revalidate using rule the constraint originated from
    if (current) {
        if (current->getSourceType() != ConstraintType::NONE) {
            if (current->getRule()->appliesInState(state, task, at_junction)
                    && current->getRule()->isConstraintRequired(state, task)) {
                current->getRule()->onExtendConstraint(current, state, task);
                LOG_INFO("existing rule extended");
                return;
            } else {
                LOG_INFO("existing rule resetted");
            current.reset();
            }
        }
    }
    // find rule that applies in current state, create constraint if necessary
    shared_ptr<StVORule> rule = findRule(state, task, at_junction, current);
    if (rule) {
        // find rule checks applies in state & constraint required
        current.reset(new DrivingConstraint(rule));
        rule->onInitConstraint(current, state, task);
        LOG_INFO("new rule initialized");

    }
    if (current) {
        LOG_INFO("updateConstraint: rule: %s", current->getSourceTypeReadable().c_str());
        LOG_INFO("at_junction %d", at_junction);
    }
}


StVO::StVO() {

    // add rules to stvo ruleset, currently order is important as only first rule applicable will
    // be enforced

    // junction related rules
    ruleset.emplace_back(std::make_shared<Crossing>());
    ruleset.emplace_back(std::make_shared<RightOfWay>());
    ruleset.emplace_back(std::make_shared<Stop>());
    ruleset.emplace_back(std::make_shared<StraightOnly>());
    ruleset.emplace_back(std::make_shared<GiveWay>());

    // generic, non junction rules
    ruleset.emplace_back(std::make_shared<Overtake>());
    ruleset.emplace_back(std::make_shared<CrossWalk>());
    ruleset.emplace_back(std::make_shared<Person>());
    ruleset.emplace_back(std::make_shared<Child>());

    ruleset.emplace_back(std::make_shared<EmergencyBrakeRecovery>());

}

shared_ptr<StVORule> StVO::findRule(const EnvironmentState &state, BaseTask *task, bool at_junction,
                         shared_ptr<DrivingConstraint> current)
const {
    for (auto &rule : ruleset) {
        if (rule->appliesInState(state, task, at_junction)
            && rule->isConstraintRequired(state, task)) {
            //LOG_INFO("Rule %d applies and is required", rule->getType());
            return rule;
        }
    }
    return nullptr;
}

// Crossing sign / normal junction

 bool Crossing::appliesInState(const EnvironmentState &state, BaseTask *task,
                        bool at_junction) const {
    return SignRule::appliesInState(state, task, at_junction)
        || (at_junction && !state.isJunctionMarkerValid(4));
}
bool Crossing::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    BaseTask::Type man_type = task->getType();
    if (man_type == BaseTask::Type::turn_left) {
        return state.isCarRight() || state.isCarAhead();

    } else if (man_type == BaseTask::Type::turn_right) {
        return false;

    } else if (man_type == BaseTask::Type::straight) {
        return state.isCarRight();
    }
    return false;
}

// Right of way

bool RightOfWay::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    BaseTask::Type man_type = task->getType();
    if (man_type == BaseTask::Type::turn_left) {
        return state.isCarAhead();
    } else if (man_type == BaseTask::Type::turn_right) {
        return false;

    } else if (man_type == BaseTask::Type::straight) {
        return false;
    }
    return false;
}

// straight only // Stop if trying other than going straight

bool StraightOnly::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    BaseTask::Type man_type = task->getType();
    if (man_type == BaseTask::Type::turn_left) {
        return true;

    } else if (man_type == BaseTask::Type::turn_right) {
        return true;

    } else if (man_type == BaseTask::Type::straight) {
        return false;
    }
    return true;
}

// give way
bool GiveWay::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    if (task->getType() == BaseTask::Type::turn_right) {
        return false;
    }
    return state.isCarAhead() || state.isCarRight() || state.isCarLeft();
}

// Stop
bool Stop::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    if (task->getType() == BaseTask::Type::turn_right) {
        return false;
    }
    return state.isCarAhead() || state.isCarRight() || state.isCarLeft();
}

// crosswalk detection, slow down

bool CrossWalk::appliesInState(const EnvironmentState &state, BaseTask *task,
                               bool at_junction) const {
    return !at_junction;
}

bool CrossWalk::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    return state.isCrossWalkDetected();
}

void
CrossWalk::onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                            BaseTask *task) {
    if (state.isPersonDetected() || state.isChildDetected()) {
        constraint->activate(Speed::STOP, 8);
        constraint->setMaxDuration(15);
    } else {
        constraint->activate(static_cast<float>(task->getDrivingSpeed() * 0.7), 0.5);
    }
}

void CrossWalk::onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                                   const EnvironmentState &state, BaseTask *task) {
    if (state.isPersonDetected() || state.isChildDetected()) {
        constraint->extend(Speed::STOP, 8);
        constraint->setMaxDuration(15);
    } else {
        constraint->extend(static_cast<float>(task->getDrivingSpeed() * 0.7), 0.5);
    }
}


// person detection / slow down
bool Person::appliesInState(const EnvironmentState &state, BaseTask *task,
                            bool at_junction) const {
    return !at_junction;
}

bool Person::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    return state.isPersonDetected();
}

void
Person::onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                         BaseTask *task) {
    constraint->activate(static_cast<float>(task->getDrivingSpeed() * 0.9), 3);
}

void
Person::onExtendConstraint(shared_ptr<DrivingConstraint> &constraint, const EnvironmentState &state,
                           BaseTask *task) {
    constraint->extend(1);
}


// child detection / slow down

bool Child::appliesInState(const EnvironmentState &state, BaseTask *task,
                           bool at_junction) const {
    return !at_junction;
}

bool Child::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    return state.isChildDetected();
}

void Child::onInitConstraint(shared_ptr<DrivingConstraint> constraint,
                             const EnvironmentState &state,
                             BaseTask *task) {
    constraint->activate(static_cast<float>(task->getDrivingSpeed() * 0.4), 3);
}

void
Child::onExtendConstraint(shared_ptr<DrivingConstraint> &constraint, const EnvironmentState &state,
                          BaseTask *task) {
    constraint->extend(1);
}


// overtake detection / slow down, overtake if too close

bool Overtake::appliesInState(const EnvironmentState &state, BaseTask *task,
                              bool at_junction) const {
    return !at_junction
    && isConstraintValid(state)
    && task->getName() != "Overtake";
}

bool Overtake::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    return isConstraintValid(state);
}

bool Overtake::isConstraintValid(const EnvironmentState &state) const {
    return state.isCarInPlannedPath();
    // Obstacle *closest_car = getClosestCar(state);
    // return isOvertakeTargetInLocalWindow(state, closest_car);
    //     || state.getProbCarAhead() >= MIN_PROB_CAR_AHEAD;
}

// bool Overtake::isOvertakeTargetInLocalWindow(const EnvironmentState &state, Obstacle* closest_car) const {
//     if (!closest_car) {
//         return false;
//     }

//     Point local_closest = state.getCarModel()->toLocal(*closest_car);
//     return fabs(local_closest.getX()) < 20 && state.getCarModel()->getFrontAxis().distanceTo(*closest_car) < MIN_DIST_RULE_INIT;
// }


Obstacle* Overtake::getClosestCar(const EnvironmentState &state) const {
    vector<Obstacle> cars;
    if (!state.getObstacles(Obstacle::Type::CAR, &cars)) {
        return nullptr;
    }
    Point frontAxis = state.getCarModel()->getFrontAxis();
    Obstacle *closest = nullptr;
    float closest_dist = -1;
    for (auto &car : cars) {
        auto distToCar = frontAxis.distanceTo(car);
        if (closest_dist == -1 || distToCar < closest_dist) {
            closest = &car;
            closest_dist = distToCar;
        }
    }
    return closest;
}

void Overtake::onInitConstraint(shared_ptr<DrivingConstraint> constraint, const EnvironmentState &state,
                           BaseTask *task) {

    // Obstacle *closest_car = getClosestCar(state);
    // if (closest_car == nullptr) {
    //     return;
    // }
    // if (!isOvertakeTargetInLocalWindow(state, closest_car)) {
    //     constraint.reset();
    //     return;
    // }
    if (!state.isCarInPlannedPath()) {
        constraint.reset();
        return;
    }
    // auto closest_dist = state.getCarModel()->getFrontAxis().distanceTo
    //         (*closest_car);
    // float speed = task->getDrivingSpeed();
    // LOG_INFO("Dist of closest obs : %f", closest_dist);
    // if (closest_dist <= MIN_DIST_INIT_OVERTAKE) {
    //     // 
    //     // speed = Speed::STOP;
    // } else {
    // }

    float speed = task->getDrivingSpeed();
    speed = static_cast<float>(speed * 0.7);

    constraint->activate(speed, 0.5);
    constraint->setMaxDuration(2);
    constraint->setMaxWaitOverride(ConstraintOverride(ConstraintOverride::Type::INIT_OVERTAKE));
}


void Overtake::onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                                  const EnvironmentState &state, BaseTask *task) {
    Obstacle *closest_car = getClosestCar(state);
    if (!closest_car) {
        if (isConstraintValid(state)) {
            constraint->extend(0.5);
        }
        return;
    }
    auto closest_dist = static_cast<float>(state.getCarModel()->getFrontAxis().distanceTo
            (*closest_car));
    float speed = task->getDrivingSpeed();
    LOG_INFO("extend ot constraint, closest dist: %f", closest_dist);
    if (closest_dist <= MIN_DIST_INIT_OVERTAKE) {
        // speed = Speed::STOP;
        constraint->setMaxDuration(1);
    } else {
        LOG_INFO("go slow");
        constraint->setMaxDuration(20);
    }
    speed = static_cast<float>(speed * 0.7);
    
    constraint->extend(speed, 1);
    constraint->setMaxWaitOverride(ConstraintOverride(ConstraintOverride::Type::INIT_OVERTAKE));
}

bool ConstraintOverride::apply(float *speed, deque<Point> *wps, BaseTask *task) {

    switch (overrideType) {
        case INIT_OVERTAKE:
            task->requestOvertake();
            break;

        case SLOW_MOVE:
            *speed = static_cast<float>(task->getDrivingSpeed() * 0.7);
            break;

        case REVERSE:
            task->doReverseEmBrakeRecovery();
            return true;
    }

    return false;
}



// Emergency brake
bool EmergencyBrakeRecovery::isConstraintRequired(const EnvironmentState &state, BaseTask *task) {
    return state.isEmergencyBrakeActive()
    || (task->getEmergencyBrakeCtrl()->isEnabled() && !task->getEmergencyBrakeCtrl()->getLastTriggerWatch()->didSecondsPass(2));
}

void EmergencyBrakeRecovery::onInitConstraint(shared_ptr<DrivingConstraint> constraint,
                                              const EnvironmentState &state, BaseTask *task) {
    constraint->setMaxDuration(3);
    constraint->setMaxWaitOverride(ConstraintOverride(ConstraintOverride::Type::REVERSE));
    constraint->activate(Speed::STOP, 3);

}

bool EmergencyBrakeRecovery::appliesInState(const EnvironmentState &state, BaseTask *task,
                                            bool at_junction) const {
    return state.isEmergencyBrakeActive()
    || (task->getEmergencyBrakeCtrl()->isEnabled() && !task->getEmergencyBrakeCtrl()->getLastTriggerWatch()->didSecondsPass(2));
}

void EmergencyBrakeRecovery::onExtendConstraint(shared_ptr<DrivingConstraint> &constraint,
                                                const EnvironmentState &state, BaseTask *task) {
    constraint->extend(1);

}
