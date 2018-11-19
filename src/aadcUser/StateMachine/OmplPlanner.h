/*//
// Created by aadc on 17.09.18.
//

#pragma once
#ifndef AADC_USER_RRTPLANNER_H
#define AADC_USER_RRTPLANNER_H

//
// Created by aadc on 17.09.18.
//

// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".

// For ompl::msg::setLogLevel



// The supported optimal planners, in alphabetical order


#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include "ompl/util/Console.h"
#include <vector>
#include "string"
#include <algorithm>
#include <cstdio>
#include <deque>
#include <iostream>
#include <math.h>
#include "chrono"
#include "PrintUtils.h"
#include "utility"
#include "functional"
#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <unordered_set>
#include "CostMap.h"
#include "LocalMap.h"
#include "IPlanner.h"


class RRTPlanner : public IPlanner {
protected:

    class ValidityChecker : public ompl::base::StateValidityChecker {
    public:
        LocalMap *map;

        ValidityChecker(const ompl::base::SpaceInformationPtr &si, LocalMap *map) :
                ompl::base::StateValidityChecker(si), map(map) {}

        // Returns whether the given state's position overlaps the
        // circular obstacle
        bool isValid(const ompl::base::State *state) const override {
            return true;
        }
    };

    class ClearanceObjective : public ompl::base::StateCostIntegralObjective {
    public:
        LocalMap *map;

        ClearanceObjective(const ompl::base::SpaceInformationPtr &si, LocalMap *map) :
                ompl::base::StateCostIntegralObjective(si, true), map(map) {
        }

        ompl::base::Cost stateCost(const ompl::base::State *s) const override {
            const auto *state2D = s->as<ompl::base::RealVectorStateSpace::StateType>();
            double x = state2D->values[0];
            double y = state2D->values[1];

            return ompl::base::Cost(
                    map->costAt(static_cast<int>(floor(x)), static_cast<int>(floor(y))));
        }
    };

    vector<Point> getPlan(LocalMap &local_map, CarPosition *startPosition,
                          Point goalPosition, bool applyHeadingCost) override;


};

#endif //AADC_USER_RRTPLANNER_H
*/