/*

#include "OmplPlanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


vector<Point> RRTPlanner::getPlan(LocalMap &local_map, CarPosition *startPosition, Point
goalPosition, bool applyHeadingCost) {
// Construct the robot state space in which we're planning. We're
// planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

// Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, local_map.window_height);
// Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
// Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si,
                                                                                &local_map)));
    si->setup();
// Set our robot's starting state to be the bottom-left corner of
// the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = startPosition->getX();
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = startPosition->getY();
// Set our robot's goal state to be the top-right corner of the
// environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goalPosition.getX();
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goalPosition.getY();
// Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setOptimizationObjective(std::make_shared<ClearanceObjective>(si, &local_map));
// Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    //Next, we want to define an ompl::base::OptimizationObjective for optimal planning. For now,
    //we can specify an objective that corresponds to finding the shortest path between the start
    //and goal states. We'll define another function that returns this particular objective:



    // Construct our optimizing planner using the RRTstar algorithm.
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));

// Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();
// attempt to solve the planning problem within one second of
// planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(0.05);
    if (!solved) {
        cout << "not solved" << endl;
        return vector<Point>();
    }

    auto states = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())
            ->getStates();

    std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())
            ->printAsMatrix(cout);

    vector<Point> waypoints;
    waypoints.reserve(states.size());
    cout << states.size() << "waypoints" << endl;
    for (auto &s : states) {

        const auto *state2D = s->as<ob::RealVectorStateSpace::StateType>();
        auto x = static_cast<float>(state2D->values[0]);
        auto y = static_cast<float>(state2D->values[1]);
        Point point = Point::Global(x, y, 0.0f);
        cout << point << endl;
        waypoints.emplace_back(point);
    }

    return waypoints;
};

*/