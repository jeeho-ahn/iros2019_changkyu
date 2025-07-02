#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <iostream>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// A trivial validity checker: everything in bounds is valid.
// Replace this with obstacle checks as needed.
bool isStateValid(const ob::State * /*state*/)
{
    return true;
}

int main()
{
    // 1) Create a Dubins (forward‐only, nonholonomic) state space
    double turningRadius = 1.0;
    ob::StateSpacePtr space = std::make_shared<ob::DubinsStateSpace>(turningRadius);

    // 2) Set bounds on the (x,y) part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(10);
    space->as<ob::DubinsStateSpace>()->setBounds(bounds);

    // 3) Create SimpleSetup
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(isStateValid);

    // 4) Define start and goal states
    ob::ScopedState<> start(space), goal(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(1.0);
    start->as<ob::SE2StateSpace::StateType>()->setY(1.0);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0.0);

    goal->as<ob::SE2StateSpace::StateType>()->setX(9.0);
    goal->as<ob::SE2StateSpace::StateType>()->setY(9.0);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(M_PI/2.0);

    ss.setStartAndGoalStates(start, goal);

    // 5) Attach RRT* as the planner
    auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    planner->setGoalBias(0.1);  // optional tuning
    planner->setRange(0.5);     // optional tuning
    ss.setPlanner(planner);

    // 6) Final setup and solve
    ss.setup();
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Solution found:\n";
        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No solution found within the time limit.\n";
    }

    return 0;
}
