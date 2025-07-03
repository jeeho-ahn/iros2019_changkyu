#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <iostream>
#include <vector>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Oriented rectangle obstacle
struct OrientedBox {
    double cx, cy;   // center
    double yaw;      // orientation (radians)
    double halfW, halfH; // half width, half height

    OrientedBox(double x, double y, double theta, double w, double h)
      : cx(x), cy(y), yaw(theta), halfW(w/2.0), halfH(h/2.0) {}

    bool contains(double x, double y) const {
        // translate into box frame
        double dx = x - cx;
        double dy = y - cy;
        // rotate by -yaw
        double c = std::cos(-yaw), s = std::sin(-yaw);
        double lx = c*dx - s*dy;
        double ly = s*dx + c*dy;
        // check axis-aligned bounds in local frame
        return std::abs(lx) <= halfW && std::abs(ly) <= halfH;
    }
};

int main()
{
    // 1) Dubins space
    double turningRadius = 1.41;
    //auto space = std::make_shared<ob::DubinsStateSpace>(turningRadius);
    ompl::base::StateSpacePtr space(new ompl::base::DubinsStateSpace);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0); bounds.setHigh(10);
    space->as<ob::DubinsStateSpace>()->setBounds(bounds);

    // 2) SimpleSetup
    og::SimpleSetup ss(space);

    // 3) Define a list of oriented boxes
    std::vector<OrientedBox> obstacles;
    // e.g. center=(3,3), yaw=45°, width=2, height=1
    obstacles.emplace_back(3.0, 3.0, M_PI/4, 2.0, 1.0);
    // another: center=(7,5), yaw=0°, width=1, height=3
    obstacles.emplace_back(7.0, 6.0, 0.0, 1.0, 3.0);

    // 4) State validity checks both bounds and obstacles
    ss.setStateValidityChecker(
      [&](const ob::State* state) {
        const auto *se2 = state->as<ob::SE2StateSpace::StateType>();
        double x = se2->getX();
        double y = se2->getY();
        // check world bounds
       // if (!bounds.satisfies({x,y}))
       //     return false;
        // check each obstacle
        for (auto& box : obstacles)
            if (box.contains(x,y))
                return false;
        return true;
      }
    );

    // 5) Start & goal
    ob::ScopedState<> start(space), goal(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(1);
    start->as<ob::SE2StateSpace::StateType>()->setY(1);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0);
    goal->as<ob::SE2StateSpace::StateType>()->setX(9);
    goal->as<ob::SE2StateSpace::StateType>()->setY(9);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(M_PI/2);
    ss.setStartAndGoalStates(start, goal,0.1);

    // 6) RRT* planner
    auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    planner->setGoalBias(0.1);
    planner->setRange(0.5);
    ss.setPlanner(planner);

    // 7) Solve
    ss.setup();
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved) {
        std::cout << "Solution found:\n";
        ss.getSolutionPath().printAsMatrix(std::cout);
    } else {
        std::cout << "No solution found.\n";
    }


    // 9) Print obstacle poses
     std::cout << "\nObstacles (" << obstacles.size() << "):\n";
     for (size_t i = 0; i < obstacles.size(); ++i)
     {
         const auto& b = obstacles[i];
         std::cout
             << "  [" << i << "] Center=(" << b.cx << ", " << b.cy << "), "
             << "Yaw=" << b.yaw << " rad, "
             << "Width=" << (b.halfW * 2.0) << ", "
             << "Height=" << (b.halfH * 2.0)
             << "\n";
     }

    return 0;
}
