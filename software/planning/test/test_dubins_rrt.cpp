#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <iostream>
#include <vector>
#include <cmath>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/*
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
*/
// simple 2D vector
struct Vec2 {
    double x, y;
    Vec2(double _x=0, double _y=0): x(_x), y(_y) {}
};
inline double dot(const Vec2 &a, const Vec2 &b) { return a.x*b.x + a.y*b.y; }

/// Oriented rectangle (centered at cx,cy, rotated by yaw)
/// halfLen: half‐length along the local x‐axis (forward/backward)
/// halfWid: half‐width along local y‐axis (sideways)
struct OrientedRect {
    double cx, cy, yaw, halfLen, halfWid;

    OrientedRect(double _cx, double _cy, double _yaw, double fullLen, double fullWid)
      : cx(_cx), cy(_cy), yaw(_yaw),
        halfLen(fullLen/2.0), halfWid(fullWid/2.0)
    {}

    // SAT‐based test: do these two oriented rectangles intersect?
    bool intersects(const OrientedRect &o) const {
        // local axes for each rect
        Vec2 ux1(std::cos(yaw), std::sin(yaw)), uy1(-ux1.y, ux1.x);
        Vec2 ux2(std::cos(o.yaw), std::sin(o.yaw)), uy2(-ux2.y, ux2.x);
        // vector between centers
        Vec2 d(o.cx - cx, o.cy - cy);

        auto overlapOnAxis = [&](const Vec2 &axis,
                                 const Vec2 &uaxA, const Vec2 &uayA,
                                 double hAxA, double hAyA,
                                 const Vec2 &uaxB, const Vec2 &uayB,
                                 double hAxB, double hAyB){
            double rA = hAxA * std::fabs(dot(axis, uaxA))
                      + hAyA * std::fabs(dot(axis, uayA));
            double rB = hAxB * std::fabs(dot(axis, uaxB))
                      + hAyB * std::fabs(dot(axis, uayB));
            double dist = std::fabs(dot(axis, d));
            return dist <= (rA + rB);
        };

        // test all four separating axes
        return overlapOnAxis(ux1, ux1, uy1, halfLen, halfWid, ux2, uy2, o.halfLen, o.halfWid)
            && overlapOnAxis(uy1, ux1, uy1, halfLen, halfWid, ux2, uy2, o.halfLen, o.halfWid)
            && overlapOnAxis(ux2, ux1, uy1, halfLen, halfWid, ux2, uy2, o.halfLen, o.halfWid)
            && overlapOnAxis(uy2, ux1, uy1, halfLen, halfWid, ux2, uy2, o.halfLen, o.halfWid);
    }
};

int main()
{
    const double robotWidth    = 0.3;  // total width (side to side)
    const double robotForward  = 0.4;  // length in front of your ref point
    const double robotBackward = 0.2;  // length behind your ref point
    // 1) Dubins space
    double turningRadius = 1.41;
    //auto space = std::make_shared<ob::DubinsStateSpace>(turningRadius);
    ob::StateSpacePtr space =
        std::make_shared<ob::DubinsStateSpace>(turningRadius);
    ob::RealVectorBounds bounds(2);
    // 2) Set x limits (dimension 0)
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, 4.0);

    // 3) Set y limits (dimension 1)
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, 5.2);

    // extract for fast bounds checks
    std::vector<float> low  = {0,0};
    std::vector<float> high = {4,5.2};
    space->as<ob::DubinsStateSpace>()->setBounds(bounds);

    // 2) SimpleSetup
    og::SimpleSetup ss(space);

    // 3) Define a list of oriented boxes
    std::vector<OrientedRect> obstacles;
    // e.g. center=(3,3), yaw=45°, width=2, height=1
    obstacles.emplace_back(2, 2, M_PI/4, 0.3, 0.2);
    // another: center=(7,5), yaw=0°, width=1, height=3
    obstacles.emplace_back(3.0, 1.0, 0.0, 0.3, 0.3);
    obstacles.emplace_back(2.2, 1.7, 0.0, 0.3, 0.3);

    // 4) State validity checks both bounds and obstacles
    /*
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
    */

    ss.setStateValidityChecker(
        [&](const ob::State* s) {
            const auto *se2 = s->as<ob::SE2StateSpace::StateType>();
            double x   = se2->getX();
            double y   = se2->getY();
            double yaw = se2->getYaw();

            // compute the geometric center of the robot box
            double halfLen     = (robotForward + robotBackward)/2.0;
            double centerShift = (robotForward - robotBackward)/2.0;
            double cx = x + centerShift * std::cos(yaw);
            double cy = y + centerShift * std::sin(yaw);

            // build the robot’s oriented rect
            OrientedRect robot(cx, cy, yaw,
                               robotForward + robotBackward,
                               robotWidth);

            // quick world‐bounds check via projections onto X/Y
            Vec2 ux(std::cos(yaw), std::sin(yaw)),
                 uy(-ux.y, ux.x);
            double ex = robot.halfLen * std::fabs(dot(ux, Vec2(1,0)))
                      + robot.halfWid * std::fabs(dot(uy, Vec2(1,0)));
            double ey = robot.halfLen * std::fabs(dot(ux, Vec2(0,1)))
                      + robot.halfWid * std::fabs(dot(uy, Vec2(0,1)));
            if (cx - ex < low[0] || cx + ex > high[0] ||
                cy - ey < low[1] || cy + ey > high[1])
                return false;

            // test against every obstacle
            for (auto &obs : obstacles)
                if (robot.intersects(obs))
                    return false;

            return true;
        }
    );

    // 5) Start & goal
    ob::ScopedState<> start(space), goal(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(1);
    start->as<ob::SE2StateSpace::StateType>()->setY(1);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0);

    goal->as<ob::SE2StateSpace::StateType>()->setX(3);
    goal->as<ob::SE2StateSpace::StateType>()->setY(4.5);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(M_PI/2);
    ss.setStartAndGoalStates(start, goal,0.01); // m

    // 6) RRT* planner
    auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    planner->setGoalBias(0.2); // 20% of samples try the goal directly
    planner->setRange(0.5);
    ss.setPlanner(planner);

    // 7) Solve
    ss.setup();
    ob::PlannerStatus solved = ss.solve(2.0);  // sec

    // 9) Emit JSON
    std::cout << "{\n";
    // robot info
    std::cout << "  \"robot\": {"
              << "\"width\":"    << robotWidth    << ","
              << "\"forward\":"  << robotForward  << ","
              << "\"backward\":" << robotBackward
              << "},\n";
    // obstacles
    std::cout << "  \"obstacles\": [\n";
    for (size_t i = 0; i < obstacles.size(); ++i) {
        auto &b = obstacles[i];
        std::cout << "    {"
                  << "\"cx\":"   << b.cx    << ","
                  << "\"cy\":"   << b.cy    << ","
                  << "\"yaw\":"  << b.yaw   << ","
                  << "\"width\":"  << (b.halfLen*2.0) << ","
                  << "\"height\":" << (b.halfWid*2.0)
                  << "}" << (i+1<obstacles.size() ? "," : "") << "\n";
    }
    std::cout << "  ],\n";
    // solution flag
    std::cout << "  \"solutionFound\": " << (solved ? "true" : "false") << ",\n";
    // path
    std::cout << "  \"path\": [\n";
    if (solved==ompl::base::PlannerStatus::EXACT_SOLUTION) {
        auto path = ss.getSolutionPath();
        for (size_t i = 0; i < path.getStateCount(); ++i) {
            const auto *st = path.getState(i)->as<ob::SE2StateSpace::StateType>();
            std::cout << "    ["
                      << st->getX()   << ","
                      << st->getY()   << ","
                      << st->getYaw() << "]"
                      << (i+1<path.getStateCount() ? "," : "") << "\n";
        }
    }
    std::cout << "  ]\n";
    std::cout << "}\n";

    return 0;
}
