// src/planner_helpers.cpp
#include "planner.hpp"
#include <unordered_map>
#include <algorithm>
#include <limits>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

//-----------------------------------------------------------------------------
// 1) findBestDubins
bool Planner::findBestDubins(int o,
                             const ObjectState* s0,
                             const ObjectState* s1,
                             double turning_rad,
                             reloDubinsPath &bestDubins) const
{
    std::vector<double> yaws_start = {
        s0->getYaw(),
        s0->getYaw()+M_PI_2,
        s0->getYaw()+M_PI,
        s0->getYaw()+3*M_PI_2
    };
    std::vector<double> yaws_goal = {
        s1->getYaw(),
        s1->getYaw()+M_PI_2,
        s1->getYaw()+M_PI,
        s1->getYaw()+3*M_PI_2
    };

    double best_len = std::numeric_limits<double>::infinity();
    //reloDubinsPath cand(0);

    for(double y0 : yaws_start){
        ReloPush::State ds0(s0->getX(), s0->getY(), y0);
        for(double y1 : yaws_goal){
            ReloPush::State ds1(s1->getX(), s1->getY(), y1);
            auto path = findDubins(ds0, ds1, turning_rad, false);

            if(path.omplDubins.length()==std::numeric_limits<double>::max())
                continue;
            double L = path.lengthCost();
            if(L < best_len){
                best_len   = L;
                bestDubins = path;
            }
        }
    }
    return best_len < std::numeric_limits<double>::infinity();
}

//-----------------------------------------------------------------------------
// 2) recordCollisions
void Planner::recordCollisions(int o,
                               const ReloPush::StatePathPtr &interp,
                               ob::State *state_curr,                        // ← now provided
                               std::vector<int> &idxes_collide,
                               std::unordered_map<int,ReloPush::State> &collision_pose)
{
    // save original params
    auto param_org = env_.getParamSingleForAll();

    // scratch state for probing
    ob::State *state_midd = si_single4all_->allocState();

    for (const auto &wp : *interp)
    {
        // write the waypoint into state_midd
        state_midd->as<ObjectState>()->setX(wp.x);
        state_midd->as<ObjectState>()->setY(wp.y);
        state_midd->as<ObjectState>()->setYaw(wp.yaw);

        // test against each other object
        for (int c = 1; c <= n_objs_; ++c)
        {
            if (c == o) continue;

            // freeze object c *and* replay the selfish path from state_curr
            std::vector<int> tmp{c};
            env_.setParamSingleForAll(o, tmp, state_curr);

            // if this probe collides—and we haven’t recorded c yet—
            if (!si_single4all_->isValid(state_midd)
                && collision_pose.count(c) == 0)
            {
                idxes_collide.push_back(c);
                collision_pose[c] = ReloPush::State(wp.x, wp.y, wp.yaw);
            }
        }
    }

    // cleanup
    si_single4all_->freeState(state_midd);
    env_.setParamSingleForAll(param_org);
}

//-----------------------------------------------------------------------------
// 3) doClearance

bool Planner::doClearance(int o,
                          const std::vector<int> &idxes_collide,
                          const std::unordered_map<int,ReloPush::State> &collision_pose,
                          ob::State *state_curr,
                          const ReloPush::StatePathPtr &interp,  // <<--- interp in
                          og::PathGeometric &path_tmp)
{

    //auto deb = STATE_OBJECT(state_curr,3)->getYaw();
    // Inject the recorded collision poses into state_curr
    /*
    for (int c : idxes_collide) {
        auto it = collision_pose.find(c);
        if (it != collision_pose.end()) {
            ObjectState* so = STATE_OBJECT(state_curr, c);
            so->setX(it->second.x);
            so->setY(it->second.y);
            so->setYaw(it->second.yaw);
        }
    }

    */
    // Call clearObstacles with selfish_path
    return clearObstacles(idxes_collide, o, interp, state_curr, path_tmp);


}

bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             const ReloPush::StatePathPtr &interp,
                             ob::State* state_curr,
                             og::PathGeometric& path_tmp)
{
    auto param_org = env_.getParamSingleForAll();

    for (int c : idxes_collide)
    {
        ObjectState* state_c = STATE_OBJECT(state_curr, c);
        double yaw = state_c->getYaw();
        double step_size = 0.05;  // sampling step size
        int max_steps = 40;       // max distance to try (2m total here)

        ob::ProblemDefinitionPtr pdef_clear(new ob::ProblemDefinition(si_single4clear_));
        pdef_clear->setOptimizationObjective(opt_inf);
        pdef_clear->setGoalState(state_c);

        ob::State* state_candidate = si_single4clear_->allocState();

        int valid_starts = 0;
        for (int step = 1; step <= max_steps && valid_starts < 10; ++step)
        {
            double dist = step * step_size;
            double candidate_x = state_c->getX() + dist * cos(yaw);
            double candidate_y = state_c->getY() + dist * sin(yaw);

            state_candidate->as<ObjectState>()->setX(candidate_x);
            state_candidate->as<ObjectState>()->setY(candidate_y);
            state_candidate->as<ObjectState>()->setYaw(yaw);

            if (!si_single4clear_->getStateSpace()->satisfiesBounds(state_candidate))
                continue;

            if (!si_single4clear_->isValid(state_candidate))
                continue;

            // check collision with interp path
            bool collision_with_interp = false;
            double collision_threshold = 0.3; // set appropriately
            for (const auto &wp : *interp)
            {
                double dist_to_wp = sqrt(pow(candidate_x - wp.x, 2) + pow(candidate_y - wp.y, 2));
                if (dist_to_wp < collision_threshold)
                {
                    collision_with_interp = true;
                    break;
                }
            }

            if (!collision_with_interp)
            {
                pdef_clear->addStartState(state_candidate);
                ++valid_starts;
            }
        }

        si_single4clear_->freeState(state_candidate);

        if (valid_starts == 0)
        {
            env_.setParamSingleForAll(param_org);
            return false;
        }

        og::RRTstar planner_clear(si_single4clear_);
        planner_clear.setRange(1);
        planner_clear.setProblemDefinition(pdef_clear);
        planner_clear.setup();
        ob::PlannerStatus solved = planner_clear.solve(ob::timedPlannerTerminationCondition(0.33));

        auto geom_path_clear = static_cast<og::PathGeometric*>(pdef_clear->getSolutionPath().get());

        if (!solved || !geom_path_clear ||
            si_single4clear_->distance(geom_path_clear->getStates().back(), state_c) >= thresh_goal)
        {
            env_.setParamSingleForAll(param_org);
            return false;
        }

        auto &path_clear = *geom_path_clear;
        STATE_ROBOT(state_curr) = c;
        path_tmp.append(state_curr);
        for (int p = path_clear.getStateCount() - 1; p >= 0; --p)
        {
            auto sp = path_clear.getState(p)->as<ObjectState>();
            state_c->setX(sp->getX());
            state_c->setY(sp->getY());
            state_c->setYaw(sp->getYaw());
            path_tmp.append(state_curr);
        }
    }

    env_.setParamSingleForAll(param_org);
    return true;
}


/*

bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             const ReloPush::StatePathPtr& interp,
                             ob::State* state_curr,
                             og::PathGeometric& path_tmp)
{
    ompl::base::StateSamplerPtr ss_single = si_single4clear_->allocStateSampler();

    std::vector<std::pair<int, ObjectState*>> obstacles;
    for(int oo = 1; oo <= n_objs_; ++oo)
        obstacles.emplace_back(oo, STATE_OBJECT(state_curr, oo));

    double collision_threshold = 0.3;

    for(int c : idxes_collide)
    {
        if(path_tmp.getStateCount() == 0)
            path_tmp.append(state_curr);

        env_.setParamSingleForClear(c, o, path_tmp, obstacles);
        ObjectState* state_c = STATE_OBJECT(state_curr, c);

        ob::ProblemDefinitionPtr pdef_clear(new ob::ProblemDefinition(si_single4clear_));
        pdef_clear->setOptimizationObjective(opt_inf);
        pdef_clear->setGoalState(state_c);

        ob::State* state_clear = si_single4clear_->allocState();
        int valid_starts = 0;
        int attempts = 0;
        while(valid_starts < 50 && attempts++ < 300)
        {
            ss_single->sampleUniform(state_clear);
            if(!si_single4clear_->isValid(state_clear)) continue;

            // check collision explicitly with interp path
            bool collide_with_interp = false;
            ObjectState* clear_os = state_clear->as<ObjectState>();

            for(const auto &wp : *interp)
            {
                double dx = wp.x - clear_os->getX();
                double dy = wp.y - clear_os->getY();
                double d = sqrt(dx*dx + dy*dy);
                if(d < collision_threshold) // set this threshold appropriately
                {
                    collide_with_interp = true;
                    break;
                }
            }

            if(collide_with_interp)
                continue;

            pdef_clear->addStartState(state_clear);
            ++valid_starts;
        }

        si_single4clear_->freeState(state_clear);

        if(valid_starts == 0)
        {
            std::cout << "No valid clearance start states for object " << c << std::endl;
            return false;
        }

        og::RRTstar planner_clear(si_single4clear_);
        planner_clear.setRange(0.3);
        planner_clear.setProblemDefinition(pdef_clear);
        planner_clear.setup();

        ob::PlannerStatus solved = planner_clear.solve(ob::timedPlannerTerminationCondition(0.33));

        auto geom_path_clear = static_cast<og::PathGeometric*>(pdef_clear->getSolutionPath().get());

        if(!solved || !geom_path_clear || si_single4clear_->distance(
            geom_path_clear->getStates().back(), state_c) >= thresh_goal)
        {
            std::cout << "Clearance failed/incomplete for object " << c << std::endl;
            return false;
        }

        auto& path_clear = *geom_path_clear;
        STATE_ROBOT(state_curr) = c;
        path_tmp.append(state_curr);
        for(int p = path_clear.getStateCount()-1; p >= 0; --p)
        {
            auto sp = path_clear.getState(p)->as<ObjectState>();
            state_c->setX(sp->getX());
            state_c->setY(sp->getY());
            state_c->setYaw(sp->getYaw());
            path_tmp.append(state_curr);
        }
    }

    return true;
}




*/






/*
bool Planner::doClearance(int o,
                          const std::vector<int> &idxes_collide,
                          const std::unordered_map<int,ReloPush::State> &collision_pose,
                          ob::State *state_curr,
                          const og::PathGeometric &selfish_path,   // now used
                          og::PathGeometric &path_tmp)
{
    // Inject the recorded collision poses into state_curr
    for (int c : idxes_collide) {
        auto it = collision_pose.find(c);
        if (it != collision_pose.end()) {
            ObjectState* so = STATE_OBJECT(state_curr, c);
            so->setX(it->second.x);
            so->setY(it->second.y);
            so->setYaw(it->second.yaw);
        }
    }
    // Call clearObstacles with selfish_path
    return clearObstacles(idxes_collide, o, state_curr, selfish_path, path_tmp);
}

//-----------------------------------------------------------------------------

bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             ob::State* state_curr,
                             const og::PathGeometric &selfish_path,
                             og::PathGeometric& path_tmp)
{
    std::cout << "*debug yaw3-2*: " << STATE_OBJECT(state_curr,3)->getYaw() << std::endl;


    const double clearance_margin = 0.05;    // 5 cm safety buffer

    // 1) sampler for clearance starts
    ompl::base::StateSamplerPtr ss_single = si_single4clear_->allocStateSampler();

    // 2) collect the current obstacle poses
    std::vector<std::pair<int,ObjectState*>> obstacles;
    for(int oo = 1; oo <= n_objs_; ++oo)
        obstacles.emplace_back(oo, STATE_OBJECT(state_curr, oo));

    std::cout << "*debug yaw3-3*: " << STATE_OBJECT(state_curr,3)->getYaw() << std::endl;

    // 3) for each collided object, plan a reverse‐clear path
    for(int c : idxes_collide)
    {
        if(path_tmp.getStateCount() == 0)
            path_tmp.append(state_curr);

        // replay the selfish push path
        env_.setParamSingleForClear(c, o, selfish_path, obstacles);

        ObjectState* state_c = STATE_OBJECT(state_curr, c);

        // set up RRT* for clearance
        auto pdef_clear = std::make_shared<ob::ProblemDefinition>(si_single4clear_);
        pdef_clear->setOptimizationObjective(opt_inf);
        pdef_clear->setGoalState(state_c);

        // sample valid clearance start states
        ob::State* state_clear = si_single4clear_->allocState();
        int valid_starts = 0;
        while(valid_starts < 50)
        {
            ss_single->sampleUniform(state_clear);
            // (a) must be collision‐free per OMPL
            if(!si_single4clear_->isValid(state_clear))
                continue;

            // (b) must be far enough from every static obstacle
            {
                auto *os = state_clear->as<ObjectState>();
                double cx = os->getX(), cy = os->getY();
                bool too_close = false;
                for(auto &pr : obstacles)
                {
                    if(pr.first == c) continue;
                    double dx = cx - pr.second->getX();
                    double dy = cy - pr.second->getY();
                    if(std::hypot(dx, dy) < clearance_margin)
                    {
                        too_close = true;
                        break;
                    }
                }
                if(too_close) continue;
            }

            // (c) must be far enough from every point on the selfish path
            {
                auto *os = state_clear->as<ObjectState>();
                double cx = os->getX(), cy = os->getY();
                bool too_close = false;
                for(size_t pi = 0; pi < selfish_path.getStateCount(); ++pi)
                {
                    const ObjectState* ps =
                        selfish_path.getState(pi)->as<ObjectState>();
                    double dx = cx - ps->getX();
                    double dy = cy - ps->getY();
                    if(std::hypot(dx, dy) < clearance_margin)
                    {
                        too_close = true;
                        break;
                    }
                }
                if(too_close) continue;
            }

            // accept this as a valid start
            pdef_clear->addStartState(state_clear);
            ++valid_starts;
        }
        si_single4clear_->freeState(state_clear);

        if(valid_starts == 0)
            return false;

        // run RRT*
        og::RRTstar planner_clear(si_single4clear_);
        planner_clear.setRange(0.3);
        planner_clear.setProblemDefinition(pdef_clear);
        planner_clear.setup();
        ob::PlannerStatus solved = planner_clear.solve(
            ob::timedPlannerTerminationCondition(0.33));

        auto *geom_path_clear = static_cast<og::PathGeometric*>(
            pdef_clear->getSolutionPath().get());

        if(!solved ||
           !geom_path_clear ||
           si_single4clear_->distance(
             geom_path_clear->getStates().back(), state_c) >= thresh_goal)
        {
            return false;
        }

        // append the reverse‐clearance path into path_tmp
        STATE_ROBOT(state_curr) = c;
        path_tmp.append(state_curr);
        for(int p = geom_path_clear->getStateCount() - 1; p >= 0; --p)
        {
            auto *sp = geom_path_clear->getState(p)->as<ObjectState>();
            state_c->setX(sp->getX());
            state_c->setY(sp->getY());
            state_c->setYaw(sp->getYaw());
            path_tmp.append(state_curr);
        }
    }

    return true;
}
*/
/*
bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             ob::State* state_curr,
                             const og::PathGeometric &selfish_path,
                             og::PathGeometric& path_tmp)
{
    // 1) sampler for clearance starts
    auto ss_single = si_single4clear_->allocStateSampler();

    // 2) gather obstacle poses
    std::vector<std::pair<int,ObjectState*>> obstacles;
    for (int oo = 1; oo <= n_objs_; ++oo)
        obstacles.emplace_back(oo, STATE_OBJECT(state_curr, oo));

    // For each collided object, do reverse‐clear
    for (int c : idxes_collide)
    {
        if (path_tmp.getStateCount() == 0)
            path_tmp.append(state_curr);

        // *** HERE we replay the *selfish_path* ***
        env_.setParamSingleForClear(c, o, selfish_path, obstacles);

        ObjectState* state_c = STATE_OBJECT(state_curr, c);

        auto pdef_clear = std::make_shared<ob::ProblemDefinition>(si_single4clear_);
        pdef_clear->setOptimizationObjective(opt_inf);
        pdef_clear->setGoalState(state_c);

        // sample valid starts
        ob::State* state_clear = si_single4clear_->allocState();
        int valid_starts = 0;
        for (int i = 0; i < 300 && valid_starts < 50; ++i)
        {
            ss_single->sampleUniform(state_clear);
            if (si_single4clear_->isValid(state_clear))
            {
                pdef_clear->addStartState(state_clear);
                ++valid_starts;
            }
        }
        si_single4clear_->freeState(state_clear);

        if (valid_starts == 0)
            return false;

        // run RRT*
        og::RRTstar planner_clear(si_single4clear_);
        planner_clear.setRange(0.3);
        planner_clear.setProblemDefinition(pdef_clear);
        planner_clear.setup();
        auto solved = planner_clear.solve(
            ob::timedPlannerTerminationCondition(0.33));

        auto geom_path_clear = static_cast<og::PathGeometric*>(
            pdef_clear->getSolutionPath().get());

        if (!solved ||
            !geom_path_clear ||
            si_single4clear_->distance(
               geom_path_clear->getStates().back(), state_c) >= thresh_goal)
        {
            return false;
        }

        // append reverse‐clear path
        STATE_ROBOT(state_curr) = c;
        path_tmp.append(state_curr);
        for (int p = geom_path_clear->getStateCount()-1; p >= 0; --p)
        {
            auto *sp = geom_path_clear->getState(p)->as<ObjectState>();
            state_c->setX(sp->getX());
            state_c->setY(sp->getY());
            state_c->setYaw(sp->getYaw());
            path_tmp.append(state_curr);
        }
    }

    return true;
}
*/

/*
bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             ob::State* state_curr,
                             const og::PathGeometric &selfish_path,   // use this
                             og::PathGeometric& path_tmp)
{
    // 1) sampler for clearance starts
    auto ss_single = si_single4clear_->allocStateSampler();

    // 2) gather obstacle poses
    std::vector<std::pair<int,ObjectState*>> obstacles;
    for (int oo = 1; oo <= n_objs_; ++oo)
        obstacles.emplace_back(oo, STATE_OBJECT(state_curr, oo));

    // For each collided object, do reverse‐clear
    for (int c : idxes_collide)
    {
        if (path_tmp.getStateCount() == 0)
            path_tmp.append(state_curr);

        // *** HERE we replay the *selfish_path* ***
        env_.setParamSingleForClear(c, o, selfish_path, obstacles);

        ObjectState* state_c = STATE_OBJECT(state_curr, c);

        auto pdef_clear = std::make_shared<ob::ProblemDefinition>(si_single4clear_);
        pdef_clear->setOptimizationObjective(opt_inf);
        pdef_clear->setGoalState(state_c);

        // sample valid starts
        ob::State* state_clear = si_single4clear_->allocState();
        int valid_starts = 0;
        for (int i = 0; i < 300 && valid_starts < 50; ++i)
        {
            ss_single->sampleUniform(state_clear);
            if (si_single4clear_->isValid(state_clear))
            {
                pdef_clear->addStartState(state_clear);
                ++valid_starts;
            }
        }
        si_single4clear_->freeState(state_clear);

        if (valid_starts == 0)
            return false;

        // run RRT*
        og::RRTstar planner_clear(si_single4clear_);
        planner_clear.setRange(0.3);
        planner_clear.setProblemDefinition(pdef_clear);
        planner_clear.setup();
        auto solved = planner_clear.solve(
            ob::timedPlannerTerminationCondition(0.33));

        auto geom_path_clear = static_cast<og::PathGeometric*>(
            pdef_clear->getSolutionPath().get());

        if (!solved ||
            !geom_path_clear ||
            si_single4clear_->distance(
               geom_path_clear->getStates().back(), state_c) >= thresh_goal)
        {
            return false;
        }

        // append reverse‐clear path
        STATE_ROBOT(state_curr) = c;
        path_tmp.append(state_curr);
        for (int p = geom_path_clear->getStateCount()-1; p >= 0; --p)
        {
            auto *sp = geom_path_clear->getState(p)->as<ObjectState>();
            state_c->setX(sp->getX());
            state_c->setY(sp->getY());
            state_c->setYaw(sp->getYaw());
            path_tmp.append(state_curr);
        }
    }

    return true;
}
*/


/*
bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             ob::State* state_curr,
                             og::PathGeometric& path_tmp)
{
    // 1) sampler for clearance starts
    ompl::base::StateSamplerPtr ss_single = si_single4clear_->allocStateSampler();

    // 2) collect the current obstacle poses
    std::vector<std::pair<int,ObjectState*>> obstacles;
    for(int oo = 1; oo <= n_objs_; ++oo)
        obstacles.emplace_back(oo, STATE_OBJECT(state_curr, oo));

    // 3) for each collided object, plan a reverse‐clear path
    for(int c : idxes_collide)
    {
        // ensure there's at least one state to work from
        if(path_tmp.getStateCount() == 0)
            path_tmp.append(state_curr);

        // tell the environment which object to clear
        env_.setParamSingleForClear(c, o, path_tmp, obstacles);

        ObjectState* state_c = STATE_OBJECT(state_curr, c);

        // build the RRT* problem to clear object c
        ob::ProblemDefinitionPtr pdef_clear(new ob::ProblemDefinition(si_single4clear_));
        pdef_clear->setOptimizationObjective(opt_inf);
        pdef_clear->setGoalState(state_c);

        /////////////////////////////////////////////////////////////////////////////////////////////////

        // sample up to 50 valid starts
        ob::State* state_clear = si_single4clear_->allocState();
        int valid_starts = 0;
        for(int cc=0; cc<300 && valid_starts<50; ++cc)
        {
            ss_single->sampleUniform(state_clear);
            if(si_single4clear_->isValid(state_clear))
            {
                pdef_clear->addStartState(state_clear);
                ++valid_starts;
            }
        }
        si_single4clear_->freeState(state_clear);


        /////////////////////////////////////////////////////////////////////////////////////////////////

        // parameters
        float sampleIncrement = 0.05f;            // step size along each axis
        float maxShiftDist    = 1.0f;             // how far out to search
        int   maxSteps        = int(maxShiftDist / sampleIncrement);

        // count how many clearance‐start states we add
        int valid_starts = 0;

        // allocate one reusable state for sampling
        ob::State* state_clear = si_single4clear_->allocState();

        // get the object’s current yaw
        //ObjectState* state_c = STATE_OBJECT(state_curr, c);
        double defaultYaw = state_c->getYaw();

        // compute the four pushing‐axis orientations
        std::vector<double> orientations = {
            fromOMPL::mod2pi(defaultYaw +   0.0),
            fromOMPL::mod2pi(defaultYaw + M_PI_2),
            fromOMPL::mod2pi(defaultYaw + M_PI),
            fromOMPL::mod2pi(defaultYaw + 3*M_PI_2)
        };

        for(double yaw : orientations)
        {
            for(int step = 1; step <= maxSteps; ++step)
            {
                double dist  = step * sampleIncrement;
                double xNew  = state_c->getX() + dist * cos(yaw);
                double yNew  = state_c->getY() + dist * sin(yaw);

                auto *os = state_clear->as<ObjectState>();
                os->setX  (xNew);
                os->setY  (yNew);
                os->setYaw(yaw);

                // *** NEW: skip out‐of‐bounds samples immediately ***
                if (!si_single4clear_->getStateSpace()->satisfiesBounds(state_clear))
                    continue;

                if (si_single4clear_->isValid(state_clear))
                {
                    pdef_clear->addStartState(state_clear);
                    ++valid_starts;
                    break;
                }
            }
        }

        si_single4clear_->freeState(state_clear);

        /////////////////////////////////////////////////////////////////////////////////////////////////

        if(valid_starts == 0)
        {
            std::cout << "No valid clearance start states for object " << c << "\n";
            return false;
        }

        // run RRT*
        og::RRTstar planner_clear(si_single4clear_);
        planner_clear.setRange(0.3);
        planner_clear.setProblemDefinition(pdef_clear);
        planner_clear.setup();
        ob::PlannerStatus solved = planner_clear.solve(
            ob::timedPlannerTerminationCondition(0.33));

        auto geom_path_clear = static_cast<og::PathGeometric*>(
            pdef_clear->getSolutionPath().get());

        if(!solved || !geom_path_clear ||
           si_single4clear_->distance(
               geom_path_clear->getStates().back(),
               state_c) >= thresh_goal)
        {
            std::cout << "Clearance failed/incomplete for object " << c << "\n";
            return false;
        }

        // append reverse path
        auto& path_clear = *geom_path_clear;
        STATE_ROBOT(state_curr) = c;
        path_tmp.append(state_curr);
        for(int p = path_clear.getStateCount()-1; p>=0; --p)
        {
            auto sp = path_clear.getState(p)->as<ObjectState>();
            state_c->setX(sp->getX());
            state_c->setY(sp->getY());
            state_c->setYaw(sp->getYaw());
            path_tmp.append(state_curr);
        }
    }

    // all clearances succeeded
    return true;
}
*/

//-----------------------------------------------------------------------------
// 4) appendDubinsSegment
void Planner::appendDubinsSegment(int o,
                                  const ReloPush::StatePathPtr &interp,
                                  ob::State *state_curr,
                                  og::PathGeometric &path_tmp)
{
    // For every waypoint on the selfish Dubins path...
    for (const auto &wp : *interp)
    {
        // 1) set the robot‐index to object o
        STATE_ROBOT(state_curr) = o;

        // 2) overwrite exactly that object’s pose
        ObjectState* so = STATE_OBJECT(state_curr, o);
        so->setX  (wp.x);
        so->setY  (wp.y);
        so->setYaw(wp.yaw);

        // 3) append ONLY this new state
        path_tmp.append(state_curr);
    }
}

/*
void Planner::appendDubinsSegment(int o,
                                  const ReloPush::StatePathPtr &interp,
                                  ob::State *state_curr,
                                  og::PathGeometric &path_tmp)
{
    STATE_ROBOT(state_curr) = o;
    path_tmp.append(state_curr);
    ObjectState* so = STATE_OBJECT(state_curr,o);
    for(const auto &wp : *interp){
        so->setX(wp.x);
        so->setY(wp.y);
        so->setYaw(wp.yaw);
        path_tmp.append(state_curr);
    }
}
*/
