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
                             reloDubinsPath &bestDubins,
                             ReloPush::StatePathPtr& bestInterp,
                             double interpResolution,
                             PlanningContext &planCtx,
                             ReloPush::State& transit_start,
                             std::vector<ReloPush::StatePathPtr> transit_paths) const
{
    // 1) Precompute the 4×4 yaw combinations
    std::vector<double> yaws_start = {
        s0->getYaw(),
        s0->getYaw() + M_PI_2,
        s0->getYaw() + M_PI,
        s0->getYaw() + 3.0*M_PI_2
    };
    std::vector<double> yaws_goal = {
        s1->getYaw(),
        s1->getYaw() + M_PI_2,
        s1->getYaw() + M_PI,
        s1->getYaw() + 3.0*M_PI_2
    };

    // 2) Grab your workspace bounds (assumes env_.getWorkspaceBounds())
    //    Adjust to match your actual member or getter.

    const double xmin = 0, xmax = 4;  //todo: parse it from env settings
    const double ymin = 0, ymax = 5.2;

    double best_len = std::numeric_limits<double>::infinity();
    bool foundAny = false;

    // 3) Loop over all yaw-start / yaw-goal combos
    for (double y0 : yaws_start)
    {
        ReloPush::State ds0(s0->getX(), s0->getY(), y0);
        for (double y1 : yaws_goal)
        {
            ReloPush::State ds1(s1->getX(), s1->getY(), y1);
            // check if this pose is accessable
            auto ds0_prepush = ReloPush::find_pre_push(ds0,(planCtx.parameters.LF_push+planCtx.parameters.obs_rad)*1.01);
            auto ph = planHybridAstar(transit_start,ds0_prepush,planCtx,true);
            if(ph->validity!=PlanValidity::success)
            {
               // approach failed
               continue;
            }

            // store transit path to the object
            transit_paths.push_back(ph->getPathPtr(true));

            auto candidate = findDubins(ds0, ds1, turning_rad, /*reverse=*/false);

            // skip invalid Dubins (infinite-length)
            if (candidate.omplDubins.length() == std::numeric_limits<double>::max())
                continue;

            // 4) Interpolate at the requested resolution
            auto interp = candidate.interpolate(interpResolution);

            // 5) Reject if any interpolated pose leaves the workspace
            bool inside = true;
            for (const auto st : *interp)
            {
                double x = st.x;
                double y = st.y;
                if (x < xmin || x > xmax || y < ymin || y > ymax)
                {
                    inside = false;
                    break;
                }
            }
            if (!inside)
                continue;

            // 6) Score by path length and keep the best
            double L = candidate.lengthCost();
            if (L < best_len)
            {
                best_len    = L;
                bestDubins  = candidate;
                bestInterp  = interp;
                foundAny    = true;
            }
        }
    }

    return foundAny;
}

//-----------------------------------------------------------------------------
// 2) recordCollisions
void Planner::recordCollisions(int o,
                               const ReloPush::StatePathPtr &interp,
                               ob::State *state_curr,
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
                          og::PathGeometric &path_tmp,
                          double margin)
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
    return clearObstacles(idxes_collide, o, interp, state_curr, path_tmp, margin);


}


/*
bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             const ReloPush::StatePathPtr &interp,
                             ob::State* state_curr,
                             og::PathGeometric& path_tmp, double margin)
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
            for (const auto &wp : *interp)
            {
                double dist_to_wp = sqrt(pow(candidate_x - wp.x, 2) + pow(candidate_y - wp.y, 2));
                if (dist_to_wp < margin)
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
*/

bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             const ReloPush::StatePathPtr &interp,
                             ob::State* state_curr,
                             og::PathGeometric& path_tmp,
                             double margin)
{
    // save & restore original environment params
    auto param_org = env_.getParamSingleForAll();

    // parameters for sampling
    const double step_size = 0.05;  // 5 cm increments
    const int    max_steps = 40;    // up to 2 m

    // allocate a scratch state for validity checks
    ob::State* scratch = si_single4clear_->allocState();
    auto* so_scratch = scratch->as<ObjectState>();

    for (int c : idxes_collide)
    {
        // 1) record the collided object’s current pose
        ObjectState* state_c = STATE_OBJECT(state_curr, c);
        const double x0   = state_c->getX();
        const double y0   = state_c->getY();
        const double yaw0 = state_c->getYaw();

        // 2) four candidate push directions: forward, right, backward, left
        std::array<double,4> dirs = {
            yaw0,
            yaw0 + M_PI/2.0,
            yaw0 + M_PI,
            yaw0 + 3.0*M_PI/2.0
        };

        // 3) scan each direction to find the FIRST valid clearance start,
        //    then pick the one with the smallest distance
        double bestDist = std::numeric_limits<double>::infinity();
        double bestDir  = 0.0;

        for (double dir : dirs)
        {
            for (int step = 1; step <= max_steps; ++step)
            {
                double dist = step * step_size;
                double cx   = x0 + dist * std::cos(dir);
                double cy   = y0 + dist * std::sin(dir);

                // set scratch to candidate pose
                so_scratch->setX(cx);
                so_scratch->setY(cy);
                so_scratch->setYaw(dir);

                // bounds & environment validity
                if (!si_single4clear_->getStateSpace()->satisfiesBounds(scratch))
                    continue;
                if (!si_single4clear_->isValid(scratch))
                    continue;

                // avoid colliding with the interpolation path
                bool collide_interp = false;
                for (auto& wp : *interp)
                {
                    double dx = cx - wp.x;
                    double dy = cy - wp.y;
                    if (dx*dx + dy*dy < margin*margin)
                    {
                        collide_interp = true;
                        break;
                    }
                }
                if (collide_interp)
                    continue;

                // first valid for this direction → consider it
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestDir  = dir;
                }
                break;  // stop scanning further along this dir
            }
        }

        // if no direction was valid, bail out
        if (!std::isfinite(bestDist))
        {
            si_single4clear_->freeState(scratch);
            env_.setParamSingleForAll(param_org);
            return false;
        }

        // 4) build a straight‐line clearance path along bestDir
        //    reset object to its original collision pose
        state_c->setX(x0);
        state_c->setY(y0);
        state_c->setYaw(yaw0);

        // tag which object the robot is “pushing”
        STATE_ROBOT(state_curr) = c;

        // 4a) append the collision pose itself
        path_tmp.append(state_curr);

        // 4b) interpolate in step_size increments away from the collision
        int n_steps = static_cast<int>(std::floor(bestDist / step_size));
        for (int i = 1; i <= n_steps; ++i)
        {
            double di = i * step_size;
            double xi = x0 + di * std::cos(bestDir);
            double yi = y0 + di * std::sin(bestDir);

            auto* so = STATE_OBJECT(state_curr, c);
            so->setX(xi);
            so->setY(yi);
            so->setYaw(bestDir);

            path_tmp.append(state_curr);
        }
    }

    // clean up & restore params
    si_single4clear_->freeState(scratch);
    env_.setParamSingleForAll(param_org);
    return true;
}

//------------------------------------------------------------------------------
/**
 * Given the original object definitions and the two loaded OMPL states,
 * populate `objectMap` and `goalMap` keyed by a unique string for each object.
 */
static void populateMaps(
    const std::vector<RobotObjectSetup::Object> &defs,
    const ompl::base::State *state_init,
    const ompl::base::State *state_goal,
    ObjectMap &objectMap,
    GoalMap &goalMap)
{
    // defs.size() == number of objects
    for (size_t idx = 0; idx < defs.size(); ++idx)
    {
        int o = int(idx) + 1;        // 1-based index in STATE_OBJECT
        const auto &def = defs[idx]; // your RobotObjectSetup::Object

        // fetch the OMPL object‐state wrappers
        auto *si = STATE_OBJECT(state_init, o);
        auto *sg = STATE_OBJECT(state_goal, o);

        // build a key (you can pick any scheme you like)
        std::string key = def.name + "_" + std::to_string(o);

        // fill ObjectInfo from the init‐state
        ObjectInfo oi(
            def.name,            // name
            si->getX(),          // x
            si->getY(),          // y
            si->getYaw(),        // nominalOrientation
            /*numberOfSides=*/4, // or pull from def if you store it there
            def.radius           // enclosingRadius
        );
        objectMap.emplace(key, std::move(oi));

        // fill GoalInfo from the goal‐state
        GoalInfo gi(
            def.name,            // name
            sg->getX(),          // x
            sg->getY(),          // y
            sg->getYaw(),        // nominalOrientation
            /*numberOfSides=*/4, // same as above
            def.radius           // enclosingRadius
        );
        goalMap.emplace(key, std::move(gi));
    }
}

bool Planner::planSequence(const std::vector<int> &order,
                           const ob::State *start,
                           const ob::State *goal,
                           ob::State *state_curr,
                           og::PathGeometric &path_tmp,
                           std::vector<int> &done_objs)
{
    std::vector<ReloPush::State> arrival_poses(0);
    std::vector<ReloPush::StatePathPtr> transit_paths(0);
    for (int o : order) {
        env_.setParamSingleForAll(o, done_objs, state_curr);
        // create planning context (update delivered objs) for hybrid astar
        WorkspaceBoundary boundary(4, 5.2); // todo: parse from file
        std::unordered_map<std::string, ObjectInfo> objects_relopush;
        std::unordered_map<std::string, GoalInfo> goals_relopush, delivered_objs;

        std::vector<ReloPush::State> robots = {ReloPush::State(0.1, 0.1, 0.2)}; // todo: parse from file
        PlanningParameters params(1.41,0.8,0.1,0.3,0.15,0.54,0.3,0.2);
        params.setBoundary(boundary);

        populateMaps(defs_, start, goal, objects_relopush, goals_relopush);
        PlanningContext planCtx(params, objects_relopush, delivered_objs); //todo: delivered_objs is currently staying empty

        const double turningRad = planCtx.parameters.turning_rad_pair.push; // for pushing
        const double clearance_margin = planCtx.parameters.obs_rad;


        if (!processObject(o, goal, state_curr,
                           path_tmp, turningRad,
                           clearance_margin, done_objs, planCtx, arrival_poses, transit_paths))
        {
            std::cout << "[";
            for(auto it : order)
            {
                std::cout << it << ",";
            }

            std::cout << "] failed at " << o << ". Trying next permutation" << std::endl;
            return false; // current sequence has no solution. try different permutation
        }

        // Add object in done list
        done_objs.push_back(o);
    }
    return true;
}

bool Planner::processObject(int o,
                            const ob::State *goal,
                            ob::State *state_curr,
                            og::PathGeometric &path_tmp,
                            double turningRad,
                            double clearance_margin,
                            const std::vector<int> &done_objs,
                            PlanningContext &planCtx,
                            std::vector<ReloPush::State>& arrival_poses,
                            std::vector<ReloPush::StatePathPtr> transit_paths)
{
    // Compute best Dubins path
    const ObjectState* s0 = STATE_OBJECT(state_curr, o);
    const ObjectState* s1 = STATE_OBJECT(goal, o);
    reloDubinsPath best;
    ReloPush::StatePathPtr bestInterp(new ReloPush::StatePath);
    ReloPush::State transit_start;
    if(!arrival_poses.size()==0)
        transit_start = ReloPush::find_pre_push(arrival_poses.back(),planCtx.parameters.LF_push*1.01);
    else {
        // todo: parse from context
        transit_start = ReloPush::State(0.1,0.1,0.2);
    }

    if (!findBestDubins(o, s0, s1, turningRad, best, bestInterp, planCtx.parameters.map_resolution, planCtx, transit_start, transit_paths)) // accessable and in-boundary
        return false;

    // Build selfish path
    //auto interp = best.interpolate(0.05f);
    og::PathGeometric selfish(si_single4all_);
    appendInitialState(o, state_curr, selfish);
    appendWaypoints(o, bestInterp, state_curr, selfish);

    // Collision recording
    std::vector<int> idxes_collide;
    std::unordered_map<int, ReloPush::State> collision_pose;
    recordCollisions(o, bestInterp, state_curr,
                     idxes_collide, collision_pose);

    // Clearance if needed
    if (!idxes_collide.empty()) {
        if (!clearObstacles(idxes_collide, o, bestInterp,
                            state_curr, path_tmp,
                            clearance_margin))
            return false;
    }


    // Append segment to overall path
    appendDubinsSegment(o, bestInterp, state_curr, path_tmp);
    // Add arrival pose of this object
    arrival_poses.push_back(best.targetState);


    return true;
}

void Planner::appendInitialState(int o,
                                 const ob::State* state_curr,
                                 og::PathGeometric& path)
{
    //ob::State* st0 = si_single4all_->allocState();
    //si_single4all_->copyState(st0, state_curr);
    //STATE_ROBOT(st0) = o;
    //path.append(st0);

    ob::State* st = si_single4all_->allocState();
    // copy just the o-th ObjectState subspace
    const ompl::base::State* sub = STATE_OBJECT(state_curr, o);
    si_single4all_->copyState(st, sub);
    path.append(st);
}

void Planner::appendWaypoints(int o,
                              ReloPush::StatePathPtr interp,
                              ob::State* state_curr,
                              og::PathGeometric& path)
{
    for (const auto &wp : *interp) {
        ObjectState* so = STATE_OBJECT(state_curr, o);
        so->setX(wp.x);
        so->setY(wp.y);
        so->setYaw(wp.yaw);
        //ob::State* st = si_single4all_->allocState();
        //si_single4all_->copyState(st, state_curr);
        //path.append(st);
        ob::State* st = si_single4all_->allocState();
        const ompl::base::State* sub = STATE_OBJECT(state_curr, o);
        si_single4all_->copyState(st, sub);
        path.append(st);
    }
}



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
