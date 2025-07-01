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


/*
bool Planner::findBestDubins(int o,
                             const ObjectState* s0,
                             const ObjectState* s1,
                             double turning_rad,
                             reloDubinsPath &bestDubins,
                             ReloPush::StatePathPtr& bestInterp,
                             double interpResolution,
                             PlanningContext &planCtx,
                             ReloPush::State& transit_start,
                             std::vector<ReloPush::StatePathPtr>& transit_paths) const
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

    PathPlanResultPtr best_transit;

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

            auto candidate = findDubins(ds0, ds1, turning_rad, false);

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
                best_transit = ph;
            }
        }
    }

    if(foundAny)
    {
        // store transit path to the object
        transit_paths.push_back(best_transit->getPathPtr(true));
    }
    return foundAny;
}
*/

bool Planner::findBestDubins(int o,
                             const ReloPush::State object_start,
                             const ReloPush::State object_goal,
                             double turning_rad,
                             reloDubinsPath &bestDubins,
                             ReloPush::StatePathPtr &bestInterp,
                             double interpResolution,
                             const std::vector<std::pair<int, int>> &excludedIndices,
                             int &chosen_i,
                             int &chosen_j,
                             PlanningContext &planCtx) const
{
    // todo: parse from param
    float prepush_th = Constants::prepush_th;

    // Precompute the 4×4 yaw combinations
    std::vector<double> yaws_start = {
        object_start.yaw,
        object_start.yaw + M_PI_2,
        object_start.yaw + M_PI,
        object_start.yaw + 3.0 * M_PI_2};
    std::vector<double> yaws_goal = {
        object_goal.yaw,
        object_goal.yaw + M_PI_2,
        object_goal.yaw + M_PI,
        object_goal.yaw + 3.0 * M_PI_2};

    // Workspace bounds
    const double xmin = 0, xmax = 4;
    const double ymin = 0, ymax = 5.2;

    double best_len = std::numeric_limits<double>::infinity();
    bool foundAny  = false;
    //PathPlanResultPtr best_transit;

    // Loop over all index pairs (i,j)
    for (int i = 0; i < (int)yaws_start.size(); ++i)
    {
        double y0 = yaws_start[i];
        ReloPush::State ds0(object_start.x, object_start.y, y0);

        // approach check to ds0 (fixed: do this in the later stage)
        //auto ds0_prepush = ReloPush::find_pre_push(
        //    ds0,
       //     (planCtx.parameters.LF_push + planCtx.parameters.obs_rad)*1.01
        //);
       //auto ph0 = planHybridAstar(transit_start, ds0_prepush, planCtx, true);
        //if (ph0->validity != PlanValidity::success)
        //    continue;

        for (int j = 0; j < (int)yaws_goal.size(); ++j)
        {
            // 1) Skip if this (i,j) was excluded
            if (std::find(excludedIndices.begin(),
                          excludedIndices.end(),
                          std::make_pair(i,j))
                != excludedIndices.end())
            {
                continue;
            }

            double y1 = yaws_goal[j];
            ReloPush::State ds1(object_goal.x, object_goal.y, y1);

            // robot-centric
            ReloPush::State robot_start = ReloPush::find_pre_push(ds0, prepush_th);
            ReloPush::State robot_goal = ReloPush::find_pre_push(ds1, prepush_th);

            // 2) Generate the raw Dubins candidate
            //auto candidate = findDubins(ds0, ds1, turning_rad, /*reverse=*/false);
            auto candidate = findDubins(robot_start, robot_goal, turning_rad, /*reverse=*/false);
            if (candidate.omplDubins.length() == std::numeric_limits<double>::max())
                continue;

            // 3) Interpolate
            auto interp = candidate.interpolate(interpResolution);

            // 4) Reject if out of bounds
            bool inside = true;
            for (auto &st : *interp) {
                if (st.x < xmin || st.x > xmax || st.y < ymin || st.y > ymax) {
                    inside = false;
                    break;
                }
            }
            if (!inside)
                continue;

            // 5) Score by length
            double L = candidate.lengthCost();
            if (L < best_len) {
                best_len     = L;
                bestDubins   = candidate;
                bestInterp   = interp;
                chosen_i     = i;
                chosen_j     = j;
                foundAny     = true;
                //best_transit = ph0;
            }
        }
    }

    //if (foundAny) {
        // store the transit path to ds0
    //    transit_paths.push_back(best_transit->getPathPtr(true));
    //}
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

//bool Planner::doClearance(int o,
//                          const std::vector<int> &idxes_collide,
//                          const std::unordered_map<int,ReloPush::State> &collision_pose,
//                          ob::State *state_curr,
//                          const ReloPush::StatePathPtr &interp,  // <<--- interp in
//                          og::PathGeometric &path_tmp,
//                          double margin)
//{

//    //auto deb = STATE_OBJECT(state_curr,3)->getYaw();
//    // Inject the recorded collision poses into state_curr
//    /*
//    for (int c : idxes_collide) {
//        auto it = collision_pose.find(c);
//        if (it != collision_pose.end()) {
//            ObjectState* so = STATE_OBJECT(state_curr, c);
//            so->setX(it->second.x);
//            so->setY(it->second.y);
//            so->setYaw(it->second.yaw);
//        }
//    }

//    */
//    // Call clearObstacles with selfish_path
//    return clearObstacles(idxes_collide, o, interp, state_curr, path_tmp, margin);


//}


/*
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

*/


bool Planner::dfsClearance(const std::vector<std::vector<ClearanceCand>>& allCands,
                           PlanningContext planCtx,  // Pass-by-value (copy for each call)
                           ReloPush::State& transit_start,
                           int obsIdx,
                           ReloPush::StatePathPtrList& transit_paths,
                           std::vector<int>& selected_indices,
                           const std::vector<int>& idxes_collide,
                           const ReloPush::State transit_end)
{
    float prepush_th = Constants::prepush_th;
    if (obsIdx == allCands.size())
    {
        // last transit to the object to rearrange
        auto ph = planHybridAstar(transit_start, transit_end, planCtx, true);
        if (ph->validity != PlanValidity::success)
            return false;

        transit_paths.push_back(ph->getPathPtr(true));
        return true;
    }

    int c = idxes_collide[obsIdx];
    for (size_t cand_idx = 0; cand_idx < allCands[obsIdx].size(); ++cand_idx)
    {
        ClearanceCand cand = allCands[obsIdx][cand_idx];

        // Get the entry pose (start of path), and exit pose (end of path)
        ReloPush::State entry_pose = ReloPush::find_pre_push(cand.obs_start,prepush_th);
        //const ReloPush::State& exit_pose  = cand.path->back();

        // Plan transit from current position to this candidate's entry pose
        auto ph = planHybridAstar(transit_start, entry_pose, planCtx, true);
        if (ph->validity != PlanValidity::success) continue;

        transit_paths.push_back(ph->getPathPtr(true));
        selected_indices[obsIdx] = cand_idx;

        // *** Make a new copy for the next DFS call ***
        PlanningContext planCtxNext = planCtx; // Deep copy

        // Update the moved obstacle pose in planCtxNext!
        planCtxNext.removeObs(cand.obs_start);
        planCtxNext.addObs(cand.path->back());


        // DFS to the next obstacle
        if (dfsClearance(allCands, planCtxNext, entry_pose,
                         obsIdx+1, transit_paths, selected_indices, idxes_collide, transit_end))
            return true;

        // Backtrack
        transit_paths.pop_back();
        selected_indices[obsIdx] = -1;
    }
    return false;
}


bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
                             int o,
                             const ReloPush::StatePathPtr &interp,
                             ob::State* state_curr,
                             og::PathGeometric& path_tmp,
                             PlanningContext& planCtx,
                             ReloPush::StatePathPtrList& transit_paths,
                             ReloPush::State& transit_start,
                             const ReloPush::State& transit_end,
                             double margin)
{
    auto param_org = env_.getParamSingleForAll();

    const double step_size = 0.05;
    const int max_steps = 40;
    float prepush_th = Constants::prepush_th;

    ob::State* scratch = si_single4clear_->allocState();
    auto* so_scratch = scratch->as<ObjectState>();

    std::vector<std::vector<ClearanceCand>> allCands;
    allCands.reserve(idxes_collide.size());

    for (int c : idxes_collide)
    {
        ObjectState* state_c = STATE_OBJECT(state_curr, c);
        const double x0 = state_c->getX(), y0 = state_c->getY(), yaw0 = state_c->getYaw();

        std::array<double,4> dirs = {yaw0, yaw0 + M_PI_2, yaw0 + M_PI, yaw0 + 3*M_PI_2};
        std::vector<ClearanceCand> cand;

        for (double dir : dirs)
        {
            auto obs_pose = ReloPush::State(x0, y0, dir);
            auto obs_prepush = ReloPush::find_pre_push(obs_pose, prepush_th);
            if (!planCtx.env_nonpush.stateValid(obs_prepush)) continue;

            auto linear_path = std::make_shared<ReloPush::StatePath>();
            bool found_valid = false;

            for (int step = 1; step <= max_steps; ++step)
            {
                double d = step * step_size;
                double cx = x0 + d * cos(dir), cy = y0 + d * sin(dir);

                so_scratch->setX(cx);
                so_scratch->setY(cy);
                so_scratch->setYaw(dir);

                if (!si_single4clear_->getStateSpace()->satisfiesBounds(scratch)) continue;
                if (!si_single4clear_->isValid(scratch)) continue;

                bool bad = false;
                for (auto &wp : *interp){
                    double dx = cx - wp.x, dy = cy - wp.y;
                    if (dx*dx + dy*dy < margin*margin) { bad = true; break; }
                }
                if (bad) continue;

                // Create linear straight-line path
                for (int i = 1; i <= step; ++i) {
                    double di = i * step_size;
                    linear_path->emplace_back(x0 + di*cos(dir), y0 + di*sin(dir), dir);
                }

                cand.emplace_back(ClearanceCand(d, dir, linear_path,obs_pose));
                found_valid = true;
                break;  // only first valid candidate
            }
        }

        std::sort(cand.begin(), cand.end(), [](auto &a, auto &b){ return a.dist < b.dist; });
        allCands.push_back(std::move(cand));
    }

    //transit_paths.clear();
    std::vector<int> selected_indices(idxes_collide.size(), -1);
    bool success = dfsClearance(allCands, planCtx, transit_start,
                                0, transit_paths, selected_indices,idxes_collide,transit_end);

    if (success)
    {
        ob::State* state_clear = si_all4all_->allocState();
        si_all4all_->copyState(state_clear, state_curr);

        for (size_t obsIdx = 0; obsIdx < idxes_collide.size(); ++obsIdx)
        {
            int c = idxes_collide[obsIdx];
            ClearanceCand& selected_cand = allCands[obsIdx][selected_indices[obsIdx]];
            ReloPush::StatePath& straight_path = *(selected_cand.path);

            ObjectState* state_c = STATE_OBJECT(state_clear, c);

            STATE_ROBOT(state_clear) = c;

            for (auto& wp : straight_path)
            {
                state_c->setX(wp.x);
                state_c->setY(wp.y);
                state_c->setYaw(wp.yaw);
                path_tmp.append(state_clear);
            }

            // update planning context for hybrid astar
            planCtx.removeObs(selected_cand.obs_start);
            planCtx.addObs(straight_path.back());
        }

        // *** Copy back into your “live” planning state ***
        si_all4all_->copyState(state_curr, state_clear);

        si_all4all_->freeState(state_clear);
    }

    si_single4clear_->freeState(scratch);
    env_.setParamSingleForAll(param_org);

    return success;
}


//bool Planner::clearObstacles(const std::vector<int>& idxes_collide,
//                             int o,
//                             const ReloPush::StatePathPtr &interp,
//                             ob::State* state_curr,
//                             og::PathGeometric& path_tmp,
//                             double margin)
//{
//    // save & restore original environment params
//    auto param_org = env_.getParamSingleForAll();

//    // parameters for sampling
//    const double step_size = 0.05;  // 5 cm increments
//    const int    max_steps = 40;    // up to 2 m

//    // allocate a scratch state for validity checks
//    ob::State* scratch = si_single4clear_->allocState();
//    auto* so_scratch = scratch->as<ObjectState>();

//    for (int c : idxes_collide)
//    {
//        // 1) record the collided object’s current pose
//        ObjectState* state_c = STATE_OBJECT(state_curr, c);
//        const double x0   = state_c->getX();
//        const double y0   = state_c->getY();
//        const double yaw0 = state_c->getYaw();

//        // 2) four candidate push directions: forward, right, backward, left
//        std::array<double,4> dirs = {
//            yaw0,
//            yaw0 + M_PI/2.0,
//            yaw0 + M_PI,
//            yaw0 + 3.0*M_PI/2.0
//        };

//        // 3) scan each direction to find the FIRST valid clearance start,
//        //    then pick the one with the smallest distance
//        double bestDist = std::numeric_limits<double>::infinity();
//        double bestDir  = 0.0;

//        for (double dir : dirs)
//        {
//            for (int step = 1; step <= max_steps; ++step)
//            {
//                double dist = step * step_size;
//                double cx   = x0 + dist * std::cos(dir);
//                double cy   = y0 + dist * std::sin(dir);

//                // set scratch to candidate pose
//                so_scratch->setX(cx);
//                so_scratch->setY(cy);
//                so_scratch->setYaw(dir);

//                // bounds & environment validity
//                if (!si_single4clear_->getStateSpace()->satisfiesBounds(scratch))
//                    continue;
//                if (!si_single4clear_->isValid(scratch))
//                    continue;

//                // avoid colliding with the interpolation path
//                bool collide_interp = false;
//                for (auto& wp : *interp)
//                {
//                    double dx = cx - wp.x;
//                    double dy = cy - wp.y;
//                    if (dx*dx + dy*dy < margin*margin)
//                    {
//                        collide_interp = true;
//                        break;
//                    }
//                }
//                if (collide_interp)
//                    continue;

//                // first valid for this direction → consider it
//                if (dist < bestDist)
//                {
//                    bestDist = dist;
//                    bestDir  = dir;
//                }
//                break;  // stop scanning further along this dir
//            }
//        }

//        // if no direction was valid, bail out
//        if (!std::isfinite(bestDist))
//        {
//            si_single4clear_->freeState(scratch);
//            env_.setParamSingleForAll(param_org);
//            return false;
//        }

//        // 4) build a straight‐line clearance path along bestDir
//        //    reset object to its original collision pose
//        state_c->setX(x0);
//        state_c->setY(y0);
//        state_c->setYaw(yaw0);

//        // tag which object the robot is “pushing”
//        STATE_ROBOT(state_curr) = c;

//        // 4a) append the collision pose itself
//        path_tmp.append(state_curr);

//        // 4b) interpolate in step_size increments away from the collision
//        int n_steps = static_cast<int>(std::floor(bestDist / step_size));
//        for (int i = 1; i <= n_steps; ++i)
//        {
//            double di = i * step_size;
//            double xi = x0 + di * std::cos(bestDir);
//            double yi = y0 + di * std::sin(bestDir);

//            auto* so = STATE_OBJECT(state_curr, c);
//            so->setX(xi);
//            so->setY(yi);
//            so->setYaw(bestDir);

//            path_tmp.append(state_curr);
//        }
//    }

//    // clean up & restore params
//    si_single4clear_->freeState(scratch);
//    env_.setParamSingleForAll(param_org);
//    return true;
//}

//------------------------------------------------------------------------------
/**
 * Given the original object definitions and the two loaded OMPL states,
 * populate `objectMap` and `goalMap` keyed by a unique string for each object.
 */
static void populateMaps(
    const std::vector<RobotObjectSetup::Object> &defs,
    const ompl::base::State *state_init,
    const ompl::base::State *state_goal,
    const std::vector<int> &done_objs,
    ObjectMap &objectMap,
    GoalMap &goalMap,
    GoalMap &delivered_objs)
{
    // for fast lookup
    std::unordered_set<int> done_set(done_objs.begin(), done_objs.end());

    for (size_t idx = 0; idx < defs.size(); ++idx)
    {
        int o = int(idx) + 1; // 1-based index for STATE_OBJECT
        const auto &def = defs[idx];

        // OMPL wrappers
        auto *si = STATE_OBJECT(state_init, o);
        auto *sg = STATE_OBJECT(state_goal, o);

        // a unique key per object
        std::string key = std::to_string(o);

        // build the goal‐info
        GoalInfo gi(
            def.name,
            sg->getX(),
            sg->getY(),
            sg->getYaw(),
            /*nSide=*/4,
            def.radius);

        if (done_set.count(o))
        {
            // already delivered → go into delivered_objs
            delivered_objs.emplace(key, std::move(gi));
        }
        else
        {
            // still pending → populate both objectMap and goalMap

            // 1) pending initial‐info
            ObjectInfo oi(
                def.name,
                si->getX(),
                si->getY(),
                si->getYaw(),
                /*nSide=*/4,
                def.radius);
            objectMap.emplace(key, std::move(oi));

            // 2) pending goal‐info
            goalMap.emplace(key, std::move(gi));
        }
    }
}

bool Planner::planSequence(const std::vector<int> &order,
                           const ob::State *start,
                           const ob::State *goal,
                           ob::State *state_curr,
                           og::PathGeometric &path_tmp,
                           std::vector<int> &done_objs,
                           std::vector<ReloPush::StatePathPtr>& transit_paths)
{
    std::vector<ReloPush::State> arrival_poses(0);


    // create planning context (update delivered objs) for hybrid astar
    WorkspaceBoundary boundary(4, 5.2); // todo: parse from file
    std::unordered_map<std::string, ObjectInfo> objects_relopush;
    std::unordered_map<std::string, GoalInfo> goals_relopush, delivered_objs;

    std::vector<ReloPush::State> robots = {ReloPush::State(0.1, 0.1, 0.2)}; // todo: parse from file
    PlanningParameters params(Constants::r_push, Constants::r_nonpush,
                                Constants::mapResolution,
                            Constants::carWidth,
                            Constants::obsRadius,
                            Constants::LF_push, Constants::LF_nonpush,
                            Constants::LB);
    params.setBoundary(boundary);

    populateMaps(defs_, start, goal, done_objs, objects_relopush, goals_relopush, delivered_objs);
    PlanningContext planCtx(params, objects_relopush, delivered_objs);

    const double turningRad = planCtx.parameters.turning_rad_pair.push; // for pushing
    const double clearance_margin = planCtx.parameters.obs_rad*2;
    
    for (int o : order) {
        env_.setParamSingleForAll(o, done_objs, state_curr);
        if (!processObject(o, goal, state_curr,
                           path_tmp, turningRad,
                           clearance_margin, done_objs,
                           planCtx, arrival_poses,
                           transit_paths, robots))
        {
            std::cout << "\n[";
            for(auto it : order)
            {
                std::cout << it << ",";
            }

            std::cout << "] failed at " << o << ". Trying next permutation" << std::endl;
            done_objs.clear();
            return false; // current sequence has no solution. try different permutation
        }

        // Add object in done list
        done_objs.push_back(o);
        std::cout << " << " << o;
        std::cout.flush();
        
    }
    return true;
}

/*
bool Planner::processObject(int o,
                            const ob::State *goal,
                            ob::State *state_curr,
                            og::PathGeometric &path_tmp,
                            double turningRad,
                            double clearance_margin,
                            const std::vector<int> &done_objs,
                            PlanningContext &planCtx,
                            std::vector<ReloPush::State>& arrival_poses,
                            std::vector<ReloPush::StatePathPtr>& transit_paths)
{
    // Compute best Dubins path
    const ObjectState* s0 = STATE_OBJECT(state_curr, o);
    const ObjectState* s1 = STATE_OBJECT(goal, o);
    reloDubinsPath best;
    ReloPush::StatePathPtr bestInterp(new ReloPush::StatePath);
    ReloPush::State transit_start;
    if(!arrival_poses.size()==0)
        transit_start = ReloPush::find_pre_push(arrival_poses.back(), (planCtx.parameters.LF_push + planCtx.parameters.obs_rad) * 1.01);
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
                            {
                                // take the last transit back out
                                transit_paths.pop_back();
                                return false;
                            }
    }


    // Append segment to overall path
    appendDubinsSegment(o, bestInterp, state_curr, path_tmp);
    // Add arrival pose of this object
    arrival_poses.push_back(best.targetState);


    return true;
}
*/

bool Planner::processObject(int o,
                            const ob::State                    *goal,
                            ob::State                          *state_curr,
                            og::PathGeometric                  &path_tmp,
                            double                              turningRad,
                            double                              clearance_margin,
                            std::vector<int>             &done_objs,
                            PlanningContext                    &planCtx,
                            std::vector<ReloPush::State>       &arrival_poses,
                            std::vector<ReloPush::StatePathPtr> &transit_paths,
                            std::vector<ReloPush::State>& robots)
{
    const ObjectState* s0 = STATE_OBJECT(state_curr, o);
    const ObjectState* s1 = STATE_OBJECT(goal,       o);

    ReloPush::State obj_start_relopush = ReloPush::State(s0->getX(),s0->getY(),s0->getYaw());
    ReloPush::State obj_goal_relopush = ReloPush::State(s1->getX(),s1->getY(),s1->getYaw());

    float prepush_th = Constants::prepush_th;

    // 1) Determine starting point for transit
    ReloPush::State transit_start;
    if (!arrival_poses.empty()) {
        transit_start = ReloPush::find_pre_push(
            arrival_poses.back(),
            prepush_th
        );
    } else {
        transit_start = robots[0];
    }

    // 2) Prepare exclusion list of (i,j) index pairs
    std::vector<std::pair<int,int>> excluded;
    int chosen_i = -1, chosen_j = -1;

    // 3) Keep trying until one succeeds or none left
    while (true)
    {
        reloDubinsPath bestDubins;
        ReloPush::StatePathPtr bestInterp(new ReloPush::StatePath);

        bool gotOne = findBestDubins(
            o, obj_start_relopush, obj_goal_relopush, turningRad,
            bestDubins, bestInterp,
            planCtx.parameters.map_resolution,
            excluded,    // skip these index pairs
            chosen_i,    // OUT: start‐index in yaws_start
            chosen_j,     // OUT: goal‐index  in yaws_goal
            planCtx
        );

        if (!gotOne)
            return false;  // exhausted all candidates

        // 4) Create a “selfish” path for collision checking
        og::PathGeometric selfish(si_single4all_);
        appendInitialState(o, state_curr, selfish);
        appendWaypoints   (o, bestInterp,   state_curr, selfish);

        // 5) Record collisions
        std::vector<int> idxes_collide;
        std::unordered_map<int, ReloPush::State> collision_pose;
        recordCollisions(o, bestInterp, state_curr,
                         idxes_collide, collision_pose);

        // fail if any finished object collides with this
        bool has_common = std::any_of(idxes_collide.begin(), idxes_collide.end(), [&](int val) {
              return std::find(done_objs.begin(), done_objs.end(), val) != done_objs.end();
          });

          if (has_common) {
              std::cout << "Finished object is on the way." << std::endl;
              for (int val : done_objs) {
                  std::cout << val << " ";
              }
              std::cout << std::endl;

              std::cout << "col: ";
              for (int n : idxes_collide) {
                  std::cout << n << " ";
              }
              return false;
          }


        ReloPush::State obj_app = ReloPush::find_pre_push(bestDubins.startState,prepush_th);
        //ReloPush::State obj_app = ReloPush::find_pre_push(bestDubins.startState, 0.1); // already pre-pushed

        // 6) If collisions → attempt clearance
        if (!idxes_collide.empty())
        {
            bool cleared = clearObstacles(
                idxes_collide, o, bestInterp,
                state_curr, path_tmp,
                 planCtx, transit_paths, transit_start, obj_app, clearance_margin
            );
            if (!cleared) {
                std::cout << "\tClearing failed. Trying other start/goal poses" << std::endl;
                // rollback the transit path we just pushed
                //transit_paths.pop_back();
                // mark this (i,j) as excluded
                excluded.emplace_back(std::make_pair(chosen_i, chosen_j));
                // and try the next Dubins
                continue;
            }
        }
        else
        {
            // transit from robot todo: plan dubins in robot perspective
            auto ph0 = planHybridAstar(transit_start, obj_app, planCtx, true);
            if (ph0->validity != PlanValidity::success)
            {
                std::cout << "\tApproaching failed. Trying other start/goal poses" << std::endl;
                excluded.emplace_back(std::make_pair(chosen_i, chosen_j));
                continue;
            }
            transit_paths.push_back(ph0->getPathPtr(true));
        }

        // 7) Success: append to final path & record arrival
        appendDubinsSegment(o, bestInterp, state_curr, path_tmp, prepush_th);
        arrival_poses.push_back(bestDubins.targetState);
        return true;
    }
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
                                  og::PathGeometric &path_tmp,
                                  double pre_push_dist)
{
    // For every waypoint on the selfish Dubins path...
//    for (const auto &wp : *interp)
//    {
//        // 1) set the robot‐index to object o
//        STATE_ROBOT(state_curr) = o;

//        // 2) overwrite exactly that object’s pose
//        ObjectState* so = STATE_OBJECT(state_curr, o);
//        so->setX  (wp.x);
//        so->setY  (wp.y);
//        so->setYaw(wp.yaw);

//        // 3) append ONLY this new state
//        path_tmp.append(state_curr);
//    }

    for (const auto &wp : *interp)
    {
        // 0) which object we’re pushing
        STATE_ROBOT(state_curr) = o;

        // 1) compute the object’s new pose by “pushing” the robot waypoint forward
        //    (wp is the robot pose; pushDistance is how far the object moves)
        ReloPush::State objPose = ReloPush::find_post_push(
            const_cast<ReloPush::State&>(wp),
            static_cast<float>(pre_push_dist)
        );

        // 2) overwrite exactly that object’s pose in the OMPL state
        ObjectState* so = STATE_OBJECT(state_curr, o);
        so->setX  (objPose.x);
        so->setY  (objPose.y);
        so->setYaw(objPose.yaw);

        // 3) record this updated state
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
