#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <string>

struct WorkspaceBoundary
{
    double xMin = 0.0;
    double xMax = 4.0;
    double yMin = 0.0;
    double yMax = 5.0;

    WorkspaceBoundary()
    {}

    WorkspaceBoundary(double xMax_in, double yMax_in) : xMax(xMax_in), yMax(yMax_in)
    {}

};

struct TurningRadiusPair
{
    float push;
    float non_push;
};

struct SpeedPair
{
    float push = 0.36;
    float non_push = 0.5;
};

struct PlanningParameters
{
    // The rectangular workspace
    WorkspaceBoundary boundary;
    TurningRadiusPair turning_rad_pair;
    SpeedPair speed_pair;
    float map_resolution;
    float car_width;
    float obs_rad;
    float LF_push;
    float LF_nonpush;
    float LB;
    double PrePush_dist = 0.54; //0.54

    // Possibly other fields:
    // double normalModeThreshold;
    // double preRelocationThreshold;
    // etc.

    PlanningParameters()
    {}    

    // r_push, r_nonpush, map_res, car_w, obs_r, LF_push, LF_nonpush, LB
    PlanningParameters(float r_push, float r_nonpush, float map_resol, float car_w, float obs_r, float LF_p, float LF_np, float LB_)
    : map_resolution(map_resol), car_width(car_w), obs_rad(obs_r), LF_push(LF_p), LF_nonpush(LF_np), LB(LB_)
    {
        turning_rad_pair.push = r_push;
        turning_rad_pair.non_push = r_nonpush;
    }

    void setBoundary(WorkspaceBoundary& boundary_in)
    {
        boundary=boundary_in;
    }
};

namespace params
{
    extern const std::string world_frame;

    // todo: parse map size as params
    extern float map_max_x; // m
    extern float map_max_y; // m

    // distance in finding pre-push poses
    extern float pre_push_dist; // m
    extern float pre_relo_pre_push_offset;

    // index of post-push pose from last state of the path
    extern int post_push_ind;
    extern float interpolation_step;

    // may change multiple times while running
    extern bool is_pushing;

    extern bool use_mocap; //todo: parse as a parameter

    extern const bool reset_robot_pose;

    extern const bool print_graph;

    // baseline: MP only
    extern bool use_mp_only;

    extern int leave_log;
    extern bool print_log;
    extern bool use_better_path;
    extern bool print_final_path;
    extern bool use_testdata;
    extern bool measure_exec_time;

    extern int64_t grid_search_timeout;
}

#endif // PARAMETERS_HPP
