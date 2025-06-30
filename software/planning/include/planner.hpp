#ifndef CHANGKYU_PLANNER__HPP
#define CHANGKYU_PLANNER__HPP
#include "statespace.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include "bullet_simulation/bullet_simulation.hpp"

#include "mdp_planner.hpp"

#include <fromReloPush/DubinsTools.h>
#include <fromReloPush/pre_post_push_pose.hpp>
#include <fromReloPush/PlanHybridAstar.hpp>
#include <cmath>  //  std::tan, std::sqrt
#include <limits> //  std::numeric_limits

#define DEBUG 1

class Planner 
{
public:
    Planner(RobotObjectSetup &env);
    Planner(RobotObjectSetup &env, std::vector<RobotObjectSetup::Object> defs); // jeeho
    ~Planner();

    enum TYPE_ACTION
    {
        ACTION_TRANSITION=0,
        ACTION_TRANSFER,
        ACTION_PUSHING,
    };

    struct Action
    {
        TYPE_ACTION type;
        int idx_target;
        int idx_target2;
        double x;
        double y;
        double yaw;
    };

    void UseKino()
    {
        use_kino = true;
    }

    typedef void (*fnDoAction)( const ompl::base::State* state, int o, 
                                const ompl::geometric::PathGeometric &path_obj,
                                std::vector<ompl::base::State*> &states_res );

    void load_precomputed_planners( std::ifstream &ifs )
    {
        if( soap_rotator_ ) delete soap_rotator_;
        soap_rotator_ = new SoapRotators(ifs);
    }

    void plan( const ompl::base::State *start,
               const ompl::base::State *goal,
               ompl::geometric::PathGeometric &path_res,
               std::vector<Action> &actions_res,
               bool do_merge=true );

    void plan_plRS( const ompl::base::State *state_start,
                    const ompl::base::State *state_goal,
                    ompl::geometric::PathGeometric &path_res,
                    std::vector<Action> &actions_res );

    bool plan_plrs_jeeho(const ob::State* start,
                                  const ob::State* goal,
                                  og::PathGeometric &path_res,
                                  std::vector<Action> &actions_res);


/*
    void plan_plrs_jeeho(const ompl::base::State *state_start,
                         const ompl::base::State *state_goal,
                         ompl::geometric::PathGeometric &path_res,
                         std::vector<Action> &actions_res);
                         */

    /*
        void plan_kino( const ompl::base::State *state_start,
                        const ompl::base::State *state_goal,
                        ompl::geometric::PathGeometric &path_res,
                        std::vector<Action> &actions_res );
    */
    bool simulate(const ompl::base::State *state, int idx,
                  const std::vector<btVector3> &pos_delta,
                  const std::vector<double> &yaw_delta,
                  std::vector<ompl::base::State *> &state_res);

    bool simulate( const ompl::base::State* state, int o, 
                   const ompl::geometric::PathGeometric &path_obj,
                   std::vector<ompl::base::State*> &states_res     );

#if DEBUG
    void simulate_gui( const ompl::geometric::PathGeometric &path,
                       const std::string &fp_record="");

    void simulate_gui( const ompl::geometric::PathGeometric &path,
                       const std::vector<Action> &actions );

    void simulate_gui( int o_obj, 
                       const ompl::geometric::PathGeometric &path_obj,
                       ompl::base::State* state_res );
#endif 

    bool isValidPath( ompl::base::State* state, int o_target,
                      const ompl::geometric::PathGeometric &path,
                      std::vector<ompl::base::State*> &states_res,
                      const bool do_action=false );
    
    static double compute_cost( const ompl::geometric::PathGeometric &path );

    void path2Actions( const ompl::geometric::PathGeometric &path, 
                       std::vector<Action> &actions);

    void path2ActionsWithTransitPaths(
        const ompl::geometric::PathGeometric &path,
        const std::vector<ReloPush::StatePathPtr> &transitPaths,
        std::vector<Action> &actions);

    double distance( const ompl::base::State* s1, const ompl::base::State* s2 )
    {
        return si_all4all_->distance(s1,s2);
    }

    static void save_plan( const std::string &fp_save,
                           const std::string &name,
                           int n_objs,
                           double time_spent,
                           double time_moving,
                           const ompl::base::State* state_init,
                           const ompl::base::State* state_goal,
                           const ompl::geometric::PathGeometric &path,
                           const std::vector<Action> &actions          );

    void visualizePath(cv::Mat& img, const ompl::geometric::PathGeometric &path, bool is_relopush = false);
    void visualizePath(cv::Mat& img, const ompl::geometric::PathGeometric &path, double minx, double maxx, double miny, double maxy, bool is_relopush = true);

    void visualizeState(cv::Mat& img, const ompl::base::State* state);

    void test();    

private:

    bool plan_pushing( const ompl::base::State* state, int o_pusher, int o_target,
                       const ompl::geometric::PathGeometric &path_mrg,
                       const ompl::geometric::PathGeometric &path_psh,
                       const ompl::geometric::PathGeometric &path_sep,
                       const ompl::base::State* state_target_goal,
                       const ompl::base::State* state_pusher_goal,
                       ompl::geometric::PathGeometric &path_pushing,
                       std::vector<Action> &actions_res );

    bool plan_pushing( const ompl::base::State* state, int o_pusher, int o_target,
                       const ompl::geometric::PathGeometric &path_target,
                       ompl::geometric::PathGeometric &path_pushing,
                       const bool do_action=false );

    bool plan_pushing( const ompl::base::State* state, int o_pusher, int o_target,
                       double x_goal, double y_goal,
                       ompl::geometric::PathGeometric &path_docking,
                       bool* stop, const bool do_action=false );

    bool plan_pushing_kino( const ompl::base::State* state, 
                            int o_pusher, int o_target,
                            double x_goal, double y_goal,
                            ompl::geometric::PathGeometric &path_res,
                            bool* stop );

    bool plan_docking( const ompl::base::State* state, int o_pusher, int o_target,
                       double x_goal, double y_goal,
                       ompl::geometric::PathGeometric &path_docking,
                       SoapRotators::DOCKING_SIDE* docking_side=NULL,
                       const bool do_action=false );

    bool mergePath( const ompl::geometric::PathGeometric &path, 
                          ompl::geometric::PathGeometric &path_res,
                          std::vector<Action> &actions_res );

    void splitPath( const ompl::geometric::PathGeometric &path, 
                    std::vector<ompl::geometric::PathGeometric*> &paths_res,
                    std::vector<int> &order_res,
                    std::vector<std::pair<int,int> > &idxes_startend,
                    std::vector<double> &accum_costs );

    bool findMergingSeparatingPoints( const ompl::geometric::PathGeometric &path, 
                                      const ompl::base::State* state_pusher_init, 
                                      const ompl::base::State* state_pusher_goal, 
                                      ompl::base::State* state_target_mrg, 
                                      ompl::base::State* state_target_sep,
                                      ompl::geometric::PathGeometric &path_mrg, 
                                      ompl::geometric::PathGeometric &path_psh, 
                                      ompl::geometric::PathGeometric &path_sep, 
                                      double* cost );



    // Pick the shortest Dubins path over 4×4 start/goal yaws.

    /*bool findBestDubins(int o,
                        const ObjectState* s0,
                        const ObjectState* s1,
                        double turning_rad,
                        reloDubinsPath &bestDubins,
                        ReloPush::StatePathPtr& bestInterp,
                        double interpResolution,
                        PlanningContext &planCtx,
                        ReloPush::State& transit_start,
                        std::vector<ReloPush::StatePathPtr>& transit_paths) const;
                        */

    bool findBestDubins(int o,
                        const ReloPush::State object_start,
                        const ReloPush::State object_goal,
                        double turning_rad,
                        reloDubinsPath &bestDubins,
                        ReloPush::StatePathPtr &bestInterp,
                        double interpResolution,
                        const std::vector<std::pair<int, int>> &excludedIndices,
                        int &chosen_i,
                        int &chosen_j,
                        PlanningContext &planCtx) const;
                        
    // Walk that Dubins path and record which objects first collide.
    void recordCollisions(int o,
                          const ReloPush::StatePathPtr &interp,
                          ob::State *state_curr,
                          std::vector<int> &idxes_collide,
                          std::unordered_map<int,ReloPush::State> &collision_pose);

    // Use interp directly for clearance collision checks.
    /*
    bool doClearance(int o,
                     const std::vector<int> &idxes_collide,
                     const std::unordered_map<int,ReloPush::State> &collision_pose,
                     ob::State *state_curr,
                     const ReloPush::StatePathPtr &interp,  // <<--- pass interp
                     og::PathGeometric &path_tmp,
                     double margin = 0.3);
*/
    // Inject the recorded collision poses and clear them.
    /*
    bool doClearance(int o,
                     const std::vector<int> &idxes_collide,
                     const std::unordered_map<int,ReloPush::State> &collision_pose,
                     ob::State *state_curr,
                     const og::PathGeometric &selfish_path,
                     og::PathGeometric &path_tmp);
    */

    struct ClearanceCand {
        double dist;
        double dir;
        ReloPush::StatePathPtr path;  // Using shared pointer to avoid unnecessary copying
        ReloPush::State obs_start;
        //ReloPush::State obs_goal;

        ClearanceCand(double d, double di, ReloPush::StatePathPtr p, ReloPush::State s)
            : dist(d), dir(di), path(std::move(p)), obs_start(s) {}
    };
    using ClearanceCands = std::vector<ClearanceCand>;

    bool dfsClearance(const std::vector<std::vector<ClearanceCand>>& allCands,
                               PlanningContext planCtx,  // Pass-by-value (copy for each call)
                               ReloPush::State& transit_start,
                               int obsIdx,
                               ReloPush::StatePathPtrList& transit_paths,
                               std::vector<int>& selected_indices,
                               const std::vector<int>& idxes_collide,
                               const ReloPush::State transit_end);



    bool clearObstacles(const std::vector<int>& idxes_collide,
                         int o,
                         const ReloPush::StatePathPtr& interp,
                         ob::State* state_curr,
                         og::PathGeometric& path_tmp,
                        PlanningContext& planCtx,
                        ReloPush::StatePathPtrList& transit_paths,
                        ReloPush::State& transit_start,
                        const ReloPush::State& transit_end,
                         double margin = 0.3);

//    bool clearObstacles(const std::vector<int>& idxes_collide,
//                                 int o,
//                                 const ReloPush::StatePathPtr &interp,
//                                 const ReloPush::State &transit_start,
//                                 ob::State* state_curr,
//                                 og::PathGeometric& path_tmp,
//                                 std::vector<ReloPush::StatePathPtr> &newTransits,
//                                 double margin, PlanningContext& planCtx);

    bool planSequence(const std::vector<int> &order,
                      const ob::State *start,
                      const ob::State *goal,
                      ob::State *state_curr,
                      og::PathGeometric &path_tmp,
                      std::vector<int> &done_objs,
                      std::vector<ReloPush::StatePathPtr> &transit_paths);

    bool processObject(int o,
                       const ob::State *goal,
                       ob::State *state_curr,
                       og::PathGeometric &path_tmp,
                       double turningRad,
                       double clearance_margin,
                       const std::vector<int> &done_objs,
                       PlanningContext &planCtx,
                       std::vector<ReloPush::State>& arrival_poses,
                       std::vector<ReloPush::StatePathPtr>& transit_paths,
                       std::vector<ReloPush::State>& robots);

    void appendInitialState(int o,
                             const ob::State* state_curr,
                             og::PathGeometric& path);


    void appendWaypoints(int o,
                          ReloPush::StatePathPtr interp,
                          ob::State* state_curr,
                          og::PathGeometric& path);


    // implementation of clearance
    /*
    bool clearObstacles(const std::vector<int>& idxes_collide,
                        int o,
                        ob::State* state_curr,
                        const og::PathGeometric &selfish_path,
                        og::PathGeometric& path_tmp);
                        */

    // Append a Dubins segment (the “selfish push”) into path_tmp.
    void appendDubinsSegment(int o,
                             const ReloPush::StatePathPtr &interp,
                             ob::State *state_curr,
                             og::PathGeometric &path_tmp,
                             double pre_push_dist);

    RobotObjectSetup &env_;
    std::vector<RobotObjectSetup::Object> defs_; // jeeho
    ompl::base::SpaceInformationPtr si_single_;
    ompl::base::SpaceInformationPtr si_single4all_;
    ompl::base::SpaceInformationPtr si_single4clear_;
    ompl::base::SpaceInformationPtr si_all4all_;
    int n_objs_;
    double solve_time_;

    cv::Mat colors_;

#if DEBUG
    BulletSimulationGui sim_;
#else
    BulletSimulation sim_;
#endif

    std::vector<int> sim_idxes_;

#if DEBUG
    BulletSimulationGui simgui_;    
    btCylinderShape* simgui_stick_;
    int              simgui_stick_idx_;
    std::vector<int> simgui_idxes_;
#endif

    std::vector<RobotObjectSetup::Object> &objects_;

    SoapRotators* soap_rotator_;

    const double thresh_goal;
    ompl::base::OptimizationObjectivePtr opt_inf;

    fnDoAction doAction;

    bool use_kino;

    btBoxShape* table_shape;
};

#endif
