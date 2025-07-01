#ifndef RUN_EXPR_HELPER_HPP
#define RUN_EXPR_HELPER_HPP



#include <InstanceParser.hpp>


using namespace std;

const std::string cmake_dir = std::string(CMAKE_SOURCE_DIR);

namespace fs = boost::filesystem;

struct ObjectGoalPair
{
    std::string objectName;
    std::string goalName;

    ObjectGoalPair()
    {}

    ObjectGoalPair(std::string obj, std::string goal)
        : objectName(obj), goalName(goal)
    {}
};

// 1) build the list of identical objects
static std::vector<RobotObjectSetup::Object> makeObjects(int n_objs, double box_width, double box_height)
{
    std::vector<RobotObjectSetup::Object> objects(n_objs);
    for (auto &obj : objects)
    {
        obj.name = "relo_box";
        obj.dims = {0.3, box_width, box_height};
        obj.radius = 0.5 * std::hypot(box_width, box_height);
        obj.q_offset.setEulerZYX(0, -89.9999 / 180.0 * M_PI, 0);
        obj.shape = new btBoxShape(btVector3(obj.dims[0] * 0.5, obj.dims[1] * 0.5, obj.dims[2] * 0.5));
        obj.z_offset = 0.3 * 0.5;
        obj.mass = 0.135;
    }
    return objects;
}

// 2) factory for each experiment name
static std::unique_ptr<RobotObjectSetup> makeEnvironment(
    const std::string &experiment,
    const std::vector<RobotObjectSetup::Object> &objects)
{
    if (experiment == "openspace_sim")
    {
        return std::make_unique<BoxSetup>(objects, 2.10);
    }
    if (experiment == "relopush")
    {
        return std::make_unique<BoxSetup>(objects, 0, 4, 0, 5.2);
    }
    if (experiment == "tabletop_kuka" || experiment == "tabletop_video")
    {
        return std::make_unique<KukaTableSetup>(objects);
    }
    static const std::unordered_map<std::string, std::vector<double>> workspaces = {
        {"redbox_kuka", {0.43, 0.34, -0.095, 0.6123724, 0.6123724, -0.3535534, -0.3535534, 0.44, 0.22, 0.34}},
        {"bluebox_kuka", {0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.90, 0.12, 0.60}},
        {"amazonbox_kuka", {0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.31, 0.12, 0.24}},
        {"rectbox_kuka", {0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 1.20, 0.12, 0.90}}};
    auto it = workspaces.find(experiment);
    if (it != workspaces.end())
    {
        return std::make_unique<KukaBoxSetup>(objects, it->second);
    }
    return nullptr;
}

// 3) load a single YAML state file into an OMPL state
static void loadYamlState(const std::string &fp,
                          ompl::base::State *s,
                          int n_objs,
                          bool isInit)
{
    auto node = YAML::LoadFile(fp);
    STATE_ROBOT(s) = isInit ? 3 : 1;
    for (int o = 1; o <= n_objs; ++o)
    {
        double x = node["state"][0 + (o - 1) * 3].as<double>();
        double y = node["state"][1 + (o - 1) * 3].as<double>();
        double yaw = node["state"][2 + (o - 1) * 3].as<double>();
        auto so = STATE_OBJECT(s, o);
        so->setX(x);
        so->setY(y);
        so->setYaw(yaw);
        std::cout << (isInit ? "[INIT] " : "[GOAL] ")
                  << "Obj " << o
                  << " x=" << x << " y=" << y << " yaw=" << yaw
                  << std::endl;
    }
}

// 4) special “bluebox” re-assignment logic
static void rematchBlueboxInit(ompl::base::State *state_init,
                               ompl::base::State *state_goal,
                               int n_objs)
{
    std::vector<double> xs, ys, yaws;
    xs.reserve(n_objs);
    ys.reserve(n_objs);
    yaws.reserve(n_objs);
    for (int o = 1; o <= n_objs; ++o)
    {
        xs.push_back(STATE_OBJECT(state_init, o)->getX());
        ys.push_back(STATE_OBJECT(state_init, o)->getY());
        yaws.push_back(STATE_OBJECT(state_init, o)->getYaw());
    }
    for (int o = 1; o <= n_objs; ++o)
    {
        double x, y;
        if (o == 1)
        {
            x = STATE_OBJECT(state_goal, 1)->getX();
            y = STATE_OBJECT(state_goal, 1)->getY();
        }
        else
        {
            double dx = STATE_OBJECT(state_goal, o)->getX() - STATE_OBJECT(state_goal, 1)->getX();
            double dy = STATE_OBJECT(state_goal, o)->getY() - STATE_OBJECT(state_goal, 1)->getY();
            x = STATE_OBJECT(state_init, 1)->getX() + dx;
            y = STATE_OBJECT(state_init, 1)->getY() + dy;
        }
        // find nearest init-object
        int best = -1;
        double mind = INFINITY;
        for (int i = 0; i < n_objs; ++i)
        {
            double d = std::hypot(x - xs[i], y - ys[i]);
            if (d < mind)
            {
                mind = d;
                best = i;
            }
        }
        auto so = STATE_OBJECT(state_init, o);
        so->setX(xs[best]);
        so->setY(ys[best]);
        so->setYaw(yaws[best]);
        xs[best] = ys[best] = INFINITY;
    }
}

// 5) choose & run planner
static void runChosenPlanner(const std::string &plannerName,
                             Planner &planner,
                             ompl::base::State *si,
                             ompl::base::State *sg,
                             og::PathGeometric &path,
                             std::vector<Planner::Action> &actions,
                             bool do_merge)
{
    if (plannerName.rfind("ours", 0) == 0)
    {
        planner.plan(si, sg, path, actions, do_merge);
    }
    else if (plannerName == "plrs")
    {
        planner.plan_plrs_jeeho(si, sg, path, actions);
    }
    else if (plannerName == "kino")
    {
        planner.UseKino();
        planner.plan(si, sg, path, actions, true);
    }
    else
    {
        std::cerr << "[Error] Unknown planner " << plannerName << std::endl;
        std::exit(1);
    }
}

static std::pair<std::string, std::string>
makeInitGoalFiles(const std::string &dp_root,
                  const std::string &experiment,
                  int relopush_n,
                  std::vector<int> &out_ns)
{
    static const std::vector<int> allowed{4, 5, 6, 9};
    if (std::find(allowed.begin(), allowed.end(), relopush_n) == allowed.end())
    {
        throw std::runtime_error("Unsupported relopush_n: " + std::to_string(relopush_n));
    }

    // set the single-element ns
    out_ns = {relopush_n};

    // build init/goal paths
    char buf[256];
    sprintf(buf, "%s/input/%s/input_%dobj.init",
            dp_root.c_str(), experiment.c_str(), relopush_n);
    std::string fp_init(buf);

    sprintf(buf, "%s/input/%s/output_%dobj.goal",
            dp_root.c_str(), experiment.c_str(), relopush_n);
    std::string fp_goal(buf);

    return {fp_init, fp_goal};
}

// 1) Build the result‐file name
static std::string makeResultFile(const std::string &dp_root,
                                  const std::string &experiment,
                                  const std::string &planner,
                                  const std::string &objectName,
                                  int n_objs, int instance, int id)
{
    char buf[512];
    sprintf(buf,
            "%s/result/%s/now/%s/%s.%s.n%d.%03d.id%03d.res",
            dp_root.c_str(),
            experiment.c_str(),
            planner.c_str(),
            planner.c_str(),
            objectName.c_str(),
            n_objs, instance, id);
    return std::string(buf);
}

// 2) Optionally skip if file exists
static bool skipIfExists(const std::string &fp_res, bool skip)
{
    fs::path p(fp_res);
    if (skip && fs::is_regular_file(p))
    {
        std::cout << "[SKIP] " << fp_res << " already exists\n";
        return true;
    }
    return false;
}

// 3) Allocate & load both init/goal states
static std::pair<ompl::base::State *, ompl::base::State *>
allocateAndLoad(RobotObjectSetup *env,
                const std::string &fp_init,
                const std::string &fp_goal,
                int n_objs)
{
    auto s_init = env->allocState();
    auto s_goal = env->allocState();
    loadYamlState(fp_init, s_init, n_objs, /*isInit=*/true);
    loadYamlState(fp_goal, s_goal, n_objs, /*isInit=*/false);
    return {s_init, s_goal};
}



//------------------------------------------------------------------------------
// Run planning for each object‐count in 'ns'
//------------------------------------------------------------------------------
static void runObjectLoop(
    int instance, int id,
    const std::vector<int> &ns,
    const std::string &dp_root,
    const std::string &name_experiment,
    const std::string &name_planner,
    const std::string &fp_init,
    const std::string &fp_goal,
    bool skip,
    bool vis,
    int vis_height, int vis_width,
    bool do_merge,
    double box_width, double box_height)
{
    for (auto n_objs : ns)
    {
        // 1) make objects & env
        auto objects = makeObjects(n_objs, box_width, box_height);
        auto env = makeEnvironment(name_experiment, objects);
        if (!env)
        {
            std::cerr << "[Error] Unknown experiment: "
                      << name_experiment << std::endl;
            std::exit(1);
        }

        // 2) build + maybe skip result filename
        std::string fp_res = makeResultFile(
            dp_root,
            name_experiment,
            name_planner,
            "dove_beauty_bar",
            n_objs, instance, id);
        if (skipIfExists(fp_res, skip))
            continue;

        // 3) report input files
        std::cout << "[READ] " << fp_init << "\n"
                  << "[READ] " << fp_goal << "\n";

        // 4) allocate & load states
        //auto [state_init, state_goal] = allocateAndLoad(env.get(), fp_init, fp_goal, n_objs);

        auto [state_init, state_goal] = allocateAndLoadSingle(env.get(), fp_init, /*line=*/instance, /*n_objs=*/n_objs);

    
        // 5) special bluebox rematch
        if (name_experiment == "bluebox_kuka" ||
            name_experiment == "rectbox_kuka")
        {
            rematchBlueboxInit(state_init, state_goal, n_objs);
        }

        // 6) plan
        og::PathGeometric path(env->getAllForAllSpaceInformation());
        std::vector<Planner::Action> actions;
        Planner planner(*env, std::move(objects));
        clock_t t0 = clock();
        runChosenPlanner(
            name_planner,
            planner,
            state_init, state_goal,
            path, actions,
            do_merge);
        double elapsed = double(clock() - t0) / CLOCKS_PER_SEC;

        // 7) save + report
        planner.save_plan(
            fp_res,
            name_planner,
            n_objs,
            elapsed,
            -1,
            state_init,
            state_goal,
            path,
            actions);
        std::cout << "Result path length: "
                  << path.length() << std::endl;



        // Print summary
        double total_path_length = 0.0;
        double action1_length = 0.0;
        for (size_t i = 1; i < actions.size(); ++i) {
            // Compute distance between consecutive actions
            double dx = actions[i].x - actions[i-1].x;
            double dy = actions[i].y - actions[i-1].y;
            double segment_length = std::sqrt(dx*dx + dy*dy);

            total_path_length += segment_length;
            // If previous action is ACTION_TRANSFER (type 1)
            if (actions[i-1].type == Planner::TYPE_ACTION::ACTION_TRANSFER) {
                action1_length += segment_length;
            }
        }

        std::cout << "\n\t---------- Results ----------" << std::endl;
        std::cout << "Total path length: " << total_path_length << std::endl;
        std::cout << "Path length for action type 1: " << action1_length << std::endl;
        std::cout << "Planning Time: " << elapsed << std::endl;


        // Save summary statistics to a file
        std::string resultStatFile = dp_root + "/result/jeeho/results_obj" + std::to_string(n_objs) + ".txt";
        std::cout << "Saving to: " << resultStatFile << std::endl;
        std::ofstream fout_stat(resultStatFile.c_str(), std::ios::app);
        fout_stat << "===\n";
        fout_stat << "index:" << instance << "\n";
        fout_stat << "planning_time(s):" << elapsed << "\n";
        fout_stat << "total_length(m):" << total_path_length << "\n";
        fout_stat << "transfer_length(m):" << action1_length << "\n";
        fout_stat.close();

        std::cout << "file saved" << std::endl;



        // 8) optional visualization
        if (vis)
        {
            cv::Mat img = (name_experiment == "relopush")
                              ? cv::Mat::zeros(vis_height, vis_width, CV_8UC3)
                              : cv::Mat::zeros(1000, 1000, CV_8UC3);
            env->visualizeSetup(img);
            planner.visualizePath(img, path, 0, 4, 0, 5.2);
            cv::imshow("vis", img);
            cv::waitKey();
        }

        // 9) cleanup
        env->freeState(state_init);
        env->freeState(state_goal);
    }
}

#endif // RUN_EXPR_HELPER_HPP
