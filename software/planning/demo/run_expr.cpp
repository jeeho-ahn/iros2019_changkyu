#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#define LOG BOOST_LOG_TRIVIAL(trace)

#include "../include/planner.hpp"
#include <config.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

const std::string cmake_dir = std::string(CMAKE_SOURCE_DIR);

namespace fs = boost::filesystem;

// 1) build the list of identical objects
static std::vector<RobotObjectSetup::Object> makeObjects(int n_objs, double box_width, double box_height) {
    std::vector<RobotObjectSetup::Object> objects(n_objs);
    for (auto &obj : objects) {
        obj.name      = "relo_box";
        obj.dims      = {0.3, box_width, box_height};
        obj.radius    = 0.5 * std::hypot(box_width, box_height);
        obj.q_offset.setEulerZYX(0, -89.9999/180.0 * M_PI, 0);
        obj.shape     = new btBoxShape(btVector3(obj.dims[0]*0.5, obj.dims[1]*0.5, obj.dims[2]*0.5));
        obj.z_offset  = 0.3 * 0.5;
        obj.mass      = 0.135;
    }
    return objects;
}

// 2) factory for each experiment name
static std::unique_ptr<RobotObjectSetup> makeEnvironment(
    const std::string &experiment,
    const std::vector<RobotObjectSetup::Object> &objects)
{
    if (experiment == "openspace_sim") {
        return std::make_unique<BoxSetup>(objects, 2.10);
    }
    if (experiment == "relopush") {
        return std::make_unique<BoxSetup>(objects, 0, 4, 0, 5.2);
    }
    if (experiment == "tabletop_kuka" || experiment == "tabletop_video") {
        return std::make_unique<KukaTableSetup>(objects);
    }
    static const std::unordered_map<std::string,std::vector<double>> workspaces = {
        {"redbox_kuka",    {0.43, 0.34, -0.095, 0.6123724, 0.6123724, -0.3535534, -0.3535534, 0.44, 0.22, 0.34}},
        {"bluebox_kuka",   {0.43,-0.36,-0.140,   0.615,     0.615,     0.348,      0.348,      0.90, 0.12, 0.60}},
        {"amazonbox_kuka", {0.43,-0.36,-0.140,   0.615,     0.615,     0.348,      0.348,      0.31, 0.12, 0.24}},
        {"rectbox_kuka",   {0.43,-0.36,-0.140,   0.615,     0.615,     0.348,      0.348,      1.20, 0.12, 0.90}}
    };
    auto it = workspaces.find(experiment);
    if (it != workspaces.end()) {
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
    for (int o = 1; o <= n_objs; ++o) {
        double x   = node["state"][0 + (o-1)*3].as<double>();
        double y   = node["state"][1 + (o-1)*3].as<double>();
        double yaw = node["state"][2 + (o-1)*3].as<double>();
        auto so = STATE_OBJECT(s,o);
        so->setX(x); so->setY(y); so->setYaw(yaw);
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
    xs.reserve(n_objs); ys.reserve(n_objs); yaws.reserve(n_objs);
    for (int o=1; o<=n_objs; ++o) {
        xs  .push_back(STATE_OBJECT(state_init,o)->getX());
        ys  .push_back(STATE_OBJECT(state_init,o)->getY());
        yaws.push_back(STATE_OBJECT(state_init,o)->getYaw());
    }
    for (int o=1; o<=n_objs; ++o) {
        double x, y;
        if (o == 1) {
            x = STATE_OBJECT(state_goal,1)->getX();
            y = STATE_OBJECT(state_goal,1)->getY();
        } else {
            double dx = STATE_OBJECT(state_goal,o)->getX()
                        - STATE_OBJECT(state_goal,1)->getX();
            double dy = STATE_OBJECT(state_goal,o)->getY()
                        - STATE_OBJECT(state_goal,1)->getY();
            x = STATE_OBJECT(state_init,1)->getX() + dx;
            y = STATE_OBJECT(state_init,1)->getY() + dy;
        }
        // find nearest init-object
        int best = -1;
        double mind=INFINITY;
        for (int i=0; i<n_objs; ++i) {
            double d = std::hypot(x-xs[i], y-ys[i]);
            if (d < mind) { mind=d; best=i; }
        }
        auto so = STATE_OBJECT(state_init,o);
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
    if (plannerName.rfind("ours",0)==0) {
        planner.plan(si, sg, path, actions, do_merge);
    }
    else if (plannerName=="plrs") {
        planner.plan_plrs_jeeho(si, sg, path, actions);
    }
    else if (plannerName=="kino") {
        planner.UseKino();
        planner.plan(si, sg, path, actions, true);
    }
    else {
        std::cerr<<"[Error] Unknown planner "<<plannerName<<std::endl;
        std::exit(1);
    }
}

static std::pair<std::string,std::string>
makeInitGoalFiles(const std::string &dp_root,
                  const std::string &experiment,
                  int relopush_n,
                  std::vector<int> &out_ns)
{
    static const std::vector<int> allowed{4,5,6,9};
    if (std::find(allowed.begin(), allowed.end(), relopush_n) == allowed.end()) {
        throw std::runtime_error("Unsupported relopush_n: " + std::to_string(relopush_n));
    }

    // set the single-element ns
    out_ns = { relopush_n };

    // build init/goal paths
    char buf[256];
    sprintf(buf, "%s/input/%s/input_%dobj.init",
            dp_root.c_str(), experiment.c_str(), relopush_n);
    std::string fp_init(buf);

    sprintf(buf, "%s/input/%s/output_%dobj.goal",
            dp_root.c_str(), experiment.c_str(), relopush_n);
    std::string fp_goal(buf);

    return { fp_init, fp_goal };
}

int main(int argc, char* argv[])
{
    int id;
    string name_experiment;
    string name_planner;
    bool do_merge;
    bool vis;
    bool skip;
    vector<int> ns;
    vector<int> ks{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

    ///////////////
/*
    po::options_description desc("Example Usage");
    desc.add_options()
        ("help", "help")
        ("id,i",   po::value<int>(&id)->default_value(0), "id")
        ("idx,k",  po::value<vector<int>>(&ks)->multitoken(), "init #")
        ("vis,v",  po::value<bool>(&vis)->default_value(false), "vis")
        ("skip,s", po::value<bool>(&skip)->default_value(true), "skip")
        ("nobjs,n",po::value<vector<int> >(&ns)->multitoken(), "numbers of objects")
        //("merge,m",po::value<bool>(&do_merge)->default_value(true), "do_merge")
        ("expr,e", po::value<string>(&name_experiment)->default_value("tabletop_kuka"), 
                   "the name of experiment [openspace_sim, tabletop_kuka, redbox_kuka, amazonbox_kuka]")
        ("planner,p", po::value<string>(&name_planner)->default_value("ours"), 
                   "the name of planner [ours, plrs, mopl]");



    //const char* args[] = {"reloPush", "-i", "0", "-k", "1", "-v", "true", "-n", "2", "-e", "amazonbox_kuka", "-p", "plrs"};
    //argv = const_cast<char**>(args);
    //argc = 5;


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if( vm.count("help") ) 
    {        
        cout << desc << endl;
        return 0;
    }
    */
    /////////////////

    id = 0;
    name_experiment = "relopush";
    name_planner = "plrs";
    vis = true;
    skip = false;
    ns = {5}; // num of object
    ks = {1};

    std::string fp_init, fp_goal;
    char fp_res[256];

    /*
    sprintf(fp_init,"%s/input/%s/%s.n%d.%03d.init",                 dp_root.c_str(), name_experiment.c_str(), "dove_beauty_bar", n_objs, i);
    sprintf(fp_goal,"%s/input/%s/%s.n%d.%03d.goal",                 dp_root.c_str(), name_experiment.c_str(), "dove_beauty_bar", n_objs, 1);
    sprintf(fp_res,"%s/result/%s/now/%s/%s.%s.n%d.%03d.id%03d.res", dp_root.c_str(), name_experiment.c_str(), name_planner.c_str(), name_planner.c_str(), "dove_beauty_bar", n_objs, i, id);
    */
    // manual file name
    string dp_root = cmake_dir;
    fp_init = dp_root + "/input/relopush/input_6obj.init";
    fp_goal = dp_root + "/input/relopush/output_6obj.goal";

    // preset
    int relopush_n = 6; // or whatever you choose
    try {
        std::tie(fp_init, fp_goal)
          = makeInitGoalFiles(dp_root, name_experiment, relopush_n, ns);
    }
    catch (const std::exception &e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }

    // override
    /*
    ns={2};
    fp_init = dp_root + "/input/tabletop_kuka/dove_beauty_bar.n2.004.init";
    fp_goal = dp_root + "/input/tabletop_kuka/dove_beauty_bar.n2.001.goal";
    name_experiment = "tabletop_kuka";
    */

    //string dp_root = "/home/jeeho/cpp_ws/iros2019_changkyu/software/planning";

    if( name_planner.compare("ours_selfish")==0 )
    {
        do_merge = false;
    }
    else if( name_planner.compare("ours_pushing")==0 )
    {
        do_merge = true;
    }

/*
    int i_max = 20;
    if( name_experiment.compare("bluebox_kuka")==0 )
    {
        i_max = 10;
    }
*/        
    //for( int i=1; i<=i_max; i++ )    


    for (auto i : ks) {
        for (auto n_objs : ns) {
            auto objects = makeObjects(n_objs, box_width, box_height);
            auto env     = makeEnvironment(name_experiment, objects);
            if (!env) {
                std::cerr << "[Error] Unknown experiment: " << name_experiment << std::endl;
                return 1;
            }

            char fp_res[512];
            sprintf(fp_res, "%s/result/%s/now/%s/%s.%s.n%d.%03d.id%03d.res",
                    dp_root.c_str(),
                    name_experiment.c_str(),
                    name_planner.c_str(),
                    name_planner.c_str(),
                    "dove_beauty_bar",
                    n_objs, i, id);

            fs::path path_res(fp_res);
            if (skip && fs::is_regular_file(path_res)) {
                std::cout << "[SKIP] " << fp_res << " already exists\n";
                continue;
            }

            std::cout << "[READ] " << fp_init << std::endl;
            std::cout << "[READ] " << fp_goal << std::endl;

            auto state_init = env->allocState();
            auto state_goal = env->allocState();
            loadYamlState(fp_init,  state_init, n_objs, /*isInit=*/true);
            loadYamlState(fp_goal,  state_goal, n_objs, /*isInit=*/false);

            if (name_experiment == "bluebox_kuka" || name_experiment == "rectbox_kuka") {
                rematchBlueboxInit(state_init, state_goal, n_objs);
            }

            og::PathGeometric path(env->getAllForAllSpaceInformation());
            std::vector<Planner::Action> actions;
            Planner planner(*env);

            clock_t t0 = clock();
            runChosenPlanner(name_planner, planner, state_init, state_goal, path, actions, do_merge);
            double elapsed = double(clock() - t0) / CLOCKS_PER_SEC;

            planner.save_plan(fp_res, name_planner, n_objs, elapsed, -1,
                              state_init, state_goal, path, actions);

            std::cout << "Result path length: " << path.length() << std::endl;

            if (vis) {
                cv::Mat img = (name_experiment == "relopush")
                    ? cv::Mat::zeros(vis_height, vis_width, CV_8UC3)
                    : cv::Mat::zeros(1000, 1000, CV_8UC3);
                env->visualizeSetup(img);
                planner.visualizePath(img, path, 0, 4, 0, 5.2);
                cv::imshow("vis", img);
                cv::waitKey();
            }

            env->freeState(state_init);
            env->freeState(state_goal);
        }
    }


    /*
    for( int k=0;k<ks.size(); k++) // jeeho: for each instance?
    {   
        int i=ks[k];
        //if( k!=-1 && i!=k ) continue;
        for( int j=0; j<ns.size(); j++ ) // jeeho: for each 'number of objects'
        {
            int n = ns[j];            // jeeho: corresponding number of objects
            string name = "ours"; // jeeho: ??

            int n_objs = n;
            //// jeeho: Object shape defined here
            vector<RobotObjectSetup::Object> objects(n_objs);
            for( int o=0; o<n_objs; o++ )
            {
                //objects[o].name = "dove_beauty_bar";
                objects[o].name = "relo_box";
                objects[o].dims.resize(3);
                objects[o].dims[0] = 0.3; // z dim?
                objects[o].dims[1] = box_width; // width
                objects[o].dims[2] = box_height; // height
                //objects[o].radius  = sqrt(0.096*0.096 + 0.066*0.066)*0.5;
                objects[o].radius  = sqrt(box_height*box_height + box_width*box_width)*0.5; // jeeho
                objects[o].q_offset.setEulerZYX(0, -89.9999/ 180.0 * M_PI, 0);
                objects[o].shape = new btBoxShape(btVector3(objects[o].dims[0]*0.5, 
                                                            objects[o].dims[1]*0.5, 
                                                            objects[o].dims[2]*0.5));
                objects[o].z_offset = 0.3 * 0.5;
                objects[o].mass = 0.135; // jeeho: not changed
            }
            
            RobotObjectSetup* env;
            if( name_experiment.compare("openspace_sim")==0 )
            {
                env = new BoxSetup(objects,2.10);
            }
            else if( name_experiment.compare("relopush")==0 ) // jeeho: setup for relopush
            {
                env = new BoxSetup(objects,0,4,0,5.2);
            }
            else if( name_experiment.compare("tabletop_kuka")==0 )
            {
                env = new KukaTableSetup(objects);
            }
            else if( name_experiment.compare("tabletop_video")==0 )
            {
                env = new KukaTableSetup(objects);
            }
            else if( name_experiment.compare("redbox_kuka")==0 )
            {
                vector<double> workspace{0.43, 0.34, -0.095, 0.6123724, 0.6123724, -0.3535534, -0.3535534, 0.44, 0.22, 0.34};                
                env = new KukaBoxSetup(objects,workspace);
            }
            else if( name_experiment.compare("bluebox_kuka")==0 )
            {
                //vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.47, 0.12, 0.27};                
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.90, 0.12, 0.60};
                env = new KukaBoxSetup(objects,workspace);
            }
            else if( name_experiment.compare("amazonbox_kuka")==0 )
            {
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.31, 0.12, 0.24};
                env = new KukaBoxSetup(objects,workspace);                
            }
            else if( name_experiment.compare("rectbox_kuka")==0 )
            {
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 1.20, 0.12, 0.90};
                env = new KukaBoxSetup(objects,workspace);                
            }
            else
            {
                cerr << "[Error] Unknown Experiment: " << name_experiment << endl;
                return 0;
            }

            //char fp_init[256], fp_goal[256], fp_res[256];


            sprintf(fp_res,"%s/result/%s/now/%s/%s.%s.n%d.%03d.id%03d.res", dp_root.c_str(), name_experiment.c_str(), name_planner.c_str(), name_planner.c_str(), "dove_beauty_bar", n_objs, i, id);

            fs::path path_res(fp_res);
            if( skip && fs::is_regular_file(path_res)==true )
            {
                cout << "[SKIP]" << fp_res << " already exists" << endl;
                continue;
            }

            cout << "[READ] " << fp_init << endl;
            cout << "[READ] " << fp_goal << endl;            


            ompl::base::State* state_init = env->allocState();
            ompl::base::State* state_goal = env->allocState();

            YAML::Node node_init = YAML::LoadFile(fp_init);
            YAML::Node node_goal = YAML::LoadFile(fp_goal);

            STATE_ROBOT(state_init) = 3; // jeeho: ??
            STATE_ROBOT(state_goal) = 1;


            for (int o = 1; o <= n_objs; o++) {
                double x_init = node_init["state"][0 + (o - 1) * 3].as<double>();
                double y_init = node_init["state"][1 + (o - 1) * 3].as<double>();
                double yaw_init = node_init["state"][2 + (o - 1) * 3].as<double>();

                STATE_OBJECT(state_init, o)->setX(x_init);
                STATE_OBJECT(state_init, o)->setY(y_init);
                STATE_OBJECT(state_init, o)->setYaw(yaw_init);

                double x_goal = node_goal["state"][0 + (o - 1) * 3].as<double>();
                double y_goal = node_goal["state"][1 + (o - 1) * 3].as<double>();
                double yaw_goal = node_goal["state"][2 + (o - 1) * 3].as<double>();

                STATE_OBJECT(state_goal, o)->setX(x_goal);
                STATE_OBJECT(state_goal, o)->setY(y_goal);
                STATE_OBJECT(state_goal, o)->setYaw(yaw_goal);

                // Output the values for monitoring
                std::cout << "Object " << o << " initial state: x = " << x_init
                          << ", y = " << y_init << ", yaw = " << yaw_init << std::endl;
                std::cout << "Object " << o << " goal state: x = " << x_goal
                          << ", y = " << y_goal << ", yaw = " << yaw_goal << std::endl;
            }



            if( name_experiment.compare("bluebox_kuka")==0  ||
                name_experiment.compare("rectbox_kuka")==0    )
            {
#if 0
                vector<double> xs,ys,yaws;
                for( int o=1; o<=n_objs; o++ )
                {
                    xs.push_back(  STATE_OBJECT(state_init,o)->getX());
                    ys.push_back(  STATE_OBJECT(state_init,o)->getY());
                    yaws.push_back(STATE_OBJECT(state_init,o)->getYaw());
                }
                for( int o=1; o<=n_objs; o++ )
                {
                    double x   = STATE_OBJECT(state_goal,o)->getX();
                    double y   = STATE_OBJECT(state_goal,o)->getY();
                    double yaw = STATE_OBJECT(state_goal,o)->getYaw();
                
                    int i_min = -1;
                    double dist_min = INFINITY;
                    for( int i=0; i<n_objs; i++ )
                    {
                        double dist = sqrt((x-xs[i])*(x-xs[i])+(y-ys[i])*(y-ys[i]));
                        if( dist_min > dist )
                        {
                            dist_min = dist;
                            i_min = i;
                        }
                    }

                    STATE_OBJECT(state_init,o)->setX(xs[i_min]);
                    STATE_OBJECT(state_init,o)->setY(ys[i_min]);
                    STATE_OBJECT(state_init,o)->setYaw(yaws[i_min]);

                    xs[i_min] = INFINITY;
                    ys[i_min] = INFINITY;
                }
#else 
                vector<double> xs,ys,yaws;
                for( int o=1; o<=n_objs; o++ )
                {
                    xs.push_back(  STATE_OBJECT(state_init,o)->getX());
                    ys.push_back(  STATE_OBJECT(state_init,o)->getY());
                    yaws.push_back(STATE_OBJECT(state_init,o)->getYaw());
                }
                for( int o=1; o<=n_objs; o++ )
                {

                    //double x   = STATE_OBJECT(state_goal,o)->getX();
                    //double y   = STATE_OBJECT(state_goal,o)->getY();
                    //double yaw = STATE_OBJECT(state_goal,o)->getYaw();

                    double x,y;
                    if( o==1 )
                    {
                        x   = STATE_OBJECT(state_goal,o)->getX();
                        y   = STATE_OBJECT(state_goal,o)->getY();
                    }
                    else
                    {
                        double x_del = STATE_OBJECT(state_goal,o)->getX() - STATE_OBJECT(state_goal,1)->getX();
                        double y_del = STATE_OBJECT(state_goal,o)->getY() - STATE_OBJECT(state_goal,1)->getY();
                        x = STATE_OBJECT(state_init,1)->getX() + x_del;
                        y = STATE_OBJECT(state_init,1)->getY() + y_del;
                    }
                
                    int i_min = -1;
                    double dist_min = INFINITY;
                    for( int i=0; i<n_objs; i++ )
                    {
                        double dist = sqrt((x-xs[i])*(x-xs[i])+(y-ys[i])*(y-ys[i]));
                        if( dist_min > dist )
                        {
                            dist_min = dist;
                            i_min = i;
                        }
                    }

                    STATE_OBJECT(state_init,o)->setX(xs[i_min]);
                    STATE_OBJECT(state_init,o)->setY(ys[i_min]);
                    STATE_OBJECT(state_init,o)->setYaw(yaws[i_min]);

                    xs[i_min] = INFINITY;
                    ys[i_min] = INFINITY;
                }
#endif
            }

            og::PathGeometric path(env->getAllForAllSpaceInformation());
            vector<Planner::Action> actions;

            Planner planner(*env);

            clock_t begin, end;
            begin = clock();
            
            if( name_planner.compare(0,4,"ours")==0 ) // jeeho: ours, ours_selfish, ours_pushing
            {
                //ifstream ifs("/home/cs1080/tmp.save");
                //planner.load_precomputed_planners(ifs);
                //ifs.close();
                planner.plan(state_init, state_goal, path, actions, do_merge);
            }
            else if( name_planner.compare("plrs")==0 )
            {
                //planner.plan_plRS(state_init, state_goal, path, actions);
                planner.plan_plrs_jeeho(state_init, state_goal, path, actions);
            }
            else if( name_planner.compare("mopl")==0 )
            {

            }
            else if( name_planner.compare("kino")==0 )
            {
                planner.UseKino();
                planner.plan(state_init, state_goal, path, actions, true);
            }
            else                
            {
                cerr << "[Error] Unknown planner: " << name_planner << endl;
                return 0;
            }

            end = clock();
            double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;          
            planner.save_plan( fp_res, name, n_objs, time_spent, -1, state_init, state_goal, path, actions );

            std::cout << "Result path length: " << path.length() << std::endl;

            if( vis )
            {
                cv::Mat img;
                if(name_experiment.compare("relopush")==0)
                    img = cv::Mat::zeros(vis_height,vis_width,CV_8UC3); // jeeho: relopush setup
                else
                    img =cv::Mat::zeros(1000,1000,CV_8UC3);

                env->visualizeSetup(img);
                //planner.visualizePath(img,path,(name_experiment.compare("relopush")==0));
                planner.visualizePath(img,path,0,4,0,5.2);
                cv::imshow("vis",img);
                cv::waitKey();
            }
            

            env->freeState(state_init);
            env->freeState(state_goal);
            delete env;
        }
    }
    */
    return 0;
}
