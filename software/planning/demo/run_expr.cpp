#include <run_expr_helper.hpp>
#include <ompl/util/Console.h>

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



int count_objects_in_mo_section(const std::string& line) {
    size_t mo_start = line.find("mo:");
    if (mo_start == std::string::npos) return 0;

    size_t mo_end = line.find('!', mo_start);
    std::string mo_section = line.substr(mo_start + 3, mo_end - (mo_start + 3));

    std::stringstream ss(mo_section);
    std::string object_entry;
    int count = 0;

    while (std::getline(ss, object_entry, ';')) {
        if (!object_entry.empty()) {
            ++count;
        }
    }

    return count;
}

int main(int argc, char* argv[])
{
    ompl::msg::noOutputHandler();
    int id;
    string name_experiment;
    string name_planner;
    bool do_merge;
    bool vis;
    bool skip;
    vector<int> ns;
    vector<int> ks{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

    id = 0;
    name_experiment = "relopush";
    name_planner = "plrs";
    vis = false;
    skip = false;
    ns = {8}; // num of object
    ks = {1};

    std::string fp_init, fp_goal;
    string dp_root = cmake_dir;
    //char fp_res[256];

    std::string inst_file;
    size_t inst_idx = 61;
    bool use_single = false;
    bool use_rrt = false; // dubins-based rrt


    if (argc == 1)
    {
        // no args: keep the old two-file behavior
        //fp_init = dp_root + "/input/relopush/input_6obj.init";
        fp_init = dp_root + "/input/relopush/iros_obj8.txt";
        fp_goal = dp_root + "/input/relopush/output_8obj.goal";
    }
    else if (argc >= 3)
    {
        // two args: single-file mode
        inst_file = argv[1];
        inst_idx = std::stoul(argv[2]);
        use_single = true;

        if(argc==4)
        {
            auto use_rrt_arg = std::stoi(argv[3]);
            if(use_rrt_arg==1)
                use_rrt = true;
        }

        fp_init = dp_root + "/input/relopush/" + inst_file;

        std::cout << "=== " << inst_file << ": " << inst_idx << " ===" << " use RRT: " << use_rrt << std::endl;
    }

    else
    {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << "                     # old init/goal mode\n"
                  << "  " << argv[0] << " <file> <instance#>   # single-file mode\n";
        return EXIT_FAILURE;
    }

    std::ifstream infile(fp_init.c_str());
    std::string line;
    std::getline(infile,line);
    int num_of_obj = count_objects_in_mo_section(line);

    ns = {num_of_obj};


    /*
    sprintf(fp_init,"%s/input/%s/%s.n%d.%03d.init",                 dp_root.c_str(), name_experiment.c_str(), "dove_beauty_bar", n_objs, i);
    sprintf(fp_goal,"%s/input/%s/%s.n%d.%03d.goal",                 dp_root.c_str(), name_experiment.c_str(), "dove_beauty_bar", n_objs, 1);
    sprintf(fp_res,"%s/result/%s/now/%s/%s.%s.n%d.%03d.id%03d.res", dp_root.c_str(), name_experiment.c_str(), name_planner.c_str(), name_planner.c_str(), "dove_beauty_bar", n_objs, i, id);
    */


    /*
    // preset
    int relopush_n = 6; // or whatever you choose
    try {
        std::tie(fp_init, fp_goal) = makeInitGoalFiles(dp_root, name_experiment, relopush_n, ns);
    }
    catch (const std::exception &e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }
    */

    // override
    /*
    ns={2};
    fp_init = dp_root + "/input/tabletop_kuka/dove_beauty_bar.n2.004.init";
    fp_goal = dp_root + "/input/tabletop_kuka/dove_beauty_bar.n2.001.goal";
    name_experiment = "tabletop_kuka";
    */

    // override
    //fp_init = dp_root + "/input/relopush/iros_obj6.txt";



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
    /*
    for (auto instance : ks)
    {
        runObjectLoop(
            instance, id,
            ns,
            dp_root,
            name_experiment,
            name_planner,
            fp_init, fp_goal,
            skip,
            vis,
            vis_height, vis_width,
            do_merge,
            box_width, box_height);
    }
            */

    runObjectLoop(
        inst_idx, id,
        ns,
        dp_root,
        name_experiment,
        name_planner,
        fp_init, fp_goal,
        skip,
        vis,
        vis_height, vis_width,
        do_merge, use_rrt,
        box_width, box_height);

    return 0;
}
