#include <boost/program_options.hpp>
#include <filesystem>
#include "erl_search_planning/amra_star.hpp"
#include "erl_search_planning/planning_interface_multi_resolutions.hpp"
#include "erl_search_planning/ltl_3d.hpp"
#include "erl_search_planning/llm_scene_graph_heuristic.hpp"
#include "erl_env/environment_ltl_scene_graph.hpp"
#include "erl_common/test_helper.hpp"

struct Options {
    std::string output_dir = {};
    std::string scene_graph_file = {};
    std::string map_data_dir = {};
    std::string automaton_file = {};
    std::string ap_file = {};
    std::string llm_heuristic_file = {};
    int init_grid_x = -1;
    int init_grid_y = -1;
    int init_grid_z = -1;
    bool occupancy_only = false;
    bool use_llm_heuristic = false;
    bool use_llm_heuristic_for_occupancy = true;
    bool use_llm_heuristic_for_object = true;
    bool use_llm_heuristic_for_room = true;
    bool use_llm_heuristic_for_floor = true;
    int repeat = 1;
    bool save_amra_log = true;
};

int
main(int argc, char *argv[]) {
    using namespace erl::env;
    using namespace erl::search_planning;

    Options options;

    try {
        namespace po = boost::program_options;
        po::options_description desc;
        po::positional_options_description positional_options;
        // clang-format off
        desc.add_options()
            ("help", "produce help message")
            ("output_dir", po::value<std::string>(&options.output_dir), "path to save the results.")
            ("scene_graph_file", po::value<std::string>(&options.scene_graph_file), "path to the YAML of scene graph.")
            ("map_data_dir", po::value<std::string>(&options.map_data_dir), "path to the map data directory.")
            ("automaton_file", po::value<std::string>(&options.automaton_file), "path to the automaton file.")
            ("ap_file", po::value<std::string>(&options.ap_file), "path to the AP description file.")
            ("llm_heuristic_file", po::value<std::string>(&options.llm_heuristic_file), "path to the LLM heuristic file.")
            ("init_grid_x", po::value<int>(&options.init_grid_x), "initial x position in grids.")
            ("init_grid_y", po::value<int>(&options.init_grid_y), "initial y position in grids.")
            ("init_grid_z", po::value<int>(&options.init_grid_z), "initial z position in grids.")
            ("occupancy_only", po::bool_switch(&options.occupancy_only)->default_value(options.occupancy_only), "only use occupancy information for planning.")
            ("use_llm_heuristic", po::bool_switch(&options.use_llm_heuristic)->default_value(options.use_llm_heuristic), "use LLM heuristic.")
            ("repeat", po::value<int>(&options.repeat), "repeat the experiment for multiple times.")
            ("save_amra_log", po::bool_switch(&options.save_amra_log)->default_value(options.save_amra_log), "save AMRA* log");
        // positional_options.add("tree-bt-file", 1);
        // clang-format on

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).positional(positional_options).run(), vm);

        if (vm.count("help")) {
            std::cout << "Usage: " << argv[0] << " [options] amra_ltl_scene_graph" << std::endl << desc << std::endl;
            return 0;
        }
        po::notify(vm);
        if (options.scene_graph_file.empty()) {
            std::cerr << "scene_graph_file is not provided." << std::endl;
            return 1;
        }
        if (options.map_data_dir.empty()) {
            std::cerr << "map_data_dir is not provided." << std::endl;
            return 1;
        }
        if (options.automaton_file.empty()) {
            std::cerr << "automaton_file is not provided." << std::endl;
            return 1;
        }
        if (options.ap_file.empty()) {
            std::cerr << "ap_file is not provided." << std::endl;
            return 1;
        }
        if (options.use_llm_heuristic && options.llm_heuristic_file.empty()) {
            std::cerr << "llm_heuristic_file is not provided." << std::endl;
            return 1;
        }
        if (options.init_grid_x < 0) {
            std::cerr << "init_grid_x is not provided." << std::endl;
            return 1;
        }
        if (options.init_grid_y < 0) {
            std::cerr << "init_grid_y is not provided." << std::endl;
            return 1;
        }
        if (options.init_grid_z < 0) {
            std::cerr << "init_grid_z is not provided." << std::endl;
            return 1;
        }
    } catch (std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    if (options.map_data_dir.empty()) {
        options.map_data_dir = std::filesystem::path(options.scene_graph_file).parent_path().string();
        ERL_INFO("map_data_dir is not provided, using the default value: %s", options.map_data_dir.c_str());
    }

    // create output dir
    if (!std::filesystem::exists(options.output_dir)) { std::filesystem::create_directories(options.output_dir); }
    // load scene graph
    auto scene_graph = std::make_shared<scene_graph::Building>();
    scene_graph->FromYamlFile(options.scene_graph_file);
    // load the env setting
    auto env_setting = std::make_shared<EnvironmentLTLSceneGraph::Setting>();
    env_setting->data_dir = options.map_data_dir;
    using AutFileType = FiniteStateAutomaton::Setting::FileType;
    env_setting->fsa = std::make_shared<FiniteStateAutomaton::Setting>(options.automaton_file, AutFileType::kSpotHoa);
    env_setting->LoadAtomicPropositions(options.ap_file);
    // create the environment
    if (options.occupancy_only) { env_setting->max_level = scene_graph::Node::Type::kNA; }
    auto env = std::make_shared<EnvironmentLTLSceneGraph>(scene_graph, env_setting);
    // get initial states, goal set and goal tolerance.
    auto init_q = int(env_setting->fsa->initial_state);
    Eigen::VectorXd start = env->GridToMetric(Eigen::Vector4i(options.init_grid_x, options.init_grid_y, options.init_grid_z, init_q));
    auto num_goals = long(env_setting->fsa->accepting_states.size());
    std::vector<Eigen::VectorXd> goals(num_goals);
    for (long i = 0; i < num_goals; ++i) {
        Eigen::VectorXd &goal = goals[i];
        goal.resize(4);
        goal[0] = 0;  // does not matter
        goal[1] = 0;  // does not matter
        goal[2] = 0;  // does not matter
        goal[3] = double(env_setting->fsa->accepting_states[i]);
    }
    double inf = std::numeric_limits<double>::infinity();
    Eigen::VectorXd goal_tolerance = Eigen::Vector4d(inf, inf, inf, 0);
    // get ltl heuristic and llm heuristic
    auto ltl_heuristic = std::make_shared<LinearTemporalLogicHeuristic3D>(env->GetFiniteStateAutomaton(), env->GetLabelMaps(), env->GetGridMapInfo());
    std::shared_ptr<LLMSceneGraphHeuristic> llm_heuristic = nullptr;
    if (options.use_llm_heuristic) {
        auto llm_heuristic_setting = std::make_shared<LLMSceneGraphHeuristic::Setting>();
        llm_heuristic_setting->FromYamlFile(options.llm_heuristic_file);
        llm_heuristic = std::make_shared<LLMSceneGraphHeuristic>(llm_heuristic_setting, env);
    }
    std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics;
    if (options.occupancy_only) {
        heuristics = {
            {ltl_heuristic, 0},
            {ltl_heuristic, 1},
        };
        if (options.use_llm_heuristic) { heuristics.push_back({llm_heuristic, 1}); }
    } else {
        heuristics = {
            {ltl_heuristic, 0},
            {ltl_heuristic, 1},  // occupancy
            {ltl_heuristic, 2},  // object
            {ltl_heuristic, 3},  // room
            {ltl_heuristic, 4},  // floor
        };
        if (options.use_llm_heuristic) {
            if (options.use_llm_heuristic_for_occupancy) { heuristics.push_back({llm_heuristic, 1}); }
            if (options.use_llm_heuristic_for_object) { heuristics.push_back({llm_heuristic, 2}); }
            if (options.use_llm_heuristic_for_room) { heuristics.push_back({llm_heuristic, 3}); }
            if (options.use_llm_heuristic_for_floor) { heuristics.push_back({llm_heuristic, 4}); }
        }
    }
    // create the planner
    auto planning_interface = std::make_shared<PlanningInterfaceMultiResolutions>(env, heuristics, start, goals, std::vector<Eigen::VectorXd>{goal_tolerance});
    auto amra_setting = std::make_shared<amra_star::AMRAStar::Setting>();
    amra_setting->log = false;
    std::shared_ptr<amra_star::Output> result;
    for (int i = 0; i < options.repeat; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        amra_star::AMRAStar planner(planning_interface, amra_setting);
        auto t1 = std::chrono::high_resolution_clock::now();
        result = planner.Plan();
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "[" << i << "]Planner construction time: " << std::chrono::duration<double, std::micro>(t1 - t0).count() << " us." << std::endl;
        std::cout << "[" << i << "]Planning time: " << std::chrono::duration<double, std::micro>(t2 - t1).count() << " us." << std::endl;
    }
    // save the result
    std::filesystem::path output_dir = options.output_dir;
    if (options.save_amra_log) {
        amra_setting->log = true;
        amra_star::AMRAStar planner(planning_interface, amra_setting);
        result = planner.Plan();
        result->Save(output_dir / "amra.solution");
    }
    // draw path
    for (auto &itr: result->paths) {
        uint32_t plan_itr = itr.first;
        Eigen::Matrix4Xd amra_path = itr.second;
        long num_points = amra_path.cols();
        std::unordered_map<int, std::vector<cv::Point2i>> cv_paths;  // floor -> path
        cv_paths.reserve(num_points);
        for (long i = 0; i < num_points; ++i) {
            Eigen::Vector4d p = amra_path.col(i);
            Eigen::Vector4i grid = env->MetricToGrid(p);
            cv_paths[grid[2]].emplace_back(grid[1], grid[0]);
        }
        std::unordered_map<int, cv::Mat> cat_maps;  // floor -> map
        for (auto &[floor_num, cv_path]: cv_paths) {
            auto &cat_map = cat_maps[floor_num];
            cat_map = erl::common::ColorGrayCustom(scene_graph->LoadCatMap(options.map_data_dir, floor_num));
            cv::polylines(cat_map, cv_path, false, cv::Scalar(0, 0, 255), 2);
            cv::imshow(erl::common::AsString("plan_", plan_itr, "_floor_", floor_num), cat_map);
            cv::imwrite(output_dir / erl::common::AsString("plan_", plan_itr, "_floor_", floor_num, ".png"), cat_map);
        }
    }

    return 0;
}
