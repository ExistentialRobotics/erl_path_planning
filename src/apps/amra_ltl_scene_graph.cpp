#include "erl_common/yaml.hpp"
#include "erl_env/environment_ltl_scene_graph.hpp"
#include "erl_path_planning/amra_star.hpp"
#include "erl_path_planning/llm_scene_graph_heuristic.hpp"
#include "erl_path_planning/ltl_3d_heuristic.hpp"
#include "erl_path_planning/search_planning_interface.hpp"

#include <boost/program_options.hpp>

#include <filesystem>

template<typename Dtype>
struct Options : public erl::common::Yamlable<Options<Dtype>> {
    std::string output_dir = {};
    std::string scene_graph_file = {};
    std::string map_data_dir = {};
    std::string automaton_file = {};
    std::string ap_file = {};
    std::string llm_heuristic_file = {};
    int init_grid_x = -1;
    int init_grid_y = -1;
    int init_grid_z = -1;
    Dtype robot_radius = 0.0;
    Dtype object_reach_radius = 0.6;
    erl::env::scene_graph::NodeType max_level = erl::env::scene_graph::NodeType::kFloor;
    std::string ltl_heuristic_layout = {};
    std::string llm_heuristic_layout = {};
    int repeat = 1;
    bool save_amra_log = false;
    bool hold_for_visualization = false;

    template<typename T>
    void
    Load(const Options<T> &options) {
        output_dir = options.output_dir;
        scene_graph_file = options.scene_graph_file;
        map_data_dir = options.map_data_dir;
        automaton_file = options.automaton_file;
        ap_file = options.ap_file;
        llm_heuristic_file = options.llm_heuristic_file;
        init_grid_x = options.init_grid_x;
        init_grid_y = options.init_grid_y;
        init_grid_z = options.init_grid_z;
        robot_radius = static_cast<Dtype>(options.robot_radius);
        object_reach_radius = static_cast<Dtype>(options.object_reach_radius);
        max_level = options.max_level;
        ltl_heuristic_layout = options.ltl_heuristic_layout;
        llm_heuristic_layout = options.llm_heuristic_layout;
        repeat = options.repeat;
        save_amra_log = options.save_amra_log;
        hold_for_visualization = options.hold_for_visualization;
    }

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, output_dir),
        ERL_REFLECT_MEMBER(Options, scene_graph_file),
        ERL_REFLECT_MEMBER(Options, map_data_dir),
        ERL_REFLECT_MEMBER(Options, automaton_file),
        ERL_REFLECT_MEMBER(Options, ap_file),
        ERL_REFLECT_MEMBER(Options, llm_heuristic_file),
        ERL_REFLECT_MEMBER(Options, init_grid_x),
        ERL_REFLECT_MEMBER(Options, init_grid_y),
        ERL_REFLECT_MEMBER(Options, init_grid_z),
        ERL_REFLECT_MEMBER(Options, robot_radius),
        ERL_REFLECT_MEMBER(Options, object_reach_radius),
        ERL_REFLECT_MEMBER(Options, max_level),
        ERL_REFLECT_MEMBER(Options, ltl_heuristic_layout),
        ERL_REFLECT_MEMBER(Options, llm_heuristic_layout),
        ERL_REFLECT_MEMBER(Options, repeat),
        ERL_REFLECT_MEMBER(Options, save_amra_log),
        ERL_REFLECT_MEMBER(Options, hold_for_visualization));
};

inline static const char *g_hierarchy_layout = R"(Hierarchical Planning Domain Layout:
|  LEVEL  | Anchor | Occupancy | Object | Room | Floor |
| ENABLED |   {:c}    |     {:c}     |   {:c}    |  {:c}   |   {:c}   |
|   LTL   |   {:c}    |     {:c}     |   {:c}    |  {:c}   |   {:c}   |
|   LLM   |   {:c}    |     {:c}     |   {:c}    |  {:c}   |   {:c}   |)";

bool use_llm_heuristic;
char level_enabled_chars[5] = {'1', '0', '0', '0', '0'};
char level_ltl_chars[5] = {'1', 'X', 'X', 'X', 'X'};
char level_llm_chars[5] = {'X', 'X', 'X', 'X', 'X'};

template<typename Dtype>
void
run(const Options<Dtype> &options) {
    using namespace erl::common;
    using namespace erl::env;
    using namespace erl::path_planning;

    using Env = EnvironmentLTLSceneGraph<Dtype>;
    using MetricState = typename Env::MetricState;
    using LtlHeuristic = LinearTemporalLogicHeuristic3D<Dtype>;
    using LlmHeuristic = LlmSceneGraphHeuristic<Dtype>;
    using PlanningInterface = SearchPlanningInterfaceMultiResolutions<Dtype, 4>;
    using Heuristic = typename PlanningInterface::Heuristic;
    using AmraStar = amra_star::AmraStar<Dtype, 4>;

    // load scene graph
    auto scene_graph = std::make_shared<scene_graph::Building>();
    ERL_ASSERTM(
        scene_graph->FromYamlFile(options.scene_graph_file),
        "Failed to load scene graph from {}",
        options.scene_graph_file);
    // load the env setting
    auto env_setting = std::make_shared<typename Env::Setting>();
    env_setting->data_dir = options.map_data_dir;
    using AutFileType = FiniteStateAutomaton::Setting::FileType;
    env_setting->fsa = std::make_shared<FiniteStateAutomaton::Setting>(
        options.automaton_file,
        AutFileType::kSpotHoa,
        false);
    env_setting->LoadAtomicPropositions(options.ap_file);
    // create the environment
    env_setting->max_level = options.max_level;
    if (options.robot_radius > 0) {
        constexpr long n = 360;
        Eigen::VectorX<Dtype> angles =
            Eigen::VectorX<Dtype>::LinSpaced(n, 0, 2 * M_PI - 2 * M_PI / static_cast<Dtype>(n));
        env_setting->robot_metric_contour.resize(2, n);
        for (long i = 0; i < n; ++i) {
            env_setting->robot_metric_contour.col(i) << options.robot_radius * cos(angles[i]),
                options.robot_radius * sin(angles[i]);
        }
    }
    env_setting->object_reach_distance = options.object_reach_radius;
    auto env = std::make_shared<Env>(scene_graph, env_setting);
    // get initial states, goal set and goal tolerance.
    auto init_q = static_cast<int>(env_setting->fsa->initial_state);
    MetricState start = env->GridToMetric(
        Eigen::Vector4i(options.init_grid_x, options.init_grid_y, options.init_grid_z, init_q));
    auto num_goals = static_cast<long>(env_setting->fsa->accepting_states.size());
    std::vector<MetricState> goals(num_goals);
    for (long i = 0; i < num_goals; ++i) {
        MetricState &goal = goals[i];
        goal[0] = 0;  // does not matter
        goal[1] = 0;  // does not matter
        goal[2] = 0;  // does not matter
        goal[3] = static_cast<Dtype>(env_setting->fsa->accepting_states[i]);
    }
    constexpr Dtype inf = std::numeric_limits<Dtype>::infinity();
    MetricState goal_tolerance(inf, inf, inf, 0);
    // get ltl heuristic and llm heuristic
    ERL_INFO(
        g_hierarchy_layout,
        level_enabled_chars[0],
        level_enabled_chars[1],
        level_enabled_chars[2],
        level_enabled_chars[3],
        level_enabled_chars[4],
        level_ltl_chars[0],
        level_ltl_chars[1],
        level_ltl_chars[2],
        level_ltl_chars[3],
        level_ltl_chars[4],
        level_llm_chars[0],
        level_llm_chars[1],
        level_llm_chars[2],
        level_llm_chars[3],
        level_llm_chars[4]);
    auto ltl_heuristic = std::make_shared<LtlHeuristic>(
        env->GetFiniteStateAutomaton(),
        env->GetLabelMaps(),
        env->GetGridMapInfo());
    std::shared_ptr<LlmHeuristic> llm_heuristic = nullptr;
    if (use_llm_heuristic) {
        auto llm_heuristic_setting = std::make_shared<typename LlmHeuristic::Setting>();
        ERL_ASSERTM(
            llm_heuristic_setting->FromYamlFile(options.llm_heuristic_file),
            "Failed to load LLM heuristic setting from {}",
            options.llm_heuristic_file);
        llm_heuristic = std::make_shared<LlmHeuristic>(llm_heuristic_setting, env);
    }
    std::vector<std::pair<std::shared_ptr<Heuristic>, std::size_t>> heuristics;
    for (std::size_t i = 0; i < options.ltl_heuristic_layout.length(); ++i) {
        if (options.ltl_heuristic_layout[i] == '1') { heuristics.emplace_back(ltl_heuristic, i); }
    }
    for (std::size_t i = 0; i < options.llm_heuristic_layout.length(); ++i) {
        if (options.llm_heuristic_layout[i] == '1') { heuristics.emplace_back(llm_heuristic, i); }
    }
    // create the planner
    auto planning_interface = std::make_shared<PlanningInterface>(
        env,
        heuristics,
        start,
        goals,
        std::vector<MetricState>{goal_tolerance});
    auto amra_setting = std::make_shared<typename AmraStar::Setting>();
    amra_setting->log = false;
    std::shared_ptr<typename AmraStar::Output_t> result;
    for (int i = 0; i < options.repeat; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        AmraStar planner(planning_interface, amra_setting);
        auto t1 = std::chrono::high_resolution_clock::now();
        result = planner.Plan();
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "[" << i << "]Planner construction time: "
                  << std::chrono::duration<double, std::micro>(t1 - t0).count() << " us."
                  << std::endl;
        std::cout << "[" << i << "]Planning time: "
                  << std::chrono::duration<double, std::micro>(t2 - t1).count() << " us."
                  << std::endl;
    }
    // save the experiment setting
    std::filesystem::path output_dir = options.output_dir;
    options.AsYamlFile(output_dir / "experiment_setting.yaml");
    // save the result
    if (options.save_amra_log) {
        std::cout << "Running AMRA* again with logging enabled." << std::endl;
        amra_setting->log = true;
        AmraStar planner(planning_interface, amra_setting);
        result = planner.Plan();
        std::cout << "Saving AMRA* log..." << std::endl;
        result->Save(output_dir / "amra.solution");
    }
    // draw path
    for (auto &[plan_itr, plan_record]: result->plan_records) {
        long num_points = plan_record.path.cols();
        std::vector<cv::Point2i> cv_path;  // floor -> path
        int img_idx = 0;
        int cur_floor = -1;
        for (long i = 0; i < num_points; ++i) {
            Eigen::Vector4<Dtype> p = plan_record.path.col(i);
            Eigen::Vector4i grid = env->MetricToGrid(p);
            if (cur_floor < 0) { cur_floor = grid[2]; }
            if (cur_floor != grid[2]) {
                cv::Mat img;
                img = erl::common::ColorGrayCustom(
                    scene_graph->LoadCatMap(options.map_data_dir, cur_floor));
                cv::polylines(img, cv_path, false, cv::Scalar(0, 0, 0), 2);
                // draw start and goal
                cv::circle(img, cv_path.front(), 10, cv::Scalar(0, 0, 255), -1);
                cv::circle(img, cv_path.back(), 10, cv::Scalar(0, 255, 0), -1);
                std::string img_name =
                    fmt::format("plan_{}_img_{}_floor_{}", plan_itr, img_idx, cur_floor);
                if (options.hold_for_visualization) { cv::imshow(img_name, img); }
                cv::imwrite(output_dir / (img_name + ".png"), img);
                img_idx++;
                cur_floor = grid[2];
                cv_path.clear();
            }
            cv_path.emplace_back(grid[1], grid[0]);
        }
        cv::Mat img;
        img =
            erl::common::ColorGrayCustom(scene_graph->LoadCatMap(options.map_data_dir, cur_floor));
        cv::polylines(img, cv_path, false, cv::Scalar(0, 0, 0), 2);
        // draw start and goal
        cv::circle(img, cv_path.front(), 10, cv::Scalar(0, 0, 255), -1);
        cv::circle(img, cv_path.back(), 10, cv::Scalar(0, 255, 0), -1);
        std::string img_name = fmt::format("plan_{}_img_{}_floor_{}", plan_itr, img_idx, cur_floor);
        if (options.hold_for_visualization) { cv::imshow(img_name, img); }
        cv::imwrite(output_dir / (img_name + ".png"), img);
    }
    // hold for visualization
    if (options.hold_for_visualization) { cv::waitKey(0); }
}

int
main(int argc, char *argv[]) {

    std::string max_level_str;
    Options<double> tmp;
    Options<double> loaded;

    bool is_double = false;

    try {
        namespace po = boost::program_options;
        po::options_description desc;
        po::positional_options_description positional_options;
        // clang-format off
        desc.add_options()
            ("help", "produce help message")
            ("precision", po::value<std::string>()->default_value("float"), "precision of the computation: float or double.")
            ("config", po::value<std::string>(), "path to the config file.")
            ("output_dir", po::value<std::string>(&tmp.output_dir), "path to save the results.")
            ("scene_graph_file", po::value<std::string>(&tmp.scene_graph_file), "path to the YAML of scene graph.")
            ("map_data_dir", po::value<std::string>(&tmp.map_data_dir), "path to the map data directory.")
            ("automaton_file", po::value<std::string>(&tmp.automaton_file), "path to the automaton file.")
            ("ap_file", po::value<std::string>(&tmp.ap_file), "path to the AP description file.")
            ("llm_heuristic_file", po::value<std::string>(&tmp.llm_heuristic_file), "path to the LLM heuristic file.")
            ("init_grid_x", po::value<int>(&tmp.init_grid_x), "initial x position in grids.")
            ("init_grid_y", po::value<int>(&tmp.init_grid_y), "initial y position in grids.")
            ("init_grid_z", po::value<int>(&tmp.init_grid_z), "initial z position in grids.")
            ("robot_radius", po::value<double>(&tmp.robot_radius)->default_value(tmp.robot_radius), "robot radius.")
            ("object_reach_radius", po::value<double>(&tmp.object_reach_radius)->default_value(tmp.object_reach_radius), "object reach radius.")
            ("max_level", po::value<std::string>(&max_level_str), "maximum level to plan: kOcc, kObject, kRoom, kFloor, anchor cannot be disabled.")
            ("ltl_heuristic_config", po::value<std::string>(&tmp.ltl_heuristic_layout), "a sequence of 0,1 to indicate whether to use LTL heuristic for each level up to the max_level: anchor, kOcc, kObject, kRoom, kFloor.")
            ("llm_heuristic_config", po::value<std::string>(&tmp.llm_heuristic_layout), "a sequence of 0,1 to indicate whether to use LLM heuristic for each level up to the max_level: anchor, kOcc, kObject, kRoom, kFloor.")
            ("repeat", po::value<int>(&tmp.repeat)->default_value(tmp.repeat), "repeat the experiment for multiple times.")
            ("save_amra_log", po::bool_switch(&tmp.save_amra_log)->default_value(tmp.save_amra_log), "save AMRA* log")
            ("hold_for_visualization", po::bool_switch(&tmp.hold_for_visualization)->default_value(tmp.hold_for_visualization), "pause for visualization");
        // clang-format on

        po::variables_map vm;
        po::store(
            po::command_line_parser(argc, argv).options(desc).positional(positional_options).run(),
            vm);

        if (vm.count("help")) {
            std::cout << "Usage: " << argv[0] << " [options] amra_ltl_scene_graph" << std::endl
                      << desc << std::endl;
            return 0;
        }
        po::notify(vm);

        if (vm["precision"].as<std::string>() == "float") {
            is_double = false;
        } else if (vm["precision"].as<std::string>() == "double") {
            is_double = true;
        } else {
            std::cerr << "precision is not valid." << std::endl;
            return 1;
        }

        if (vm.count("config")) {
            std::string config_file = vm["config"].as<std::string>();
            std::cout << "Loading config from " << config_file << "..." << std::endl;
            ERL_ASSERTM(
                loaded.FromYamlFile(config_file),
                "Failed to load config from {}",
                config_file);
        }

#define LOAD_IF_SET(ATTR) \
    if (vm.count(#ATTR)) loaded.ATTR = tmp.ATTR

        LOAD_IF_SET(output_dir);
        LOAD_IF_SET(scene_graph_file);
        LOAD_IF_SET(map_data_dir);
        LOAD_IF_SET(automaton_file);
        LOAD_IF_SET(ap_file);
        LOAD_IF_SET(llm_heuristic_file);
        LOAD_IF_SET(init_grid_x);
        LOAD_IF_SET(init_grid_y);
        LOAD_IF_SET(init_grid_z);
        LOAD_IF_SET(robot_radius);
        LOAD_IF_SET(object_reach_radius);
        LOAD_IF_SET(ltl_heuristic_layout);
        LOAD_IF_SET(llm_heuristic_layout);
        LOAD_IF_SET(repeat);
        LOAD_IF_SET(save_amra_log);
        LOAD_IF_SET(hold_for_visualization);

#undef LOAD_IF_SET

        if (loaded.output_dir.empty()) {
            std::cerr << "output_dir is not provided." << std::endl;
            return 1;
        }
        if (loaded.scene_graph_file.empty()) {
            std::cerr << "scene_graph_file is not provided." << std::endl;
            return 1;
        }
        if (loaded.map_data_dir.empty()) {
            std::cerr << "map_data_dir is not provided." << std::endl;
            return 1;
        }
        if (loaded.automaton_file.empty()) {
            std::cerr << "automaton_file is not provided." << std::endl;
            return 1;
        }
        if (loaded.ap_file.empty()) {
            std::cerr << "ap_file is not provided." << std::endl;
            return 1;
        }
        if (loaded.init_grid_x < 0) {
            std::cerr << "init_grid_x is not provided." << std::endl;
            return 1;
        }
        if (loaded.init_grid_y < 0) {
            std::cerr << "init_grid_y is not provided." << std::endl;
            return 1;
        }
        if (loaded.init_grid_z < 0) {
            std::cerr << "init_grid_z is not provided." << std::endl;
            return 1;
        }
        if (loaded.object_reach_radius < loaded.robot_radius) {
            std::cerr << "object_reach_radius should be at least robot_radius." << std::endl;
            return 1;
        }
        if (max_level_str.empty()) {
            std::cerr << "max_level is not provided." << std::endl;
            return 1;
        }
        if (max_level_str == "kOcc") {
            loaded.max_level = erl::env::scene_graph::NodeType::kOcc;
        } else if (max_level_str == "kFloor") {
            loaded.max_level = erl::env::scene_graph::NodeType::kFloor;
        } else if (max_level_str == "kRoom") {
            loaded.max_level = erl::env::scene_graph::NodeType::kRoom;
        } else if (max_level_str == "kObject") {
            loaded.max_level = erl::env::scene_graph::NodeType::kObject;
        } else {
            std::cerr << "max_level is not valid." << std::endl;
            return 1;
        }
        for (int i = 0; i <= static_cast<int>(loaded.max_level); ++i) {
            level_enabled_chars[i + 1] = '1';
        }
        if (loaded.ltl_heuristic_layout.empty()) {
            std::cerr << "ltl_heuristic_layout is not provided." << std::endl;
            return 1;
        }
        if (loaded.ltl_heuristic_layout.length() !=
            static_cast<std::size_t>(loaded.max_level) + 2) {
            std::cerr << "ltl_heuristic_layout is not valid with max_level = " << max_level_str
                      << std::endl;
            return 1;
        }
        for (char c: loaded.ltl_heuristic_layout) {
            if (c != '0' && c != '1') {
                std::cerr << "ltl_heuristic_layout should only contain 0 or 1." << std::endl;
                return 1;
            }
        }
        if (loaded.ltl_heuristic_layout[0] != '1') {
            std::cerr << "ltl_heuristic_layout should start with 1 to enable LTL heuristic for the "
                         "anchor level."
                      << std::endl;
            return 1;
        }
        for (std::size_t i = 0; i < loaded.ltl_heuristic_layout.length(); ++i) {
            if (loaded.ltl_heuristic_layout[i] == '1') {
                level_ltl_chars[i] = '1';
            } else {
                level_ltl_chars[i] = '0';
            }
        }
        if (loaded.llm_heuristic_layout.empty()) {
            std::cerr << "llm_heuristic_config is not provided." << std::endl;
            return 1;
        }
        if (loaded.llm_heuristic_layout.length() !=
            static_cast<std::size_t>(loaded.max_level) + 2) {
            std::cerr << "llm_heuristic_config is not valid with max_level = " << max_level_str
                      << std::endl;
            return 1;
        }
        for (char c: loaded.llm_heuristic_layout) {
            if (c != '0' && c != '1') {
                std::cerr << "llm_heuristic_config should only contain 0 or 1." << std::endl;
                return 1;
            }
        }
        if (loaded.llm_heuristic_layout[0] != '0') {
            std::cerr << "llm_heuristic_layout should start with 0 because anchor level does not "
                         "allow LLM heuristic."
                      << std::endl;
            return 1;
        }
        for (std::size_t i = 0; i < loaded.llm_heuristic_layout.length(); ++i) {
            if (loaded.llm_heuristic_layout[i] == '1') {
                level_llm_chars[i] = '1';
            } else {
                level_llm_chars[i] = '0';
            }
        }
        use_llm_heuristic = std::any_of(
            loaded.ltl_heuristic_layout.begin(),
            loaded.ltl_heuristic_layout.end(),
            [](const char c) { return c == '1'; });
        if (use_llm_heuristic && loaded.llm_heuristic_file.empty()) {
            std::cerr << "llm_heuristic_file is not provided." << std::endl;
            return 1;
        }
        if (loaded.repeat < 1) {
            std::cerr << "repeat should be at least 1." << std::endl;
            return 1;
        }
    } catch (std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    if (loaded.map_data_dir.empty()) {
        loaded.map_data_dir = std::filesystem::path(loaded.scene_graph_file).parent_path().string();
        ERL_INFO("map_data_dir is not provided, using the default value: {}", loaded.map_data_dir);
    }

    // create output dir
    if (!std::filesystem::exists(loaded.output_dir)) {
        std::filesystem::create_directories(loaded.output_dir);
    }

    if (is_double) {
        run<double>(loaded);
    } else {
        Options<float> loaded_float;
        loaded_float.Load(loaded);
        run<float>(loaded_float);
    }
}
