#include "erl_common/grid_map.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_env/environment_2d.hpp"
#include "erl_env/environment_grid_anchor.hpp"
#include "erl_env/environment_ltl_2d.hpp"
#include "erl_path_planning/amra_star.hpp"
#include "erl_path_planning/heuristic.hpp"
#include "erl_path_planning/ltl_2d_heuristic.hpp"
#include "erl_path_planning/search_planning_interface.hpp"

TEST(AMRAStar2D, AStarConsistency) {
    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;
    using GridMap = erl::common::GridMap<uint8_t, Dtype, 2>;
    using CostBase = erl::env::CostBase<Dtype, 2>;
    using EuclideanDistanceCost = erl::env::EuclideanDistanceCost<Dtype, 2>;
    using EnvState = erl::env::EnvironmentState<Dtype, 2>;
    using MetricState = EnvState::MetricState;
    using EnvironmentBase = erl::env::EnvironmentBase<Dtype, 2>;
    using Environment2D = erl::env::Environment2D<Dtype, uint8_t>;
    using EnvironmentGridAnchor = erl::env::EnvironmentGridAnchor<Dtype, 2>;
    using HeuristicBase = erl::path_planning::HeuristicBase<Dtype, 2>;
    using EuclideanDistanceHeuristic = erl::path_planning::EuclideanDistanceHeuristic<Dtype, 2>;
    using PlanningInterfaceMultiResolutions =
        erl::path_planning::SearchPlanningInterfaceMultiResolutions<Dtype, 2>;
    using AmraStar = erl::path_planning::amra_star::AmraStar<Dtype, 2>;
    using Output = erl::path_planning::amra_star::Output<Dtype, 2>;

    Eigen::Vector<uint8_t, 15 * 15> grid_map_data;
    // clang-format off
    grid_map_data <<
        0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    // clang-format on
    auto grid_map_info = std::make_shared<GridMapInfo>(
        Eigen::Vector2i{15, 15},
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d::Constant(15));
    auto grid_map = std::make_shared<GridMap>(grid_map_info, grid_map_data);

    auto env_2d_setting = std::make_shared<Environment2D::Setting>();
    env_2d_setting->SetGridMotionPrimitive(1, true);
    std::shared_ptr<CostBase> cost_func = std::make_shared<EuclideanDistanceCost>();
    auto env = std::make_shared<Environment2D>(grid_map, env_2d_setting, cost_func);
    MetricState start = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 1});
    MetricState goal = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 10});
    MetricState goal_tolerance = Eigen::Vector2d::Zero();

    auto anchor_env = std::make_shared<EnvironmentGridAnchor>(
        std::vector<std::shared_ptr<EnvironmentBase>>{env},
        grid_map_info);
    std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics = {
        {std::make_shared<EuclideanDistanceHeuristic>(goal, goal_tolerance, 0), 0},
        {std::make_shared<EuclideanDistanceHeuristic>(goal, goal_tolerance, 0), 1}};

    constexpr double terminal_cost = 10.0;
    auto planning_interface = std::make_shared<PlanningInterfaceMultiResolutions>(
        // environments,
        anchor_env,
        heuristics,
        start,
        std::vector<MetricState>{goal},
        std::vector<MetricState>{goal_tolerance},
        std::vector<double>{terminal_cost});
    auto amra_setting = std::make_shared<AmraStar::Setting>();
    amra_setting->log = true;
    AmraStar planner(planning_interface, amra_setting);
    std::shared_ptr<Output> result;
    using namespace erl::common;
    ReportTime<std::chrono::microseconds>("AMRAStar2D::AStarConsistency", 0, true, [&]() {
        result = planner.Plan();
    });
    auto &plan_record = result->plan_records[result->latest_plan_itr];
    double path_cost = plan_record.cost;
    std::cout << "Path cost: " << path_cost << std::endl << "Path: " << std::endl;
    auto &path = plan_record.path;
    long num_points = path.cols();
    for (long i = 0; i < num_points; ++i) { std::cout << path.col(i).transpose() << std::endl; }
    EXPECT_DOUBLE_EQ(path_cost - terminal_cost, 16.071067811865476);
    if (amra_setting->log) { result->Save(test_output_dir / "amra.solution"); }
}

std::shared_ptr<erl::common::GridMap<uint8_t, double, 2>>
ReadAmraStarTestMap(
    const std::string &filepath,
    bool display = false,
    const std::string &img_path = "") {
    std::ifstream file(filepath);
    std::string line, word;
    std::stringstream ss;

    auto reset = [](std::stringstream &ss) {
        ss.str("");
        ss.clear();
    };

    std::getline(file, line);
    EXPECT_EQ(line, "type octile");

    // read height/width
    int height = 0, width = 0;
    for (int i = 0; i < 2; ++i) {
        std::getline(file, line);
        reset(ss);
        ss.str(line);
        std::getline(ss, word, ' ');
        if (word == "height") {
            std::getline(ss, word, ' ');
            height = std::stoi(word);
        } else if (word == "width") {
            std::getline(ss, word, ' ');
            width = std::stoi(word);
        } else {
            throw std::runtime_error("Invalid line: " + line);
        }
    }

    std::getline(file, line);
    EXPECT_EQ(line, "map");

    const std::map<char, int> moving_ai_dict = {
        {'.', 1},     // free
        {'G', 1},     // free
        {'@', -1},    // obstacle
        {'O', -1},    // obstacle
        {'T', 0},     // obstacle
        {'S', 1},     // swamp (passable from regular terrain)
        {'W', 2},     // water (traversable, but not passable from terrain)
        {'(', 1000},  // start
        {'*', 1001},  // path
        {')', 1002},  // goal
        {'E', 1003},  // expanded state
    };

    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> map(height, width);
    map.setZero();
    for (int r = 0; r < height; ++r) {
        std::getline(file, line);
        for (int c = 0; c < width; ++c) {
            auto itr = moving_ai_dict.find(line[c]);

            int &map_val = map(r, c);

            if (itr != moving_ai_dict.end()) {
                map_val = moving_ai_dict.find(line[c])->second;
            } else {
                int val = static_cast<int>(line[c]) - static_cast<int>('a') + 1;
                map_val = val * 10;
            }

            if (map_val < 0) {
                map_val = 255;  // obstacle
            } else if (map_val == 0) {
                map_val = 128;  // obstacle
            } else {
                map_val = 0;  // free
            }
        }
    }

    Eigen::Vector2i map_shape(height, width);
    Eigen::Vector2d map_min(0.0, 0.0);
    Eigen::Vector2d map_max(height, width);
    auto grid_map_info = std::make_shared<erl::common::GridMapInfo2Dd>(
        std::move(map_shape),
        std::move(map_min),
        std::move(map_max));
    Eigen::MatrixX8U binary_map(grid_map_info->Height(), grid_map_info->Width());
    binary_map.setZero();
    binary_map.topLeftCorner(height, width) =
        map.cast<uint8_t>()
            .transpose();  // erl::common::GridMapUnsigned2D requires row-major storage

    if (display) {
        cv::Mat img;
        cv::eigen2cv(Eigen::MatrixX8U(255 - binary_map.array().transpose()), img);
        cv::imshow("map", img);
        cv::waitKey(1000);
    }

    if (!img_path.empty()) {
        cv::Mat img;
        cv::eigen2cv(Eigen::MatrixX8U(255 - binary_map.array().transpose()), img);
        cv::imwrite(img_path, img);
    }

    auto grid_map = std::make_shared<erl::common::GridMap<uint8_t, double, 2>>(
        grid_map_info,
        Eigen::VectorX8U(binary_map.reshaped(binary_map.size(), 1)));
    return grid_map;
}

void
RunTestWithMap(
    const std::filesystem::path &map_file,
    const Eigen::Vector2i &start_grid,
    const Eigen::Vector2i &goal_grid,
    double expected_cost) {

    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using EnvState = erl::env::EnvironmentState<Dtype, 2>;
    using MetricState = EnvState::MetricState;
    using EnvironmentBase = erl::env::EnvironmentBase<Dtype, 2>;
    using Environment2D = erl::env::Environment2D<Dtype, uint8_t>;
    using EnvironmentGridAnchor = erl::env::EnvironmentGridAnchor<Dtype, 2>;
    using EuclideanDistanceCost = erl::env::EuclideanDistanceCost<Dtype, 2>;
    using HeuristicBase = erl::path_planning::HeuristicBase<Dtype, 2>;
    using EuclideanDistanceHeuristic = erl::path_planning::EuclideanDistanceHeuristic<Dtype, 2>;
    using PlanningInterface = erl::path_planning::SearchPlanningInterfaceMultiResolutions<Dtype, 2>;
    using AmraStar = erl::path_planning::amra_star::AmraStar<Dtype, 2>;
    using Output = erl::path_planning::amra_star::Output<Dtype, 2>;

    std::filesystem::path data_dir = std::filesystem::absolute(gtest_src_dir / "../amra/dat");

    std::string map_name = std::filesystem::relative(map_file, data_dir).string();
    std::string sep(map_name.size() + 2, '=');
    std::cout << sep << std::endl << ' ' << map_name << ' ' << std::endl << sep << std::endl;
    using namespace erl::common;
    using namespace erl::env;
    using namespace erl::path_planning;
    using namespace erl::path_planning::amra_star;

    auto map_result_dir = test_output_dir / map_name;
    if (!std::filesystem::exists(map_result_dir)) {
        std::filesystem::create_directories(map_result_dir);
    }
    constexpr bool display = false;
    std::string img_path = map_result_dir / "amra.png";
    auto grid_map = ReadAmraStarTestMap(map_file, display, img_path);
    auto grid_map_info = grid_map->info;

    auto cost_func = std::make_shared<EuclideanDistanceCost>();

    auto env_high_res_setting = std::make_shared<Environment2D::Setting>();
    env_high_res_setting->SetGridMotionPrimitive(1, true);
    env_high_res_setting->grid_stride = 1;
    auto env_high_res = std::make_shared<Environment2D>(grid_map, env_high_res_setting, cost_func);
    auto env_mid_res_setting = std::make_shared<Environment2D::Setting>();
    env_mid_res_setting
        ->motions = {{3, 0}, {-3, 0}, {0, 3}, {0, -3}, {3, 3}, {3, -3}, {-3, 3}, {-3, -3}};
    env_mid_res_setting->grid_stride = 3;
    auto env_mid_res = std::make_shared<Environment2D>(grid_map, env_mid_res_setting, cost_func);
    auto env_low_res_setting = std::make_shared<Environment2D::Setting>();
    env_low_res_setting
        ->motions = {{9, 0}, {-9, 0}, {0, 9}, {0, -9}, {9, 9}, {9, -9}, {-9, 9}, {-9, -9}};
    env_low_res_setting->grid_stride = 9;
    auto env_low_res = std::make_shared<Environment2D>(grid_map, env_low_res_setting, cost_func);
    std::vector<std::shared_ptr<EnvironmentBase>> envs = {env_high_res, env_mid_res, env_low_res};
    auto env_anchor = std::make_shared<EnvironmentGridAnchor>(envs, grid_map_info);

    MetricState start = grid_map_info->GridToMeterForPoints(start_grid);
    MetricState goal = grid_map_info->GridToMeterForPoints(goal_grid);
    MetricState goal_tolerance = MetricState::Zero();
    auto euclidean_heuristic =
        std::make_shared<EuclideanDistanceHeuristic>(goal, goal_tolerance, 0);
    std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics = {
        {euclidean_heuristic, 0},
        {euclidean_heuristic, 1},
        {euclidean_heuristic, 2},
        {euclidean_heuristic, 3}};

    auto planning_interface =
        std::make_shared<PlanningInterface>(env_anchor, heuristics, start, goal, goal_tolerance);
    auto setting = std::make_shared<AmraStar::Setting>();
    setting->log = true;
    std::shared_ptr<Output> result;
    AmraStar amra_star(planning_interface, setting);
    ReportTime<std::chrono::milliseconds>(map_name.c_str(), 0, true, [&]() {
        result = amra_star.Plan();
    });
    auto plan_record = result->plan_records[result->latest_plan_itr];
    std::cout << "Path cost: " << plan_record.cost << std::endl;
    EXPECT_NEAR(plan_record.cost, expected_cost, 1.e-6);

    if (setting->log) { result->Save(map_result_dir / "amra.solution"); }
}

TEST(AMRAStar2D, MultiResolutions) {
    GTEST_PREPARE_OUTPUT_DIR();
    const std::filesystem::path data_dir = gtest_src_dir.parent_path() / "amra/dat";

    RunTestWithMap(data_dir / "Boston_0_1024.map", {100, 100}, {1008, 756}, 1229.7363859129694);
    RunTestWithMap(data_dir / "Cauldron.map", {100, 800}, {950, 400}, 1076.9352455636508);
    RunTestWithMap(data_dir / "Denver_0_1024.map", {306, 171}, {1008, 603}, 916.77289497627919);
    RunTestWithMap(data_dir / "Expedition.map", {720, 423}, {198, 891}, 766.65246964938046);
    RunTestWithMap(data_dir / "NewYork_0_1024.map", {0, 171}, {612, 882}, 1000.4262669664867);
    RunTestWithMap(data_dir / "Octopus.map", {549, 837}, {639, 279}, 644.11319062351083);
    RunTestWithMap(data_dir / "TheFrozenSea.map", {288, 414}, {207, 990}, 649.50994061340202);
}

TEST(AMRAStar2D, LinearTemporalLogic) {
    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using EnvironmentBase = erl::env::EnvironmentBase<Dtype, 3>;
    using EnvironmentGridAnchor = erl::env::EnvironmentGridAnchor<Dtype, 3>;
    using EnvironmentLTL2D = erl::env::EnvironmentLTL2D<Dtype, uint8_t>;
    using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;
    using GridMap = erl::common::GridMap<uint8_t, Dtype, 2>;
    using EuclideanDistanceCost = erl::env::EuclideanDistanceCost<Dtype, 2>;
    using HeuristicBase = erl::path_planning::HeuristicBase<Dtype, 3>;
    using LtlHeuristic2D = erl::path_planning::LinearTemporalLogicHeuristic2D<Dtype>;
    using PlanningInterface = erl::path_planning::SearchPlanningInterfaceMultiResolutions<Dtype, 3>;
    using AmraStar = erl::path_planning::amra_star::AmraStar<Dtype, 3>;
    using Output = erl::path_planning::amra_star::Output<Dtype, 3>;

    auto output_dir = gtest_src_dir / "results" / test_output_dir;
    if (!std::filesystem::exists(output_dir)) { std::filesystem::create_directories(output_dir); }

    auto env_setting_yaml = gtest_src_dir / "environment_ltl_2d.yaml";
    auto env_setting = std::make_shared<EnvironmentLTL2D::Setting>();
    ASSERT_TRUE(env_setting->FromYamlFile(env_setting_yaml));

    auto label_map_png = gtest_src_dir / "label_map.png";
    cv::Mat label_map_img = cv::imread(label_map_png.string(), cv::IMREAD_GRAYSCALE);
    Eigen::MatrixX8U label_map_img_eigen;
    cv::cv2eigen(label_map_img, label_map_img_eigen);
    Eigen::MatrixX<uint32_t> label_map = label_map_img_eigen.cast<uint32_t>();

    Eigen::Vector2i map_shape(251, 261);
    Eigen::Vector2d map_min(-5.05, -5.05);
    Eigen::Vector2d map_max(20.05, 21.05);
    auto grid_map_info = std::make_shared<GridMapInfo>(map_shape, map_min, map_max);
    auto grid_map = std::make_shared<GridMap>(grid_map_info, 0);  // free to move everywhere
    auto cost_func = std::make_shared<EuclideanDistanceCost>();

    auto env_high_res_setting = std::make_shared<EnvironmentLTL2D::Setting>(*env_setting);
    env_high_res_setting->SetGridMotionPrimitive(1, true);
    env_high_res_setting->grid_stride = 1;
    auto env_high_res =
        std::make_shared<EnvironmentLTL2D>(label_map, grid_map, env_high_res_setting, cost_func);
    auto env_mid_res_setting = std::make_shared<EnvironmentLTL2D::Setting>(*env_setting);
    env_mid_res_setting
        ->motions = {{3, 0}, {-3, 0}, {0, 3}, {0, -3}, {3, 3}, {3, -3}, {-3, 3}, {-3, -3}};
    env_mid_res_setting->grid_stride = 3;
    auto env_mid_res =
        std::make_shared<EnvironmentLTL2D>(label_map, grid_map, env_mid_res_setting, cost_func);
    auto env_low_res_setting = std::make_shared<EnvironmentLTL2D::Setting>(*env_setting);
    env_low_res_setting
        ->motions = {{9, 0}, {-9, 0}, {0, 9}, {0, -9}, {9, 9}, {9, -9}, {-9, 9}, {-9, -9}};
    env_low_res_setting->grid_stride = 9;
    auto env_low_res =
        std::make_shared<EnvironmentLTL2D>(label_map, grid_map, env_low_res_setting, cost_func);
    std::vector<std::shared_ptr<EnvironmentBase>> envs = {env_high_res, env_mid_res, env_low_res};
    auto env_anchor = std::make_shared<EnvironmentGridAnchor>(envs, env_high_res->GetGridMapInfo());

    Eigen::VectorXd start(Eigen::Vector3d(-2, 3, env_setting->fsa->initial_state));
    Eigen::VectorXd goal(Eigen::Vector3d(0, 0, env_setting->fsa->accepting_states[0]));
    double inf = std::numeric_limits<double>::infinity();
    Eigen::VectorXd goal_tolerance(Eigen::Vector3d(inf, inf, 0));

    auto ltl_heuristic = std::make_shared<LtlHeuristic2D>(
        env_high_res->GetFiniteStateAutomaton(),
        label_map,
        grid_map_info);
    std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics = {
        {ltl_heuristic, 0},
        {ltl_heuristic, 1},
        {ltl_heuristic, 2},
        {ltl_heuristic, 3}};

    auto planning_interface =
        std::make_shared<PlanningInterface>(env_anchor, heuristics, start, goal, goal_tolerance);
    auto setting = std::make_shared<AmraStar::Setting>();
    setting->log = true;
    std::shared_ptr<Output> result;
    AmraStar amra_star(planning_interface, setting);
    erl::common::ReportTime<std::chrono::milliseconds>("AMRA* 2D LTL", 0, true, [&]() {
        result = amra_star.Plan();
    });
    auto &plan_record = result->plan_records[result->latest_plan_itr];
    std::cout << "Path cost: " << plan_record.cost << std::endl;

    EXPECT_DOUBLE_EQ(
        plan_record.cost,
        20.417871555019033);  // 20.417871555019136, slightly lower than the A* result
    if (setting->log) { result->Save(output_dir / "amra.solution"); }
}
