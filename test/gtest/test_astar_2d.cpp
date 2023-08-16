#include <gtest/gtest.h>

#include "erl_env/cost.hpp"
#include "erl_search_planning/astar.hpp"
#include "erl_env/environment_2d.hpp"
#include "erl_env/environment_ltl_2d.hpp"
#include "erl_common/test_helper.hpp"

TEST(AStar2DTest, PlanWithSingleGoal) {

    using namespace erl::common;
    using namespace erl::search_planning;
    using namespace erl::env;

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
    auto grid_map_info = std::make_shared<GridMapInfo<2>>(Eigen::Vector2i{15, 15}, Eigen::Vector2d::Zero(), Eigen::Vector2d::Constant(15));
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, grid_map_data);
    Eigen::Vector2d metric_goal = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 10});
    Eigen::Vector2d metric_goal_tolerance = Eigen::Vector2d::Zero();
    Eigen::Scalard terminal_cost(0);
    Eigen::Vector2d metric_start_coords = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 1});

    std::shared_ptr<CostBase> cost_func = std::make_shared<EuclideanDistanceCost>();
    auto setting = std::make_shared<Environment2D::Setting>();
    setting->allow_diagonal = true;
    setting->step_size = 1;
    auto env = std::make_shared<Environment2D>(grid_map, setting, cost_func);
    auto planning_interface = std::make_shared<PlanningInterface>(env, metric_start_coords, metric_goal, metric_goal_tolerance);
    std::shared_ptr<astar::Output> result;
    ReportTime<std::chrono::microseconds>("AStar2DTest::PlanWithSingleGoal", 0, true, [&]() { result = astar::AStar(planning_interface).Plan(); });
    std::cout << "Path cost: " << result->cost << std::endl << "Path: " << std::endl;
    auto &path = result->path;
    long num_points = path.cols();
    for (long i = 0; i < num_points; ++i) { std::cout << path.col(i).transpose() << std::endl; }

    EXPECT_NEAR(result->cost, 15.485281374238571, 1e-6);
}

TEST(AStar2DTest, PlanWithFourGoals) {

    using namespace erl::common;
    using namespace erl::search_planning;
    using namespace erl::env;

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
    auto grid_map_info = std::make_shared<GridMapInfo<2>>(Eigen::Vector2i{15, 15}, Eigen::Vector2d::Zero(), Eigen::Vector2d::Constant(15));
    auto grid_map = std::make_shared<GridMap<uint8_t, 2>>(grid_map_info, grid_map_data);
    Eigen::Matrix2Xi grid_goals(2, 4);
    // clang-format off
    grid_goals <<  1, 14, 7, 9,
                  10, 14, 1, 2;
    // clang-format on
    std::vector<Eigen::VectorXd> metric_goals;
    std::vector<Eigen::VectorXd> metric_goals_tolerance = {Eigen::VectorXd::Zero(2)};
    for (long i = 0; i < grid_goals.cols(); ++i) { metric_goals.emplace_back(grid_map_info->GridToMeterForPoints(grid_goals.col(i))); }
    std::vector<double> terminal_cost{1000.0, 1000.0, 0.0, 1100.0};

    auto setting = std::make_shared<Environment2D::Setting>();
    setting->allow_diagonal = true;
    setting->step_size = 1;
    std::shared_ptr<CostBase> cost_func = std::make_shared<EuclideanDistanceCost>();
    auto env = std::make_shared<Environment2D>(grid_map, setting, cost_func);
    Eigen::Vector2d metric_start_coords = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 1});
    auto planning_interface = std::make_shared<PlanningInterface>(env, metric_start_coords, metric_goals, metric_goals_tolerance, terminal_cost);
    std::shared_ptr<astar::Output> result;
    ReportTime<std::chrono::microseconds>("AStar2DTest::PlanWithFourGoals", 0, true, [&]() { result = astar::AStar(planning_interface).Plan(); });
    std::cout << "Path to goal " << result->goal_index << " cost: " << result->cost << std::endl;

    cv::Mat img = planning_interface->GetEnvironment()->ShowPaths({{result->goal_index, result->path}});
    cv::imwrite("AStar2DTest-PlanWithFourGoals.png", img);

    EXPECT_NEAR(result->cost, 1015.485281374, 1e-6);
    // EXPECT_NEAR(result->path_costs[1], 1021.899494937, 1e-6);
    // EXPECT_NEAR(result->path_costs[3], 1110.65685425, 1e-6);

    long num_points = result->path.cols();
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (long i = 0; i < num_points; ++i) {
        out << YAML::Flow << Eigen::Vector3d(result->path.col(i));
    }
    out << YAML::EndSeq;
    std::ofstream ofs("AStar2DTest-LinearTemporalLogic2D.yaml");
    ofs << out.c_str();
}

TEST(AStar2DTest, LinearTemporalLogic2D) {
    using namespace erl::common;
    using namespace erl::search_planning;
    using namespace erl::env;

    std::filesystem::path path = __FILE__;
    path = path.parent_path();

    auto env_setting_yaml = path / "environment_ltl_2d.yaml";
    auto env_setting = std::make_shared<EnvironmentLTL2D::Setting>();
    env_setting->FromYamlFile(env_setting_yaml);

    auto label_map_png = path / "label_map.png";
    cv::Mat label_map_img = cv::imread(label_map_png.string(), cv::IMREAD_GRAYSCALE);
    Eigen::MatrixX8U label_map_img_eigen;
    cv::cv2eigen(label_map_img, label_map_img_eigen);
    Eigen::MatrixXi label_map = label_map_img_eigen.cast<int>();

    Eigen::Vector2i map_shape(251, 261);
    Eigen::Vector2d map_min(-5.05, -5.05);
    Eigen::Vector2d map_max(20.05, 21.05);
    auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, 0);  // free to move everywhere
    auto cost_func = std::make_shared<EuclideanDistanceCost>();
    auto env = std::make_shared<EnvironmentLTL2D>(label_map, grid_map, env_setting, cost_func);
    Eigen::VectorXd metric_start_coords(Eigen::Vector3d(-2, 3, env_setting->fsa->initial_state));
    std::vector<Eigen::VectorXd> metric_goals_coords(env_setting->fsa->accepting_states.size());
    for (std::size_t i = 0; i < metric_goals_coords.size(); ++i) {
        Eigen::VectorXd &goal_coords = metric_goals_coords[i];
        goal_coords.resize(3);
        goal_coords[0] = 0;
        goal_coords[1] = 0;
        goal_coords[2] = env_setting->fsa->accepting_states[i];
    }
    double inf = std::numeric_limits<double>::infinity();
    std::vector<Eigen::VectorXd> metric_goal_tolerance;  // only care about LTL state
    metric_goal_tolerance.emplace_back(Eigen::Vector3d(inf, inf, 0));
    auto heuristic = std::make_shared<LinearTemporalLogicHeuristic2D>(env->GetFiniteStateAutomaton(), label_map, grid_map_info);
    auto planning_interface =
        std::make_shared<PlanningInterface>(env, metric_start_coords, metric_goals_coords, metric_goal_tolerance, std::vector{0.}, heuristic);
    std::shared_ptr<astar::Output> result;
    astar::AStar astar(planning_interface);
    ReportTime<std::chrono::microseconds>("AStar2DTest::LinearTemporalLogic2D", 0, true, [&]() { result = astar.Plan(); });

    std::cout << "Path to goal " << result->goal_index << " cost: " << result->cost << std::endl;

    cv::Mat img = planning_interface->GetEnvironment()->ShowPaths({{result->goal_index, result->path}});
    cv::imwrite("AStar2DTest-LinearTemporalLogic2D.png", img);

    EXPECT_NEAR(result->cost, 20.417871555019111, 1e-6);
    EXPECT_EQ(result->path.cols(), 165);

    long num_points = result->path.cols();
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (long i = 0; i < num_points; ++i) {
        out << YAML::Flow << Eigen::Vector3d(result->path.col(i));
    }
    out << YAML::EndSeq;
    std::ofstream ofs("AStar2DTest-LinearTemporalLogic2D.yaml");
    ofs << out.c_str();
}
