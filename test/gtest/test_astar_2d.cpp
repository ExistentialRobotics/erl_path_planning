#include <gtest/gtest.h>

#include "erl_search_planning/astar.hpp"
#include "erl_search_planning/planning_interface.hpp"
#include "erl_env/environment_2d.hpp"
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
    bool allow_diagonal = true;
    int step_size = 1;
    Eigen::Vector2d metric_start_coords = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 1});

    std::shared_ptr<CostBase> cost_func = nullptr;  // use EuclideanDistanceCost as default
    auto env = std::make_shared<Environment2D>(allow_diagonal, step_size, false, cost_func, grid_map);
    auto planning_interface = std::make_shared<PlanningInterface>(env, metric_start_coords, metric_goal, metric_goal_tolerance);
    std::shared_ptr<erl::search_planning::astar::Output> result;
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
    Eigen::Vector4d terminal_cost(1000.0, 1000.0, 0.0, 1100.0);

    bool allow_diagonal = true;
    int step_size = 1;
    std::shared_ptr<CostBase> cost_func = nullptr;  // use EuclideanDistanceCost as default
    auto env = std::make_shared<Environment2D>(allow_diagonal, step_size, false, cost_func, grid_map);
    Eigen::Vector2d metric_start_coords = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 1});
    auto planning_interface = std::make_shared<PlanningInterface>(env, metric_start_coords, metric_goals, metric_goals_tolerance, terminal_cost);
    std::shared_ptr<erl::search_planning::astar::Output> result;
    ReportTime<std::chrono::microseconds>("AStar2DTest::PlanWithFourGoals", 0, true, [&]() { result = astar::AStar(planning_interface).Plan(); });
    std::cout << "Path to goal " << result->goal_index << " cost: " << result->cost << std::endl;

    cv::Mat img = planning_interface->GetEnvironment()->ShowPaths({{result->goal_index, result->path}});
    cv::imwrite("AStar2DTest-PlanWithFourGoals.png", img);

    EXPECT_NEAR(result->cost, 1015.485281374, 1e-6);
    // EXPECT_NEAR(result->path_costs[1], 1021.899494937, 1e-6);
    // EXPECT_NEAR(result->path_costs[3], 1110.65685425, 1e-6);
}
