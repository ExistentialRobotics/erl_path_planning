#include <gtest/gtest.h>

#include "erl_common/grid_map.hpp"
#include "erl_common/test_helper.hpp"
#include "erl_env/environment_2d.hpp"
#include "erl_search_planning/amra_star.hpp"
#include "erl_search_planning/planning_interface_multi_resolutions.hpp"
#include "erl_search_planning/heuristic.hpp"
#include "erl_search_planning/environment_grid_anchor.hpp"

TEST(AMRAStar2DTest, AStarConsistency) {
    using namespace erl::common;
    using namespace erl::env;
    using namespace erl::search_planning;
    using namespace erl::search_planning::amra_star;

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

    bool allow_diagonal = true;
    int step_size = 1;
    bool down_sampled = false;
    std::shared_ptr<CostBase> cost_func = nullptr;
    auto env = std::make_shared<Environment2D>(allow_diagonal, step_size, down_sampled, cost_func, grid_map);
    Eigen::VectorXd start = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 1});
    Eigen::VectorXd goal = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 10});
    Eigen::VectorXd goal_tolerance = Eigen::Vector2d::Zero();

    auto anchor_env = std::make_shared<EnvironmentGridAnchor<2>>(std::vector<std::shared_ptr<EnvironmentBase>>{env}, grid_map_info);
    std::vector<std::shared_ptr<EnvironmentBase>> environments = {anchor_env, env};
    std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics = {
        {std::make_shared<EuclideanDistanceHeuristic>(goal, goal_tolerance), 0},
        {std::make_shared<EuclideanDistanceHeuristic>(goal, goal_tolerance), 1}};

    auto planning_interface = std::make_shared<PlanningInterfaceMultiResolutions>(
        environments,
        heuristics,
        start,
        std::vector<Eigen::VectorXd>{goal},
        std::vector<Eigen::VectorXd>{goal_tolerance});
    std::shared_ptr<Output> result;
    ReportTime<std::chrono::microseconds>("AMRAStar::AStarConsistency", 0, true, [&]() { result = AMRAStar(planning_interface).Plan(); });
    std::cout << "Path cost: " << result->cost << std::endl << "Path: " << std::endl;
    auto &path = result->path;
    long num_points = path.cols();
    for (long i = 0; i < num_points; ++i) { std::cout << path.col(i).transpose() << std::endl; }
    EXPECT_NEAR(result->cost, 15.485281374238571, 1e-6);
}
