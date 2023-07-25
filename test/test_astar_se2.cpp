#include <gtest/gtest.h>

#include "erl_search_planning/astar.hpp"
#include "erl_search_planning/planning_se2.hpp"
// THE ABOVE TWO HEADERS MUST BE INCLUDED BEFORE THE FOLLOWING HEADERS
#include "erl_geometry/bresenham_2d.hpp"

using namespace erl::common;
using namespace erl::env;
using namespace erl::geometry;
using namespace erl::search_planning;

TEST(AStarSE2Test, PlanWithOri25Eps2) {

    const int kNumThetas = 25;
    const double kCollisionCheckDt = 0.05;
    const double kEps = 2;
    const long kMaxNumReachedGoals = -1;
    const long kMaxNumIterations = -1;
    const bool kReopenInconsistent = false;
    const bool kLog = false;

    Eigen::Vector3d metric_goal_coords(50, 1, 0);
    Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
    Eigen::Scalard terrain_cost(0.);
    Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
    auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
    // the loaded matrix is a map with top left corner at (0, 0) and bottom right corner at (100, 100), x increasing to the right and y increasing down,
    // column-major order (along the y-axis)
    // the grid map is a map with top left corner at (0, 0) and bottom right corner at (100, 100), x increasing down and y increasing to the right,
    // row-major order (along the y-axis). So, we flatten the matrix directly.
    Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
    auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
    auto planning_interface = std::make_shared<PlanningSe2>(
        metric_goal_coords,
        metric_goal_tolerance,
        terrain_cost,
        kCollisionCheckDt,
        motion_primitives,
        grid_map,
        kNumThetas);
    Eigen::Vector3d metric_start_coords(10, 90, 0);
    auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();

    std::cout << "Path length: " << result->paths[0].size() << std::endl
              << "Path cost: " << result->path_costs[0] << std::endl
              << "Number of actions: " << result->action_ids[0].size() << std::endl
              << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
    EXPECT_EQ(result->path_costs[0], 95);
    EXPECT_EQ(result->action_ids[0].size(), 95);
    EXPECT_EQ(result->inconsistent_list.size(), 0);
}

TEST(AStarSE2Test, PlanWithOri25Eps1) {

    const int kNumThetas = 25;
    const double kCollisionCheckDt = 0.05;
    const double kEps = 1;
    const long kMaxNumReachedGoals = -1;
    const long kMaxNumIterations = -1;
    const bool kReopenInconsistent = false;
    const bool kLog = false;

    Eigen::Vector3d metric_goal_coords(50, 1, 0);
    Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
    Eigen::Scalard terminal_cost(0.);
    Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
    auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
    Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
    auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
    auto planning_interface = std::make_shared<PlanningSe2>(
        metric_goal_coords,
        metric_goal_tolerance,
        terminal_cost,
        kCollisionCheckDt,
        motion_primitives,
        grid_map,
        kNumThetas);
    Eigen::Vector3d metric_start_coords(10, 90, 0);
    auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();

    std::cout << "Path length: " << result->paths[0].size() << std::endl
              << "Path cost: " << result->path_costs[0] << std::endl
              << "Number of actions: " << result->action_ids[0].size() << std::endl
              << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
    EXPECT_EQ(result->path_costs[0], 95);
    EXPECT_EQ(result->action_ids[0].size(), 95);
    EXPECT_EQ(result->inconsistent_list.size(), 0);
}

TEST(AStarSE2Test, PlanWithOri101Eps1) {

    const int kNumThetas = 101;
    const double kCollisionCheckDt = 0.05;
    const double kEps = 1;
    const long kMaxNumReachedGoals = -1;
    const long kMaxNumIterations = -1;
    const bool kReopenInconsistent = false;
    const bool kLog = false;

    Eigen::Vector3d metric_goal_coords(50, 1, 0);
    Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
    Eigen::Scalard terminal_cost(0.);
    Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
    auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
    Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
    auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
    auto planning_interface = std::make_shared<PlanningSe2>(
        metric_goal_coords,
        metric_goal_tolerance,
        terminal_cost,
        kCollisionCheckDt,
        motion_primitives,
        grid_map,
        kNumThetas);
    Eigen::Vector3d metric_start_coords(10, 90, 0);
    auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();

    std::cout << "Path length: " << result->paths[0].size() << std::endl
              << "Path cost: " << result->path_costs[0] << std::endl
              << "Number of actions: " << result->action_ids[0].size() << std::endl
              << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
    EXPECT_EQ(result->path_costs[0], 95);
    EXPECT_EQ(result->action_ids[0].size(), 95);
    EXPECT_EQ(result->inconsistent_list.size(), 0);
}

TEST(AStarSE2Test, PlanWithOri361Eps1) {

    const int kNumThetas = 361;
    const double kCollisionCheckDt = 0.05;
    const double kEps = 1;
    const long kMaxNumReachedGoals = -1;
    const long kMaxNumIterations = -1;
    const bool kReopenInconsistent = false;
    const bool kLog = false;

    Eigen::Vector3d metric_goal_coords(50, 1, 0);
    Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
    Eigen::Scalard terminal_cost(0.);
    Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
    auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
    Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
    auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
    auto planning_interface = std::make_shared<PlanningSe2>(
        metric_goal_coords,
        metric_goal_tolerance,
        terminal_cost,
        kCollisionCheckDt,
        motion_primitives,
        grid_map,
        kNumThetas);
    Eigen::Vector3d metric_start_coords(10, 90, 0);
    auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();

    std::cout << "Path length: " << result->paths[0].size() << std::endl
              << "Path cost: " << result->path_costs[0] << std::endl
              << "Number of actions: " << result->action_ids[0].size() << std::endl
              << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
    EXPECT_EQ(result->path_costs[0], 95);
    EXPECT_EQ(result->action_ids[0].size(), 95);
    EXPECT_EQ(result->inconsistent_list.size(), 0);
}

TEST(AStarSE2Test, PlanWithTriangleShape) {

    const int kNumThetas = 25;
    const double kCollisionCheckDt = 0.05;
    const double kEps = 1;
    const long kMaxNumReachedGoals = -1;
    const long kMaxNumIterations = -1;
    const bool kReopenInconsistent = false;
    const bool kLog = false;

    Eigen::Vector3d metric_goal_coords(50, 1, 0);
    Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
    Eigen::Scalard terminal_cost(0.);
    Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
    auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
    Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
    auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
    Eigen::Matrix23d triangle_vertices;
    // clang-format off
    triangle_vertices << -0.3, 0.3, -0.3,
                         -0.2,   0,  0.2;
    // clang-format on
    triangle_vertices.array() *= 3;
    // Eigen::Matrix2Xd metric_grids = grid_map_info->GetMetricCoordinatesOfFilledMetricPolygon(triangle_vertices);
    // If we only use the contour, we may miss some obstacles that are inside the triangle.
    // Eigen::Matrix2Xd metric_grids =
    //     grid_map_info->PixelToMeterForPoints(ComputePixelsOfPolygonContour(grid_map_info->MeterToPixelForPoints(triangle_vertices)));
    double inflate_scale = 1.0;
    auto planning_interface = std::make_shared<PlanningSe2>(
        metric_goal_coords,
        metric_goal_tolerance,
        terminal_cost,
        kCollisionCheckDt,
        motion_primitives,
        grid_map,
        kNumThetas,
        inflate_scale,
        triangle_vertices);
    Eigen::Vector3d metric_start_coords(10, 90, 0);
    auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();

    std::cout << "Path length: " << result->paths[0].size() << std::endl
              << "Path cost: " << result->path_costs[0] << std::endl
              << "Number of actions: " << result->action_ids[0].size() << std::endl
              << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
    EXPECT_EQ(result->path_costs[0], 95);
    EXPECT_EQ(result->action_ids[0].size(), 95);
    EXPECT_EQ(result->inconsistent_list.size(), 0);
}
