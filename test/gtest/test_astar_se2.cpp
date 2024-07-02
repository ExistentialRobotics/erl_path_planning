#include "erl_search_planning/astar.hpp"
#include "erl_search_planning/planning_interface.hpp"

#include <gtest/gtest.h>
// THE ABOVE TWO HEADERS MUST BE INCLUDED BEFORE THE FOLLOWING HEADERS
#include "erl_env/ddc_motion_primitive.hpp"
#include "erl_env/environment_se2.hpp"
#include "erl_geometry/bresenham_2d.hpp"
#include "erl_search_planning/heuristic.hpp"

using namespace erl::common;
using namespace erl::env;
using namespace erl::geometry;
using namespace erl::search_planning;

TEST(AStarSE2Test, PlanWithOri25Eps2) {

    const int kNumThetas = 37;
    const double kCollisionCheckDt = 0.05;
    const double kEps = 2;
    const long kMaxNumIterations = -1;
    const bool kReopenInconsistent = false;
    const bool kLog = false;

    Eigen::Vector3d metric_start_coords(90, 10, 0);
    Eigen::Vector3d metric_goal_coords(1, 50, 0);
    Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.1, 0.1, 0.53);
    Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
    auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
    // the loaded matrix is a map with top left corner at (0, 0) and bottom right corner at (100, 100), x increasing to the right and y increasing down,
    // column-major order (along the y-axis)
    // the grid map is a map with top left corner at (0, 0) and bottom right corner at (100, 100), x increasing down and y increasing to the right,
    // row-major order (along the y-axis). So, we flatten the matrix directly.
    auto data_dir = std::filesystem::path(__FILE__).parent_path();
    Eigen::MatrixX8U map_data = LoadEigenMatrixFromTextFile<uint8_t>((data_dir / "circles_map_1001x1001.txt").string());
    Eigen::VectorX8U data = map_data.transpose().reshaped(map_shape.prod(), 1);  // row-major order
    auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));

    // visualize the grid map
    cv::Mat img;
    cv::eigen2cv(map_data, img);
    img = img * 255;
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    Eigen::Vector2i start_coords = grid_map_info->MeterToGridForPoints(metric_start_coords.head<2>());
    Eigen::Vector2i goal_coords = grid_map_info->MeterToGridForPoints(metric_goal_coords.head<2>());
    // grid_map: row is x, col is y
    // img: row is y, col is x
    cv::circle(img, cv::Point(start_coords[1], start_coords[0]), 5, cv::Scalar(0, 0, 255), -1);
    cv::drawMarker(img, cv::Point(goal_coords[1], goal_coords[0]), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
    cv::imshow("grid map", img);
    cv::waitKey(1);

    std::cout << "start coords: " << metric_start_coords.transpose() << std::endl
              << "goal coords: " << metric_goal_coords.transpose() << std::endl
              << "map res x: " << grid_map_info->Resolution()[0] << std::endl
              << "map res y: " << grid_map_info->Resolution()[1] << std::endl
              << std::flush;
    auto env_setting = std::make_shared<EnvironmentSe2::Setting>();
    env_setting->time_step = kCollisionCheckDt;
    env_setting->motion_primitives = LoadDdcMotionPrimitivesFromYaml((data_dir / "ddc_motion_primitives.yaml").string());
    env_setting->num_orientations = kNumThetas;
    auto env = std::make_shared<EnvironmentSe2>(grid_map, env_setting);
    auto heuristic = std::make_shared<EuclideanDistanceHeuristic<2>>(metric_goal_coords, metric_goal_tolerance);
    auto planning_interface = std::make_shared<PlanningInterface>(env, metric_start_coords, metric_goal_coords, metric_goal_tolerance, 0, heuristic);

    auto astar_setting = std::make_shared<astar::AStar::Setting>();
    astar_setting->eps = kEps;
    astar_setting->max_num_iterations = kMaxNumIterations;
    astar_setting->reopen_inconsistent = kReopenInconsistent;
    astar_setting->log = kLog;
    std::shared_ptr<astar::Output> result = astar::AStar(planning_interface, astar_setting).Plan();

    img = env->ShowPaths({{result->goal_index, result->path}}, false);
    cv::imwrite("AStarSE2Test-PlanWithOri25Eps2.png", img);

    std::cout << "Path start: " << result->path.col(0).transpose() << std::endl
              << "Path end: " << result->path.col(result->path.cols() - 1).transpose() << std::endl
              << "Path length: " << result->path.cols() << std::endl
              << "Path cost: " << result->cost << std::endl
              << "Number of actions: " << result->action_coords.size() << std::endl
              << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
    EXPECT_NEAR(result->cost, 142.39922522662116, 1e-6);
    EXPECT_EQ(result->action_coords.size(), 136);
    EXPECT_EQ(result->inconsistent_list.size(), 0);
}

// TEST(AStarSE2Test, PlanWithOri25Eps1) {
//
//     const int kNumThetas = 25;
//     const double kCollisionCheckDt = 0.05;
//     const double kEps = 1;
//     const long kMaxNumReachedGoals = -1;
//     const long kMaxNumIterations = -1;
//     const bool kReopenInconsistent = false;
//     const bool kLog = false;
//
//     Eigen::Vector3d metric_goal_coords(50, 1, 0);
//     Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
//     Eigen::Scalard terminal_cost(0.);
//     Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
//     Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
//     Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
//     auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
//     Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
//     auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
//     auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
//     auto planning_interface =
//         std::make_shared<PlanningSe2>(metric_goal_coords, metric_goal_tolerance, terminal_cost, kCollisionCheckDt, motion_primitives, grid_map, kNumThetas);
//     Eigen::Vector3d metric_start_coords(10, 90, 0);
//     auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();
//
//     std::cout << "Path length: " << result->paths[0].size() << std::endl
//               << "Path cost: " << result->path_costs[0] << std::endl
//               << "Number of actions: " << result->action_ids[0].size() << std::endl
//               << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
//     EXPECT_EQ(result->path_costs[0], 95);
//     EXPECT_EQ(result->action_ids[0].size(), 95);
//     EXPECT_EQ(result->inconsistent_list.size(), 0);
// }
//
// TEST(AStarSE2Test, PlanWithOri101Eps1) {
//
//     const int kNumThetas = 101;
//     const double kCollisionCheckDt = 0.05;
//     const double kEps = 1;
//     const long kMaxNumReachedGoals = -1;
//     const long kMaxNumIterations = -1;
//     const bool kReopenInconsistent = false;
//     const bool kLog = false;
//
//     Eigen::Vector3d metric_goal_coords(50, 1, 0);
//     Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
//     Eigen::Scalard terminal_cost(0.);
//     Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
//     Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
//     Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
//     auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
//     Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
//     auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
//     auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
//     auto planning_interface =
//         std::make_shared<PlanningSe2>(metric_goal_coords, metric_goal_tolerance, terminal_cost, kCollisionCheckDt, motion_primitives, grid_map, kNumThetas);
//     Eigen::Vector3d metric_start_coords(10, 90, 0);
//     auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();
//
//     std::cout << "Path length: " << result->paths[0].size() << std::endl
//               << "Path cost: " << result->path_costs[0] << std::endl
//               << "Number of actions: " << result->action_ids[0].size() << std::endl
//               << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
//     EXPECT_EQ(result->path_costs[0], 95);
//     EXPECT_EQ(result->action_ids[0].size(), 95);
//     EXPECT_EQ(result->inconsistent_list.size(), 0);
// }
//
// TEST(AStarSE2Test, PlanWithOri361Eps1) {
//
//     const int kNumThetas = 361;
//     const double kCollisionCheckDt = 0.05;
//     const double kEps = 1;
//     const long kMaxNumReachedGoals = -1;
//     const long kMaxNumIterations = -1;
//     const bool kReopenInconsistent = false;
//     const bool kLog = false;
//
//     Eigen::Vector3d metric_goal_coords(50, 1, 0);
//     Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
//     Eigen::Scalard terminal_cost(0.);
//     Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
//     Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
//     Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
//     auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
//     Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
//     auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
//     auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
//     auto planning_interface =
//         std::make_shared<PlanningSe2>(metric_goal_coords, metric_goal_tolerance, terminal_cost, kCollisionCheckDt, motion_primitives, grid_map, kNumThetas);
//     Eigen::Vector3d metric_start_coords(10, 90, 0);
//     auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();
//
//     std::cout << "Path length: " << result->paths[0].size() << std::endl
//               << "Path cost: " << result->path_costs[0] << std::endl
//               << "Number of actions: " << result->action_ids[0].size() << std::endl
//               << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
//     EXPECT_EQ(result->path_costs[0], 95);
//     EXPECT_EQ(result->action_ids[0].size(), 95);
//     EXPECT_EQ(result->inconsistent_list.size(), 0);
// }
//
// TEST(AStarSE2Test, PlanWithTriangleShape) {
//
//     const int kNumThetas = 25;
//     const double kCollisionCheckDt = 0.05;
//     const double kEps = 1;
//     const long kMaxNumReachedGoals = -1;
//     const long kMaxNumIterations = -1;
//     const bool kReopenInconsistent = false;
//     const bool kLog = false;
//
//     Eigen::Vector3d metric_goal_coords(50, 1, 0);
//     Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.25, 0.25, 0.53);
//     Eigen::Scalard terminal_cost(0.);
//     Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
//     Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
//     Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
//     auto grid_map_info = std::make_shared<GridMapInfo2D>(map_shape, map_min, map_max);
//     Eigen::VectorX8U data = LoadEigenMatrixFromTextFile<uint8_t>(GRID_MAP_DATA_FILE).reshaped(map_shape.prod(), 1);
//     auto grid_map = std::make_shared<GridMapUnsigned2D>(grid_map_info, std::move(data));
//     auto motion_primitives = LoadDdcMotionPrimitivesFromYaml(DDC_MOTION_PRIMITIVES_FILE);
//     Eigen::Matrix23d triangle_vertices;
//     // clang-format off
//     triangle_vertices << -0.3, 0.3, -0.3,
//                          -0.2,   0,  0.2;
//     // clang-format on
//     triangle_vertices.array() *= 3;
//     // Eigen::Matrix2Xd metric_grids = grid_map_info->GetMetricCoordinatesOfFilledMetricPolygon(triangle_vertices);
//     // If we only use the contour, we may miss some obstacles that are inside the triangle.
//     // Eigen::Matrix2Xd metric_grids =
//     //     grid_map_info->PixelToMeterForPoints(ComputePixelsOfPolygonContour(grid_map_info->MeterToPixelForPoints(triangle_vertices)));
//     double inflate_scale = 1.0;
//     auto planning_interface = std::make_shared<PlanningSe2>(
//         metric_goal_coords,
//         metric_goal_tolerance,
//         terminal_cost,
//         kCollisionCheckDt,
//         motion_primitives,
//         grid_map,
//         kNumThetas,
//         inflate_scale,
//         triangle_vertices);
//     Eigen::Vector3d metric_start_coords(10, 90, 0);
//     auto result = AStar(metric_start_coords, planning_interface, kEps, kMaxNumReachedGoals, kMaxNumIterations, kReopenInconsistent, kLog).Plan();
//
//     std::cout << "Path length: " << result->paths[0].size() << std::endl
//               << "Path cost: " << result->path_costs[0] << std::endl
//               << "Number of actions: " << result->action_ids[0].size() << std::endl
//               << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
//     EXPECT_EQ(result->path_costs[0], 95);
//     EXPECT_EQ(result->action_ids[0].size(), 95);
//     EXPECT_EQ(result->inconsistent_list.size(), 0);
// }
