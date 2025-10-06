#include "erl_common/test_helper.hpp"
#include "erl_env/environment_se2.hpp"
#include "erl_geometry/bresenham_2d.hpp"
#include "erl_search_planning/astar.hpp"
#include "erl_search_planning/heuristic.hpp"
#include "erl_search_planning/planning_interface.hpp"

void
RunTest(
    int num_thetas,
    Dtype eps,
    Dtype expected_cost,
    int expected_num_actions,
    const Eigen::Matrix2X<Dtype>& robot_shape_contour = {}) {

    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;
    using GridMap = erl::common::GridMap<uint8_t, Dtype, 2>;
    using EnvironmentSe2 = erl::env::EnvironmentSe2<Dtype>;
    using Se2Heuristic = erl::search_planning::Se2Heuristic<Dtype>;
    using PlanningInterface = erl::search_planning::PlanningInterface<Dtype, 3>;
    using AStar = erl::search_planning::astar::AStar<Dtype, 3>;
    using Output = erl::search_planning::astar::Output<Dtype, 3>;

    constexpr double kCollisionCheckDt = 0.05;
    constexpr long kMaxNumIterations = -1;
    constexpr bool kReopenInconsistent = false;
    constexpr bool kLog = false;

    Eigen::Vector3d metric_start_coords(90, 10, 0);
    Eigen::Vector3d metric_goal_coords(1, 50, 0);
    Eigen::Vector3d metric_goal_tolerance = Eigen::Vector3d(0.1, 0.1, 0.3);
    Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
    auto grid_map_info = std::make_shared<GridMapInfo>(map_shape, map_min, map_max);
    // the loaded matrix is a map with top left corner at (0, 0) and bottom right corner at (100,
    // 100), x increasing to the right and y increasing down, column-major order (along the y-axis)
    // the grid map is a map with top left corner at (0, 0) and bottom right corner at (100, 100), x
    // increasing down and y increasing to the right, row-major order (along the y-axis). So, we
    // flatten the matrix directly.
    Eigen::MatrixX8U map_data = erl::common::LoadEigenMatrixFromTextFile<uint8_t>(
        gtest_src_dir / "circles_map_1001x1001.txt");
    Eigen::VectorX8U data = map_data.transpose().reshaped(map_shape.prod(), 1);  // row-major order
    auto grid_map = std::make_shared<GridMap>(grid_map_info, std::move(data));

    // visualize the grid map
    cv::Mat img;
    cv::eigen2cv(map_data, img);
    img = img * 255;
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    Eigen::Vector2i start_coords =
        grid_map_info->MeterToGridForPoints(metric_start_coords.head<2>());
    Eigen::Vector2i goal_coords = grid_map_info->MeterToGridForPoints(metric_goal_coords.head<2>());
    // grid_map: row is x, col is y
    // img: row is y, col is x
    cv::circle(img, cv::Point(start_coords[1], start_coords[0]), 5, cv::Scalar(0, 0, 255), -1);
    cv::drawMarker(
        img,
        cv::Point(goal_coords[1], goal_coords[0]),
        cv::Scalar(0, 255, 0),
        cv::MARKER_CROSS,
        10,
        2);
    cv::imshow("grid map", img);
    cv::waitKey(1);

    std::cout << "start coords: " << metric_start_coords.transpose() << std::endl
              << "goal coords: " << metric_goal_coords.transpose() << std::endl
              << "map res x: " << grid_map_info->Resolution()[0] << std::endl
              << "map res y: " << grid_map_info->Resolution()[1] << std::endl
              << std::flush;
    auto env_setting = std::make_shared<typename EnvironmentSe2::Setting>();
    env_setting->time_step = kCollisionCheckDt;
    env_setting->motion_primitives =
        erl::common::LoadSequenceFromFile<erl::env::DdcMotionPrimitive<Dtype>>(
            gtest_src_dir / "ddc_motion_primitives.yaml",
            true);
    env_setting->num_orientations = num_thetas;
    if (robot_shape_contour.cols() > 0) { env_setting->robot_metric_contour = robot_shape_contour; }
    auto env = std::make_shared<EnvironmentSe2>(grid_map, env_setting);
    auto heuristic = std::make_shared<Se2Heuristic>(metric_goal_coords, metric_goal_tolerance, 0);
    auto planning_interface = std::make_shared<PlanningInterface>(
        env,
        metric_start_coords,
        metric_goal_coords,
        metric_goal_tolerance,
        0,
        heuristic);

    auto astar_setting = std::make_shared<typename AStar::Setting>();
    astar_setting->eps = eps;
    astar_setting->max_num_iterations = kMaxNumIterations;
    astar_setting->reopen_inconsistent = kReopenInconsistent;
    astar_setting->log = kLog;
    std::shared_ptr<Output> result = AStar(planning_interface, astar_setting).Plan();

    erl::common::DrawTrajectoryInplace<Dtype>(
        img,
        result->path.template topRows<2>(),
        grid_map_info,
        cv::Scalar(255, 0, 0),
        1,
        false);
    cv::imwrite(test_output_dir / "result.png", img);

    std::cout << "Path start: " << result->path.col(0).transpose() << std::endl
              << "Path end: " << result->path.col(result->path.cols() - 1).transpose() << std::endl
              << "Path length: " << result->path.cols() << std::endl
              << "Path cost: " << result->cost << std::endl
              << "Number of actions: " << result->action_indices.size() << std::endl
              << "Number of inconsistent nodes: " << result->inconsistent_list.size() << std::endl;
    EXPECT_NEAR(result->cost, expected_cost, 1e-6);
    EXPECT_EQ(result->action_indices.size(), expected_num_actions);
    EXPECT_EQ(result->inconsistent_list.size(), 0);
}

TEST(AStarSE2Test, PlanWithOri37Eps2) { RunTest(37, 2, 142.39922522662116, 136); }

TEST(AStarSE2Test, PlanWithOri37Eps1) { RunTest(37, 1, 117.89931131255226, 113); }

TEST(AStarSE2Test, PlanWithOri101Eps2) { RunTest(101, 2, 153.19849349620796, 152); }

TEST(AStarSE2Test, PlanWithTriangleShape) {
    Eigen::Matrix23d triangle_vertices;
    // clang-format off
    triangle_vertices << -0.3, 0.3, -0.3,
                         -0.2,   0,  0.2;
    // clang-format on
    triangle_vertices.array() *= 2;
    RunTest(37, 2, 144.09905305475922, 139, triangle_vertices);
}
