#include "erl_common/test_helper.hpp"
#include "erl_env/cost.hpp"
#include "erl_env/environment_2d.hpp"
#include "erl_env/environment_ltl_2d.hpp"
#include "erl_path_planning/astar.hpp"
#include "erl_path_planning/heuristic.hpp"
#include "erl_path_planning/ltl_2d_heuristic.hpp"

TEST(AStar2D, PlanWithSingleGoal) {
    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;
    using GridMap = erl::common::GridMap<uint8_t, Dtype, 2>;
    using CostBase = erl::env::CostBase<Dtype, 2>;
    using EuclideanDistanceCost = erl::env::EuclideanDistanceCost<Dtype, 2>;
    using Environment2D = erl::env::Environment2D<Dtype, uint8_t>;
    using PlanningInterface = erl::path_planning::SearchPlanningInterface<Dtype, 2>;
    using AStar = erl::path_planning::astar::AStar<Dtype, 2>;
    using Output = erl::path_planning::astar::Output<Dtype, 2>;

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
    Eigen::Vector2d metric_goal = grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 10});
    Eigen::Vector2d metric_goal_tolerance = Eigen::Vector2d::Zero();
    Eigen::Vector2d metric_start_coords =
        grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 1});

    std::shared_ptr<CostBase> cost_func = std::make_shared<EuclideanDistanceCost>();
    auto setting = std::make_shared<Environment2D::Setting>();
    setting->SetGridMotionPrimitive(1, true);
    // setting->allow_diagonal = true;
    // setting->max_axis_step = 1;
    auto env = std::make_shared<Environment2D>(grid_map, setting, cost_func);
    const auto planning_interface = std::make_shared<PlanningInterface>(
        env,
        metric_start_coords,
        metric_goal,
        metric_goal_tolerance,
        /*terminal_cost*/ 10.0);
    std::shared_ptr<Output> result;

    using namespace erl::common;
    ReportTime<std::chrono::microseconds>("AStar2D::PlanWithSingleGoal", 0, true, [&]() {
        result = AStar(planning_interface).Plan();
    });
    const auto *plan_record = result->GetLatestRecord();
    ASSERT_TRUE(plan_record != nullptr);
    std::cout << "Path to goal " << plan_record->goal_index << " cost: " << plan_record->cost
              << ", number of controls: " << plan_record->env_action_indices.size() << std::endl;

    EXPECT_DOUBLE_EQ(plan_record->cost - 10.0, 16.071067811865476);
    EXPECT_EQ(plan_record->env_action_indices.size(), 14);
}

TEST(AStar2D, PlanWithFourGoals) {
    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;
    using GridMap = erl::common::GridMap<uint8_t, Dtype, 2>;
    using CostBase = erl::env::CostBase<Dtype, 2>;
    using EuclideanDistanceCost = erl::env::EuclideanDistanceCost<Dtype, 2>;
    using Environment2D = erl::env::Environment2D<Dtype, uint8_t>;
    using PlanningInterface = erl::path_planning::SearchPlanningInterface<Dtype, 2>;
    using AStar = erl::path_planning::astar::AStar<Dtype, 2>;
    using Output = erl::path_planning::astar::Output<Dtype, 2>;

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
    Eigen::Matrix2Xi grid_goals(2, 4);
    // clang-format off
    grid_goals <<  1, 14, 7, 9,
                  10, 14, 1, 2;
    // clang-format on
    std::vector<Eigen::Vector2d> metric_goals;
    std::vector<Eigen::Vector2d> metric_goals_tolerance = {Eigen::Vector2d::Zero()};
    for (long i = 0; i < grid_goals.cols(); ++i) {
        metric_goals.emplace_back(grid_map_info->GridToMeterForPoints(grid_goals.col(i)));
    }
    std::vector<double> terminal_cost{1000.0, 1000.0, 0.0, 1100.0};

    auto setting = std::make_shared<Environment2D::Setting>();
    setting->SetGridMotionPrimitive(1, true);
    std::shared_ptr<CostBase> cost_func = std::make_shared<EuclideanDistanceCost>();
    auto env = std::make_shared<Environment2D>(grid_map, setting, cost_func);
    Eigen::Vector2d metric_start_coords =
        grid_map_info->GridToMeterForPoints(Eigen::Vector2i{1, 1});
    auto planning_interface = std::make_shared<PlanningInterface>(
        env,
        metric_start_coords,
        metric_goals,
        metric_goals_tolerance,
        terminal_cost);
    std::shared_ptr<Output> result;
    auto astar_setting = std::make_shared<AStar::Setting>();
    astar_setting->log = true;
    AStar astar_planner(planning_interface, astar_setting);

    using namespace erl::common;
    ReportTime<std::chrono::microseconds>("AStar2D::PlanWithFourGoals", 0, true, [&]() {
        result = astar_planner.Plan();
    });
    const auto *plan_record = result->GetLatestRecord();
    ASSERT_TRUE(plan_record != nullptr);
    std::cout << "Number of iterations: " << astar_planner.GetIterations() << std::endl;
    std::cout << "Path to goal " << plan_record->goal_index << " cost: " << plan_record->cost
              << ", number of controls: " << plan_record->env_action_indices.size() << std::endl;

    cv::Mat img = cv::Mat(15, 15, CV_8UC1, grid_map_data.data()).clone();
    img = (1 - img) * 255;
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

    DrawTrajectoryInplace<Dtype>(
        img,
        plan_record->path,
        grid_map_info,
        cv::Scalar(255, 0, 0),
        1,
        false);
    Eigen::Vector2i grid_start = grid_map_info->MeterToGridForPoints(metric_start_coords);
    img.at<cv::Vec3b>(grid_start[0], grid_start[1]) = cv::Vec3b(0, 128, 255);
    img.at<cv::Vec3b>(
        grid_goals.col(plan_record->goal_index)[0],
        grid_goals.col(plan_record->goal_index)[1]) = cv::Vec3b(255, 128, 0);
    cv::imwrite(test_output_dir / "result-path.png", img);

    if (astar_setting->log) {
        img = cv::Mat(15, 15, CV_8UC1, grid_map_data.data()).clone();
        img = (1 - img) * 255;
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        for (auto &[itr, closed_state]: result->closed_list) {
            Eigen::Vector2i grid_coords = grid_map_info->MeterToGridForPoints(closed_state);
            img.at<cv::Vec3b>(grid_coords[0], grid_coords[1]) = cv::Vec3b(0, 255, 0);
        }
        cv::imwrite(test_output_dir / "result-closed.png", img);

        img = cv::Mat(15, 15, CV_8UC1, grid_map_data.data()).clone();
        img = (1 - img) * 255;
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        for (auto &[itr, opened_states]: result->opened_list) {
            for (auto &opened_state: opened_states) {
                if (opened_state.size() == 0) { continue; }
                Eigen::Vector2i grid_coords = grid_map_info->MeterToGridForPoints(opened_state);
                img.at<cv::Vec3b>(grid_coords[0], grid_coords[1]) = cv::Vec3b(0, 0, 255);
            }
        }
        cv::imwrite(test_output_dir / "result-opened.png", img);
    }

    EXPECT_DOUBLE_EQ(plan_record->cost, 1016.0710678118655);
    EXPECT_EQ(plan_record->env_action_indices.size(), 14);

    long num_points = plan_record->path.cols();
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (long i = 0; i < num_points; ++i) {
        out << YAML::Flow
            << YAML::convert<Eigen::Vector2d>::encode(Eigen::Vector2d(plan_record->path.col(i)));
    }
    out << YAML::EndSeq;
    std::ofstream ofs(test_output_dir / "path.yaml");
    ofs << out.c_str();

    // continue planning to find path to the next goal
    for (int i = 1; i < 4; ++i) {
        result = astar_planner.Plan();
        plan_record = result->GetLatestRecord();
        ASSERT_TRUE(plan_record != nullptr);
        std::cout << "cost to goal " << plan_record->goal_index << ": " << plan_record->cost
                  << std::endl;
    }
}

void
RunLargeMapWithStepSize(int step_size, double expected_cost, int expected_num_actions) {
    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;
    using GridMap = erl::common::GridMap<uint8_t, Dtype, 2>;
    using CostBase = erl::env::CostBase<Dtype, 2>;
    using EuclideanDistanceCost = erl::env::EuclideanDistanceCost<Dtype, 2>;
    using Environment2D = erl::env::Environment2D<Dtype, uint8_t>;
    using EuclideanDistanceHeuristic = erl::path_planning::EuclideanDistanceHeuristic<Dtype, 2>;
    using PlanningInterface = erl::path_planning::SearchPlanningInterface<Dtype, 2>;
    using AStar = erl::path_planning::astar::AStar<Dtype, 2>;
    using Output = erl::path_planning::astar::Output<Dtype, 2>;

    Eigen::Vector2d metric_start_coords(90, 10);
    Eigen::Vector2d metric_goal_coords(1, 50);
    Eigen::Vector2d metric_goal_tolerance(0., 0.);
    // Eigen::Scalard terrain_cost(0.);
    Eigen::Vector2i map_shape = Eigen::Vector2i(1001, 1001);
    Eigen::Vector2d map_min = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_max = Eigen::Vector2d::Constant(100.0);  // resolution = 0.1
    auto grid_map_info = std::make_shared<GridMapInfo>(map_shape, map_min, map_max);

    Eigen::MatrixX8U map_data = erl::common::LoadEigenMatrixFromTextFile<uint8_t>(
        (gtest_src_dir / "../../data/circles_map_1001x1001.txt").string());
    Eigen::VectorX8U data = map_data.transpose().reshaped(map_shape.prod(), 1);  // row-major
    auto grid_map = std::make_shared<GridMap>(grid_map_info, std::move(data));

    // visualize the grid map
    cv::Mat img;
    cv::eigen2cv(map_data, img);
    img = img * 255;
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    Eigen::Vector2i start_coords = grid_map_info->MeterToGridForPoints(metric_start_coords);
    Eigen::Vector2i goal_coords = grid_map_info->MeterToGridForPoints(metric_goal_coords);
    metric_start_coords = grid_map_info->GridToMeterForPoints(start_coords);
    metric_goal_coords = grid_map_info->GridToMeterForPoints(goal_coords);
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
              << "start grid coords: " << start_coords.transpose() << std::endl
              << "goal coords: " << metric_goal_coords.transpose() << std::endl
              << "goal grid coords: " << goal_coords.transpose() << std::endl
              << "map res x: " << grid_map_info->Resolution()[0] << std::endl
              << "map res y: " << grid_map_info->Resolution()[1] << std::endl
              << std::flush;

    auto setting = std::make_shared<Environment2D::Setting>();
    setting->SetGridMotionPrimitive(step_size, true);
    std::shared_ptr<CostBase> cost_func = std::make_shared<EuclideanDistanceCost>();
    auto env = std::make_shared<Environment2D>(grid_map, setting, cost_func);
    auto heuristic =
        std::make_shared<EuclideanDistanceHeuristic>(metric_goal_coords, metric_goal_tolerance, 0);
    auto planning_interface = std::make_shared<PlanningInterface>(
        env,
        metric_start_coords,
        metric_goal_coords,
        metric_goal_tolerance,
        0,
        heuristic);

    std::shared_ptr<Output> result;
    using namespace erl::common;
    ReportTime<std::chrono::microseconds>(test_info->name(), 0, true, [&]() {
        result = AStar(planning_interface).Plan();
    });
    const auto *plan_record = result->GetLatestRecord();
    ASSERT_TRUE(plan_record != nullptr);
    std::cout << "Path to goal " << plan_record->goal_index << " cost: " << plan_record->cost
              << ", number of controls: " << plan_record->env_action_indices.size() << std::endl;

    DrawTrajectoryInplace<Dtype>(
        img,
        plan_record->path,
        grid_map_info,
        cv::Scalar(255, 0, 0),
        1,
        false);

    cv::imwrite(test_output_dir / "result.png", img);
    EXPECT_DOUBLE_EQ(plan_record->cost, expected_cost);
    EXPECT_EQ(plan_record->env_action_indices.size(), expected_num_actions);
}

TEST(AStar2D, LargeMap_Step1) { RunLargeMapWithStepSize(1, 105.46307941550766, 890); }

TEST(AStar2D, LargeMap_Step2) { RunLargeMapWithStepSize(2, 98.344374725266363, 489); }

TEST(AStar2D, LargeMap_Step3) { RunLargeMapWithStepSize(3, 97.680925318690171, 400); }

TEST(AStar2D, LinearTemporalLogic2D) {
    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using EnvironmentLTL2D = erl::env::EnvironmentLTL2D<Dtype, uint8_t>;
    using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;
    using GridMap = erl::common::GridMap<uint8_t, Dtype, 2>;
    using EuclideanDistanceCost = erl::env::EuclideanDistanceCost<Dtype, 2>;
    using LinearTemporalLogicHeuristic2D =
        erl::path_planning::LinearTemporalLogicHeuristic2D<Dtype>;
    using PlanningInterface = erl::path_planning::SearchPlanningInterface<Dtype, 3>;
    using AStar = erl::path_planning::astar::AStar<Dtype, 3>;
    using Output = erl::path_planning::astar::Output<Dtype, 3>;

    auto env_setting_yaml = gtest_src_dir / "../../data/environment_ltl_2d.yaml";
    auto env_setting = std::make_shared<EnvironmentLTL2D::Setting>();
    ASSERT_TRUE(env_setting->FromYamlFile(env_setting_yaml));

    auto label_map_png = gtest_src_dir / "../../data/label_map.png";
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
    auto env = std::make_shared<EnvironmentLTL2D>(label_map, grid_map, env_setting, cost_func);
    Eigen::Vector3d metric_start_coords(-2, 3, env_setting->fsa->initial_state);
    std::vector<Eigen::Vector3d> metric_goals_coords(env_setting->fsa->accepting_states.size());
    env_setting->fsa->AsSpotGraphHoaFile(test_output_dir / "automaton.aut", false);
    for (std::size_t i = 0; i < metric_goals_coords.size(); ++i) {
        Eigen::Vector3d &goal_coords = metric_goals_coords[i];
        goal_coords[0] = 0;
        goal_coords[1] = 0;
        goal_coords[2] = env_setting->fsa->accepting_states[i];
    }
    double inf = std::numeric_limits<double>::infinity();
    std::vector<Eigen::Vector3d> metric_goal_tolerance;  // only care about LTL state
    metric_goal_tolerance.emplace_back(inf, inf, 0);
    auto heuristic = std::make_shared<LinearTemporalLogicHeuristic2D>(
        env->GetFiniteStateAutomaton(),
        label_map,
        grid_map_info);
    auto planning_interface = std::make_shared<PlanningInterface>(
        env,
        metric_start_coords,
        metric_goals_coords,
        metric_goal_tolerance,
        std::vector<Dtype>{0.0},
        heuristic);
    std::shared_ptr<Output> result;
    AStar astar(planning_interface);

    using namespace erl::common;
    ReportTime<std::chrono::microseconds>("AStar2D::LinearTemporalLogic2D", 0, true, [&]() {
        result = astar.Plan();
    });

    const auto *plan_record = result->GetLatestRecord();
    ASSERT_TRUE(plan_record != nullptr);

    std::cout << "Path to goal " << plan_record->goal_index << " cost: " << plan_record->cost
              << ", number of controls: " << plan_record->env_action_indices.size() << std::endl;

    cv::Mat img = label_map_img.clone();
    cv::normalize(img, img, 0, 255, cv::NORM_MINMAX);
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    Eigen::Vector2i grid_start = grid_map_info->MeterToGridForPoints(metric_start_coords.head<2>());
    img.at<cv::Vec3b>(grid_start[0], grid_start[1]) = cv::Vec3b(0, 128, 255);
    DrawTrajectoryInplace<Dtype>(
        img,
        plan_record->path.topRows<2>(),
        grid_map_info,
        cv::Scalar(255, 0, 0),
        1,
        false);

    cv::imwrite(test_output_dir / "result.png", img);

    EXPECT_DOUBLE_EQ(plan_record->cost, 20.417871555019136);
    EXPECT_EQ(plan_record->path.cols(), 165);

    long num_points = plan_record->path.cols();
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (long i = 0; i < num_points; ++i) {
        out << YAML::Flow
            << YAML::convert<Eigen::Vector3d>::encode(Eigen::Vector3d(plan_record->path.col(i)));
    }
    out << YAML::EndSeq;
    std::ofstream ofs(test_output_dir / "path.yaml");
    ofs << out.c_str();
}
