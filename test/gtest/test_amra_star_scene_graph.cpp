#include <filesystem>
#include "erl_common/test_helper.hpp"
#include "erl_env/environment_scene_graph.hpp"
#include "erl_search_planning/amra_star.hpp"
#include "erl_search_planning/heuristic.hpp"

TEST(ERL_SEARCH_PLANNING, AMRAStarSceneGraph) {

    // load environment
    std::filesystem::path path = __FILE__;
    auto data_dir = path.parent_path();
    path = data_dir / "building.yaml";
    auto building = std::make_shared<erl::env::scene_graph::Building>();
    building->FromYamlFile(path.string());

    auto env_setting = std::make_shared<erl::env::EnvironmentSceneGraph::Setting>();
    env_setting->data_dir = data_dir.string();
    env_setting->shape = Eigen::Matrix2Xd(2, 360);
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(360, 0, 2 * M_PI);
    double r = 0.15;
    for (int i = 0; i < 360; ++i) {
        env_setting->shape(0, i) = r * cos(angles[i]);
        env_setting->shape(1, i) = r * sin(angles[i]);
    }
    auto env_scene_graph = std::make_shared<erl::env::EnvironmentSceneGraph>(building, env_setting);

    Eigen::VectorXd start = env_scene_graph->GridToMetric(Eigen::Vector3i(400, 900, 1));
    Eigen::VectorXd goal = env_scene_graph->GridToMetric(Eigen::Vector3i(450, 200, 1));
    Eigen::VectorXd goal_tolerance = Eigen::VectorXd::Zero(3);
    std::vector<std::pair<std::shared_ptr<erl::search_planning::HeuristicBase>, std::size_t>> heuristics = {
        {std::make_shared<erl::search_planning::EuclideanDistanceHeuristic>(goal, goal_tolerance), 0},  // anchor
        {std::make_shared<erl::search_planning::EuclideanDistanceHeuristic>(goal, goal_tolerance), 1},  // kNA
        {std::make_shared<erl::search_planning::EuclideanDistanceHeuristic>(goal, goal_tolerance), 2},  // kObject
        {std::make_shared<erl::search_planning::EuclideanDistanceHeuristic>(goal, goal_tolerance), 3},  // kRoom
        {std::make_shared<erl::search_planning::EuclideanDistanceHeuristic>(goal, goal_tolerance), 4},  // kFloor
    };

    auto planning_interface = std::make_shared<erl::search_planning::PlanningInterfaceMultiResolutions>(
        env_scene_graph,
        heuristics,
        start,
        std::vector<Eigen::VectorXd>{goal},
        std::vector<Eigen::VectorXd>{goal_tolerance});
    auto amra_setting = std::make_shared<erl::search_planning::amra_star::AMRAStar::Setting>();
    amra_setting->log = true;
    erl::search_planning::amra_star::AMRAStar planner(planning_interface, amra_setting);

    std::shared_ptr<erl::search_planning::amra_star::Output> result;
    erl::common::ReportTime<std::chrono::microseconds>("AMRAStarSceneGraph", 0, true, [&]() { result = planner.Plan(); });
    double path_cost = result->costs[result->latest_plan_itr];
    std::cout << "Path cost: " << path_cost << std::endl << "Path: " << std::endl;

    // draw path
    for (auto &itr : result->paths) {
        uint32_t plan_itr = itr.first;
        Eigen::Matrix3Xd amra_path = itr.second;
        long num_points = amra_path.cols();
        cv::Mat cat_map = erl::common::ColorGrayCustom(building->LoadCatMap(data_dir, 1));
        std::vector<cv::Point2i> cv_path;
        cv_path.reserve(num_points);
        for (long i = 0; i < num_points; ++i) {
            Eigen::Vector3d p = amra_path.col(i);
            Eigen::Vector3i grid = env_scene_graph->MetricToGrid(p);
            cv_path.emplace_back(grid[1], grid[0]);
        }
        cv::polylines(cat_map, cv_path, false, cv::Scalar(0, 0, 255), 2);
        cv::imshow("plan_" + std::to_string(plan_itr), cat_map);
    }
    cv::waitKey(0);
}
