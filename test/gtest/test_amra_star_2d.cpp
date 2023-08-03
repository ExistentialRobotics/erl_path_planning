#include <gtest/gtest.h>
#include <fstream>

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
    ReportTime<std::chrono::microseconds>("AMRAStar2D::AStarConsistency", 0, true, [&]() { result = AMRAStar(planning_interface).Plan(); });
    std::cout << "Path cost: " << result->cost << std::endl << "Path: " << std::endl;
    auto &path = result->path;
    long num_points = path.cols();
    for (long i = 0; i < num_points; ++i) { std::cout << path.col(i).transpose() << std::endl; }
    EXPECT_NEAR(result->cost, 15.485281374238571, 1e-6);
}

std::filesystem::path src_dir = std::filesystem::path(__FILE__).parent_path();
std::filesystem::path data_dir = std::filesystem::absolute(src_dir / "../amra/dat");
std::filesystem::path result_dir = src_dir.parent_path() / "results";

std::shared_ptr<erl::common::GridMapUnsigned2D>
ReadAmraStarTestMap(const std::string &filepath, bool display = false) {
    std::ifstream file(filepath);
    std::string line, word, temp;
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

    const std::map<char, int> kMovingAiDict = {
        {'.', 1},   // free
        {'G', 1},   // free
        {'@', -1},  // obstacle
        {'O', -1},  // obstacle
        {'T', 0},   // obstacle
        {'S', 1},
        {'W', 2},     // water is only traversible from water
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
            auto itr = kMovingAiDict.find(line[c]);

            int &map_val = map(r, c);

            if (itr != kMovingAiDict.end()) {
                map_val = kMovingAiDict.find(line[c])->second;
            } else {
                int val = int(line[c]) - int('a') + 1;
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
    auto grid_map_info = std::make_shared<erl::common::GridMapInfo2D>(std::move(map_shape), std::move(map_min), std::move(map_max));
    Eigen::MatrixX8U binary_map(grid_map_info->Height(), grid_map_info->Width());
    binary_map.setZero();
    binary_map.topLeftCorner(height, width) = map.cast<uint8_t>().transpose();  // erl::common::GridMapUnsigned2D requires row-major storage

    if (display) {
        cv::Mat img;
        cv::eigen2cv(Eigen::MatrixX8U(255 - binary_map.array().transpose()), img);
        cv::imshow("map", img);
        cv::waitKey(1000);
    }

    auto grid_map = std::make_shared<erl::common::GridMapUnsigned2D>(grid_map_info, Eigen::VectorX8U(binary_map.reshaped(binary_map.size(), 1)));
    return grid_map;
}

void
RunTestWithMap(const std::filesystem::path &map_file, const Eigen::Vector2i &start_grid, const Eigen::Vector2i &goal_grid, double expected_cost) {
    std::string map_name = std::filesystem::relative(map_file, data_dir).string();
    std::string sep(map_name.size() + 2, '=');
    std::cout << sep << std::endl << ' ' << map_name << ' ' << std::endl << sep << std::endl;
    using namespace erl::common;
    using namespace erl::env;
    using namespace erl::search_planning;
    using namespace erl::search_planning::amra_star;

    auto grid_map = ReadAmraStarTestMap(map_file);
    auto grid_map_info = grid_map->info;

    bool allow_diagonal = true;
    auto cost_func = std::make_shared<EuclideanDistanceCost>();
    auto env_high_res = std::make_shared<Environment2D>(allow_diagonal, 1, false, cost_func, grid_map);
    auto env_mid_res = std::make_shared<Environment2D>(allow_diagonal, 3, true, cost_func, grid_map);
    auto env_low_res = std::make_shared<Environment2D>(allow_diagonal, 9, true, cost_func, grid_map);
    std::vector<std::shared_ptr<EnvironmentBase>> envs = {env_high_res, env_mid_res, env_low_res};
    auto env_anchor = std::make_shared<EnvironmentGridAnchor<2>>(envs, grid_map_info);
    std::vector<std::shared_ptr<EnvironmentBase>> all_envs = {env_anchor, env_high_res, env_mid_res, env_low_res};

    Eigen::VectorXd start = grid_map_info->GridToMeterForPoints(start_grid);
    Eigen::VectorXd goal = grid_map_info->GridToMeterForPoints(goal_grid);
    Eigen::VectorXd goal_tolerance = Eigen::Vector2d::Zero();
    auto euclidean_heuristic = std::make_shared<EuclideanDistanceHeuristic>(goal, goal_tolerance);
    std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics = {
        {euclidean_heuristic, 0},
        {euclidean_heuristic, 1},
        {euclidean_heuristic, 2},
        {euclidean_heuristic, 3}};

    auto planning_interface = std::make_shared<PlanningInterfaceMultiResolutions>(all_envs, heuristics, start, goal, goal_tolerance);
    auto setting = std::make_shared<AMRAStar::Setting>();
    setting->log = false;
    std::shared_ptr<Output> result;
    AMRAStar amra_star(planning_interface, setting);
    ReportTime<std::chrono::milliseconds>(map_name.c_str(), 0, true, [&]() { result = amra_star.Plan(); });
    std::cout << "Path cost: " << result->cost << std::endl;
    EXPECT_NEAR(result->cost, expected_cost, 1.e-6);

    // std::cout << "Path: " << std::endl
    // auto &path = result->path;
    // long num_points = path.cols();
    // for (long i = 0; i < num_points; ++i) { std::cout << path.col(i).transpose() << std::endl; }

    // TODO: save result to file for visualization
    // if (!std::filesystem::exists(result_dir)) { std::filesystem::create_directories(result_dir); }
    // result->Save(result_dir / (map_name + ".solution"));
}

TEST(AMRAStar2DTest, MultiResolutions) {
    RunTestWithMap(data_dir / "Boston_0_1024.map", {828, 657}, {1008, 756}, 234.86104730411358);
    RunTestWithMap(data_dir / "Cauldron.map", {342, 450}, {522, 333}, 266.38660980808476);
    RunTestWithMap(data_dir / "Denver_0_1024.map", {306, 171}, {1008, 603}, 916.18768003761579);
    RunTestWithMap(data_dir / "Expedition.map", {720, 423}, {198, 891}, 766.06725471071707);
    RunTestWithMap(data_dir / "NewYork_0_1024.map", {0, 171}, {612, 882}, 999.25583708915997);
    RunTestWithMap(data_dir / "Octopus.map", {549, 837}, {639, 279}, 644.11319062351083);
    RunTestWithMap(data_dir / "TheFrozenSea.map", {288, 414}, {207, 990}, 649.50994061340202);
}
