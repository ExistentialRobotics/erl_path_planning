#include "erl_common/test_helper.hpp"
#include "erl_search_planning/heuristic.hpp"
#include "erl_search_planning/ltl_2d_heuristic.hpp"

#include <filesystem>

TEST(LinearTemporalLogic, Heuristic2D) {
    GTEST_PREPARE_OUTPUT_DIR();

    using Dtype = double;
    using GridMapInfo = erl::common::GridMapInfo2D<Dtype>;

    auto fsa_setting_yaml = gtest_src_dir / "fsa.yaml";
    auto fsa_setting = std::make_shared<erl::env::FiniteStateAutomaton::Setting>();
    ASSERT_TRUE(fsa_setting->FromYamlFile(fsa_setting_yaml));
    auto fsa = std::make_shared<erl::env::FiniteStateAutomaton>(fsa_setting);

    auto label_map_png = gtest_src_dir / "label_map.png";
    cv::Mat label_map_img = cv::imread(label_map_png.string(), cv::IMREAD_GRAYSCALE);
    Eigen::MatrixX8U label_map_img_eigen;
    cv::cv2eigen(label_map_img, label_map_img_eigen);
    Eigen::MatrixX<uint32_t> label_map = label_map_img_eigen.cast<uint32_t>();

    Eigen::Vector2i map_shape(251, 261);
    Eigen::Vector2d map_min(-5.05, -5.05);
    Eigen::Vector2d map_max(20.05, 21.05);
    auto grid_map_info = std::make_shared<GridMapInfo>(map_shape, map_min, map_max);

    erl::search_planning::LinearTemporalLogicHeuristic2D heuristic(fsa, label_map, grid_map_info);

    Eigen::MatrixXd label_distance_g(32, 9);
    double inf = std::numeric_limits<double>::infinity();
    // clang-format off
    label_distance_g <<
         13.12, 12.972,  10.12,  9.972,  5.021,      0,      0,      0,    inf,
        15.367,  15.22, 14.293, 14.145,  8.132,  5.408,  3.111,      0,    inf,
        19.376, 19.376, 15.071, 15.071,  5.021,  5.021,      0,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
        17.369, 17.221, 20.092, 19.945,  5.021,      0,  5.021,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,    inf,      0,      0,      0,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
         16.12, 15.972,  10.12,  9.972,  10.12,  9.972,  5.099,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,      0,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,      0,    inf,      0,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,      0,      0,      0,      0,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
         13.12, 12.972,  13.12, 12.972,  9.269,  4.249,  6.403,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,      0,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,      0,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,    inf,      0,      0,      0,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,      0,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
           inf,      0,    inf,      0,    inf,      0,    inf,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf,
             0,      0,    inf,      0,      0,      0,      0,      0,    inf,
           inf,    inf,    inf,    inf,    inf,    inf,    inf,      0,    inf;
    // clang-format on

    Eigen::IOFormat
        fmt(          // kCommaInitFmt
            6,        // precision
            0,        // flags
            ", ",     // coeffSeparator
            ",\n",    // rowSeparator
            "",       // rowPrefix
            "",       // rowSuffix
            "<<\n ",  // matPrefix
            ";"       // matSuffix
        );
    std::cout << "Expected answer:" << std::endl
              << label_distance_g.format(fmt) << std::endl
              << "Fsa State Distance:" << std::endl
              << heuristic.label_distance.format(fmt) << std::endl;

    EXPECT_TRUE(heuristic.label_distance.block(0, 0, 3, 8)
                    .isApprox(label_distance_g.block(0, 0, 3, 8), 1e-4));
    EXPECT_TRUE(heuristic.label_distance.block(4, 0, 1, 8)
                    .isApprox(label_distance_g.block(4, 0, 1, 8), 1e-4));
    EXPECT_TRUE(heuristic.label_distance.block(8, 0, 1, 8)
                    .isApprox(label_distance_g.block(8, 0, 1, 8), 1e-4));
    EXPECT_TRUE(heuristic.label_distance.block(16, 0, 1, 8)
                    .isApprox(label_distance_g.block(16, 0, 1, 8), 1e-4));
}
