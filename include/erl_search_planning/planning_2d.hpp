//#pragma once
//
//#include "erl_env/environment_2d.hpp"
//#include "planning_interface.hpp"
//
//namespace erl::search_planning {
//
//    class Planning2D : public PlanningInterface {
//
//    public:
//        Planning2D(
//            std::shared_ptr<env::Environment2D> env,
//            Eigen::VectorXd metric_start_coords,
//            std::vector<Eigen::VectorXd> metric_goals_coords,
//            std::vector<Eigen::VectorXd> metric_goals_tolerance,
//            Eigen::VectorXd terminal_costs)
//            : PlanningInterface(
//                  std::move(env),
//                  std::move(metric_start_coords),
//                  std::move(metric_goals_coords),
//                  std::move(metric_goals_tolerance),
//                  std::move(terminal_costs)) {}  // the robot is a point
//
//        Planning2D(
//            Eigen::VectorXd metric_start_coords,
//            std::vector<Eigen::VectorXd> metric_goals_coords,
//            std::vector<Eigen::VectorXd> metric_goals_tolerance,
//            Eigen::VectorXd terminal_costs,
//            bool allow_diagonal,
//            int step_size,
//            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map)
//            : Planning2D(
//                  std::make_shared<env::Environment2D>(allow_diagonal, step_size, grid_map),
//                  std::move(metric_start_coords),
//                  std::move(metric_goals_coords),
//                  std::move(metric_goals_tolerance),
//                  std::move(terminal_costs)) {}
//
//        Planning2D(
//            Eigen::VectorXd metric_start_coords,
//            std::vector<Eigen::VectorXd> metric_goals_coords,
//            std::vector<Eigen::VectorXd> metric_goals_tolerance,
//            Eigen::VectorXd terminal_costs,
//            bool allow_diagonal,
//            int step_size,
//            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
//            double inflate_scale,
//            const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices)
//            : Planning2D(
//                  std::make_shared<env::Environment2D>(allow_diagonal, step_size, grid_map, inflate_scale, shape_metric_vertices),
//                  std::move(metric_start_coords),
//                  std::move(metric_goals_coords),
//                  std::move(metric_goals_tolerance),
//                  std::move(terminal_costs)) {  // the robot is a polygon
//            //
//            // // TODO: remove
//            // std::cout << "Planning2D" << std::endl
//            //           << "goals: " << std::endl
//            //           << m_goals_ << std::endl
//            //           << "tolerance: " << std::endl
//            //           << m_goals_tolerances_ << std::endl
//            //           << "terminal costs: " << std::endl
//            //           << m_terminal_costs_.transpose() << std::endl;
//            //
//            // // TODO: remove
//            // cv::Mat original_map = InitializeGridMap2D(grid_map) * 255;
//            // cv::imshow("planning2d: original map", original_map);
//            //
//            // cv::Mat map = m_inflated_grid_maps_ * 255;  // inflated map
//            // cv::imshow("planning2d: inflated map", map);
//            //
//            // long num_goals = metric_goals_coords.cols();
//            // long num_vertices = shape_metric_vertices.cols();
//            // for (int i = 0; i < num_goals; ++i) {
//            //     // Eigen::Matrix2d rotation_matrix = Eigen::Rotation2Dd(metric_goals_coords(2, i)).toRotationMatrix();
//            //     // Eigen::Matrix2Xd shape_metric_vertices_rotated = (rotation_matrix * shape_metric_vertices).colwise() +
//            //     metric_goals_coords.col(i).head<2>(); std::vector<std::vector<cv::Point>> contours(1); auto &contour = contours[0];
//            //     contour.reserve(num_vertices);
//            //     for (int j = 0; j < num_vertices; ++j) {
//            //         contour.emplace_back(
//            //             m_grid_map_info_2d_->MeterToGridForValue(shape_metric_vertices(1, j) + metric_goals_coords(1, i), 1),
//            //             m_grid_map_info_2d_->MeterToGridForValue(shape_metric_vertices(0, j) + metric_goals_coords(0, i), 0));
//            //     }
//            //     cv::drawContours(map, contours, 0, cv::Scalar(128), cv::FILLED);
//            // }
//            //
//            // cv::circle(map, cv::Point(763, 435), 3, cv::Scalar(200), cv::FILLED);
//            //
//            // cv::imshow("planning2d: map with goals", map);
//            // cv::waitKey(0);
//
//            // std::cout << "goals: " << std::endl
//            //           << m_goals_ << std::endl
//            //           << "tolerance: " << std::endl
//            //           << m_goals_tolerances_ << std::endl
//            //           << "terminal costs: " << std::endl
//            //           << m_terminal_costs_.transpose() << std::endl;
//        }
//
//        // [[nodiscard]] inline double
//        // ComputeHeuristic(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
//        //     // use the custom heuristic
//        //     if (m_compute_heuristic_callback_) {
//        //         return m_compute_heuristic_callback_(
//        //             metric_state,
//        //             MetricToGrid(metric_state),
//        //             m_goals_,
//        //             m_goals_tolerances_,
//        //             m_terminal_costs_,
//        //             m_goals_reached_);
//        //     }
//        //
//        //     double min_h = std::numeric_limits<double>::max();
//        //     long num_goals = GetNumGoals();
//        //     for (long i = 0; i < num_goals; ++i) {
//        //         double dx = std::abs(metric_state[0] - m_goals_(0, i));
//        //         dx = std::max(dx - m_goals_tolerances_(0, i), 0.);
//        //         double dy = std::abs(metric_state[1] - m_goals_(1, i));
//        //         dy = std::max(dy - m_goals_tolerances_(1, i), 0.);
//        //         double h = std::sqrt(dx * dx + dy * dy) + m_terminal_costs_[i];
//        //         if (h < min_h) { min_h = h; }
//        //     }
//        //     return min_h;
//        // }
//
//        // [[nodiscard]] inline int  // return the index of the goal that is reached, -1 if none is reached
//        // IsGoal(const Eigen::Ref<const Eigen::VectorXd> &metric_state) override {
//        //     int num_goals = GetNumGoals();
//        //     for (int i = 0; i < num_goals; ++i) {
//        //         if (m_goals_reached_[i] || std::isinf(m_terminal_costs_[i])) { continue; }
//        //
//        //         double error_x = std::abs(metric_state[0] - m_goals_(0, i));
//        //         if (error_x > m_goals_tolerances_(0, i)) { continue; }
//        //
//        //         double error_y = std::abs(metric_state[1] - m_goals_(1, i));
//        //         if (error_y > m_goals_tolerances_(1, i)) { continue; }
//        //
//        //         m_goals_reached_[i] = true;
//        //         return i;
//        //     }
//        //
//        //     return -1;
//        // }
//    };
//
//}  // namespace erl::search_planning
