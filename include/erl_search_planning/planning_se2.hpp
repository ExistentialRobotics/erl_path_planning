//#pragma once
//
//#include "erl_env/environment_se2.hpp"
//#include "erl_common/grid_map.hpp"
//#include "planning_interface.hpp"
//
//namespace erl::search_planning {
//
//    class PlanningSe2 : public PlanningInterface {
//
//    public:
//        PlanningSe2(
//            std::shared_ptr<env::EnvironmentSe2> env,
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_coords,
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_tolerance,
//            const Eigen::Ref<const Eigen::VectorXd> &terminal_costs)
//            : PlanningInterface(std::move(env), metric_goals_coords, metric_goals_tolerance, terminal_costs) {}  // the robot is a point
//
//        PlanningSe2(
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_coords,
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_tolerance,
//            const Eigen::Ref<const Eigen::VectorXd> &terminal_costs,
//            double collision_check_dt,
//            std::vector<env::DdcMotionPrimitive> motion_primitives,
//            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
//            int num_orientations)
//            : PlanningSe2(
//                  std::make_shared<env::EnvironmentSe2>(collision_check_dt, std::move(motion_primitives), grid_map, num_orientations),
//                  metric_goals_coords,
//                  metric_goals_tolerance,
//                  terminal_costs) {}
//
//        PlanningSe2(
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_coords,
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_tolerance,
//            const Eigen::Ref<const Eigen::VectorXd> &terminal_costs,
//            double collision_check_dt,
//            std::vector<env::DdcMotionPrimitive> motion_primitives,
//            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
//            int num_thetas,
//            double inflate_scale,
//            const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices)  // the robot is assumed to be a polygon
//            : PlanningSe2(
//                  std::make_shared<
//                      env::EnvironmentSe2>(collision_check_dt, std::move(motion_primitives), grid_map, num_thetas, inflate_scale, shape_metric_vertices),
//                  metric_goals_coords,
//                  metric_goals_tolerance,
//                  terminal_costs) {
//            // // cv::Mat original_map = InitializeGridMap2D(grid_map) * 255;  // DEBUG
//            // // cv::imshow("planning differential_drive_controls: original map", original_map);  // DEBUG
//            //
//            // cv::Mat map = m_inflated_grid_maps_[0] * 255;  // DEBUG
//            // // cv::imshow("planning differential_drive_controls: inflated map", map);  // DEBUG
//            //
//            // long num_goals = metric_goals_coords.cols();
//            // long num_vertices = shape_metric_vertices.cols();
//            // for (int i = 0; i < num_goals; ++i) {
//            //     Eigen::Matrix2d rotation_matrix = Eigen::Rotation2Dd(metric_goals_coords(2, i)).toRotationMatrix();
//            //     Eigen::Matrix2Xd shape_metric_vertices_rotated = (rotation_matrix * shape_metric_vertices).colwise() + metric_goals_coords.col(i).head<2>();
//            //     std::vector<std::vector<cv::Point>> contours(1);
//            //     auto &contour = contours[0];
//            //     contour.reserve(num_vertices);
//            //     for (int j = 0; j < num_vertices; ++j) {
//            //         contour.emplace_back(
//            //             m_grid_map_info_->MeterToGridForValue(shape_metric_vertices_rotated(1, j), 1),
//            //             m_grid_map_info_->MeterToGridForValue(shape_metric_vertices_rotated(0, j), 0));
//            //     }
//            //     cv::drawContours(map, contours, 0, cv::Scalar(128), cv::FILLED);
//            // }
//            //
//            // // cv::circle(map, cv::Point(763, 435), 3, cv::Scalar(200), cv::FILLED);
//            //
//            // cv::imshow("planning differential_drive_controls: map with goals", map);
//            // cv::waitKey(0);  // DEBUG
//        }
//
//        [[nodiscard]] double
//        ComputeHeuristic(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
//            // use the custom heuristic
//            if (m_compute_heuristic_callback_) {
//                return m_compute_heuristic_callback_(
//                    metric_state,
//                    MetricToGrid(metric_state),
//                    m_goals_,
//                    m_goals_tolerances_,
//                    m_terminal_costs_,
//                    m_goals_reached_);
//            }
//
//            double min_h = std::numeric_limits<double>::max();
//            long num_goals = GetNumGoals();
//            for (long i = 0; i < num_goals; ++i) {
//                double dx = std::abs(metric_state[0] - m_goals_(0, i));
//                dx = std::max(dx - m_goals_tolerances_(0, i), 0.);
//                double dy = std::abs(metric_state[1] - m_goals_(1, i));
//                dy = std::max(dy - m_goals_tolerances_(1, i), 0.);
//                double h = std::sqrt(dx * dx + dy * dy) + m_terminal_costs_[i];
//                if (h < min_h) { min_h = h; }
//            }
//            return min_h;
//        }
//
//        [[nodiscard]] inline int  // return the index of the goal that is reached, -1 if none is reached
//        IsGoal(const Eigen::Ref<const Eigen::VectorXd> &metric_state) override {
//            int num_goals = GetNumGoals();
//            for (int i = 0; i < num_goals; ++i) {
//                if (m_goals_reached_[i] || std::isinf(m_terminal_costs_[i])) { continue; }
//
//                double error_x = std::abs(metric_state[0] - m_goals_(0, i));
//                if (error_x > m_goals_tolerances_(0, i)) { continue; }
//
//                double error_y = std::abs(metric_state[1] - m_goals_(1, i));
//                if (error_y > m_goals_tolerances_(1, i)) { continue; }
//
//                double error_theta = std::abs(metric_state[2] - m_goals_(2, i));
//                error_theta = std::min(error_theta, 2 * M_PI - error_theta);
//                if (error_theta > m_goals_tolerances_(2, i)) { continue; }
//
//                m_goals_reached_[i] = true;
//                return i;
//            }
//
//            return -1;
//        }
//
//        [[nodiscard]] std::size_t
//        GetGridSpaceSize() const override {
//            return m_grid_map_info_->Size();
//        }
//
//        void
//        Reset() override {
//            ResetPlanningInterface();
//            Reset();
//        }
//    };
//
//}  // namespace erl::search_planning
