//#pragma once
//
//#include "erl_env/environment_grid_se2.hpp"
//#include "planning_interface.hpp"
//
//namespace erl::search_planning {
//
//    class PlanningGridSe2 : public PlanningInterface, public env::EnvironmentGridSe2 {
//
//    public:
//        PlanningGridSe2(
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_coords,
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_tolerance,
//            const Eigen::Ref<const Eigen::VectorXd> &terminal_costs,
//            int step_size,
//            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
//            int num_orientations)
//            : PlanningInterface(metric_goals_coords, metric_goals_tolerance, terminal_costs),
//              env::EnvironmentGridSe2(step_size, grid_map, num_orientations) {}  // the robot is a point
//
//        PlanningGridSe2(
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_coords,
//            const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_tolerance,
//            const Eigen::Ref<const Eigen::VectorXd> &terminal_costs,
//            int step_size,
//            const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
//            int num_orientations,
//            double inflate_scale,
//            const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices)
//            : PlanningInterface(metric_goals_coords, metric_goals_tolerance, terminal_costs),
//              env::EnvironmentGridSe2(step_size, grid_map, num_orientations, inflate_scale, shape_metric_vertices) {}  // the robot is a polygon
//
//        [[nodiscard]] inline int
//        IsGoal(const Eigen::Ref<const Eigen::VectorXd> &metric_state) override {
//            int num_goals = GetNumGoals();
//
//            for (int i = 0; i < num_goals; ++i) {
//                if (m_goals_reached_[i] || std::isinf(m_terminal_costs_[i])) { continue; }
//
//                double error_x = std::abs(metric_state[0] - m_goals_(0, i));
//                if (error_x > m_goals_tolerances_(0, i)) { break; }
//
//                double error_y = std::abs(metric_state[1] - m_goals_(1, i));
//                if (error_y > m_goals_tolerances_(1, i)) { break; }
//
//                double error_theta = std::abs(metric_state[2] - m_goals_(2, i));
//                error_theta = std::min(error_theta, 2 * M_PI - error_theta);
//                if (error_theta > m_goals_tolerances_(2, i)) { break; }
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
//}  // namespace erl::search_planning
//
//// #pragma once
////
//// #include "erl_env/environment_grid_se2.hpp"
//// #include "planning_interface.hpp"
////
//// namespace erl::search_planning {
////
////     class PlanningGridSe2 : public PlanningInterface, public env::EnvironmentGridSe2 {
////
////     public:
////         PlanningGridSe2(
////             const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_coords,
////             const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_tolerance,
////             const Eigen::Ref<const Eigen::VectorXd> &terminal_costs,
////             double linear_velocity_min,
////             double linear_velocity_max,
////             double linear_velocity_step,
////             double euclidean_square_distance_cost_weight,
////             double angular_velocity_min,
////             double angular_velocity_max,
////             double angular_velocity_step,
////             double angular_square_distance_cost_weight,
////             double duration_step,
////             double duration,
////             int max_step_size,
////             const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////             int num_orientations)
////             : PlanningInterface(metric_goals_coords, metric_goals_tolerance, terminal_costs),
////               env::EnvironmentGridSe2(
////                   linear_velocity_min,
////                   linear_velocity_max,
////                   linear_velocity_step,
////                   euclidean_square_distance_cost_weight,
////                   angular_velocity_min,
////                   angular_velocity_max,
////                   angular_velocity_step,
////                   angular_square_distance_cost_weight,
////                   duration_step,
////                   duration,
////                   max_step_size,
////                   grid_map,
////                   num_orientations) {}  // the robot is a point
////
////         PlanningGridSe2(
////             const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_coords,
////             const Eigen::Ref<const Eigen::Matrix3Xd> &metric_goals_tolerance,
////             const Eigen::Ref<const Eigen::VectorXd> &terminal_costs,
////             double linear_velocity_min,
////             double linear_velocity_max,
////             double linear_velocity_step,
////             double euclidean_square_distance_cost_weight,
////             double angular_velocity_min,
////             double angular_velocity_max,
////             double angular_velocity_step,
////             double angular_square_distance_cost_weight,
////             double duration_step,
////             double duration,
////             int max_step_size,
////             const std::shared_ptr<common::GridMapUnsigned2D> &grid_map,
////             int num_orientations,
////             double inflate_scale,
////             const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices)
////             : PlanningInterface(metric_goals_coords, metric_goals_tolerance, terminal_costs),
////               env::EnvironmentGridSe2(
////                   linear_velocity_min,
////                   linear_velocity_max,
////                   linear_velocity_step,
////                   euclidean_square_distance_cost_weight,
////                   angular_velocity_min,
////                   angular_velocity_max,
////                   angular_velocity_step,
////                   angular_square_distance_cost_weight,
////                   duration_step,
////                   duration,
////                   max_step_size,
////                   grid_map,
////                   num_orientations,
////                   inflate_scale,
////                   shape_metric_vertices) {}  // the robot is a polygon
////
////         [[nodiscard]] inline int
////         IsGoal(const Eigen::Ref<const Eigen::VectorXd> &metric_state) override {
////             int num_goals = GetNumGoals();
////
////             for (int i = 0; i < num_goals; ++i) {
////                 if (m_goals_reached_[i] || std::isinf(m_terminal_costs_[i])) { continue; }
////
////                 double error_x = std::abs(metric_state[0] - m_goals_(0, i));
////                 if (error_x > m_goals_tolerances_(0, i)) { break; }
////
////                 double error_y = std::abs(metric_state[1] - m_goals_(1, i));
////                 if (error_y > m_goals_tolerances_(1, i)) { break; }
////
////                 double error_theta = std::abs(metric_state[2] - m_goals_(2, i));
////                 error_theta = std::min(error_theta, 2 * M_PI - error_theta);
////                 if (error_theta > m_goals_tolerances_(2, i)) { break; }
////
////                 m_goals_reached_[i] = true;
////                 return i;
////             }
////
////             return -1;
////         }
////
////         [[nodiscard]] std::size_t
////         GetGridSpaceSize() const override {
////             return m_grid_map_info_->GetSize();
////         }
////     };
//// }  // namespace erl::search_planning
