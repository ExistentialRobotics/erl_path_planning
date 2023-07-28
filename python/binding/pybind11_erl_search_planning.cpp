#include "erl_search_planning/pybind11_erl_search_planning.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_search_planning/astar.hpp"
#include "erl_search_planning/planning_interface.hpp"
#include "erl_search_planning/planning_2d.hpp"
#include "erl_search_planning/planning_se2.hpp"
#include "erl_search_planning/planning_grid_se2.hpp"

using namespace erl::common;
using namespace erl::search_planning;
using namespace erl::env;

// template<class PlanningInterfaceBase = PlanningInterface>
// class PyPlanningInterface : virtual public PlanningInterfaceBase {
// public:
//     using PlanningInterfaceBase::PlanningInterfaceBase;
//
//     [[nodiscard]] std::shared_ptr<EnvironmentBase>
//     GetEnvironment() const {
//         PYBIND11_OVERLOAD_NAME(std::shared_ptr<EnvironmentBase>, PlanningInterfaceBase, "get_environment", GetEnvironment);
//     }
//
//     void
//     SetHeuristic(std::shared_ptr<HeuristicBase> heuristic) const {
//         PYBIND11_OVERLOAD_NAME(void, PlanningInterfaceBase, "set_heuristic", SetHeuristic, heuristic);
//     }
//
//     [[nodiscard]] double
//     GetHeuristic(const std::shared_ptr<EnvironmentState> &state) const {
//         PYBIND11_OVERLOAD_NAME(double, PlanningInterfaceBase, "get_heuristic", GetHeuristic, state);
//     }
//
//     [[nodiscard]] std::vector<Successor>
//     GetSuccessors(const std::shared_ptr<const EnvironmentState> &state) const {
//         PYBIND11_OVERLOAD_NAME(std::vector<Successor>, PlanningInterfaceBase, "get_successors", GetSuccessors, state);
//     }
//
//     [[nodiscard]] inline Eigen::VectorXd
//     GetInitStart() const {
//         PYBIND11_OVERLOAD_NAME(Eigen::VectorXd, PlanningInterfaceBase, "get_init_start", GetInitStart);
//     }
//
//     [[nodiscard]] inline Eigen::VectorXd
//     GetMetricStart() const {
//         PYBIND11_OVERLOAD_NAME(Eigen::VectorXd, PlanningInterfaceBase, "get_metric_start", GetMetricStart);
//     }
//
//     [[nodiscard]] inline Eigen::VectorXi
//     GetGridStart() const {
//         PYBIND11_OVERLOAD_NAME(Eigen::VectorXi, PlanningInterfaceBase, "get_grid_start", GetGridStart)
//     }
//
//     [[nodiscard]] inline Eigen::VectorXd
//     GetMetricGoal(long index) const {
//         PYBIND11_OVERLOAD_NAME(Eigen::VectorXd, PlanningInterfaceBase, "get_goal", GetMetricGoal, index);
//     }
//
//     [[nodiscard]] inline Eigen::VectorXd
//     GetGoalTolerance(long index) const {
//         PYBIND11_OVERLOAD_NAME(Eigen::VectorXd, PlanningInterfaceBase, "get_goal_tolerance", GetGoalTolerance, index);
//     }
//
//     [[nodiscard]] inline int
//     GetNumGoals() const {
//         PYBIND11_OVERLOAD_NAME(int, PlanningInterfaceBase, "get_num_goals", GetNumGoals);
//     }
//
//     [[nodiscard]] inline double
//     GetTerminalCost(int goal_index) const {
//         PYBIND11_OVERLOAD_NAME(double, PlanningInterfaceBase, "get_terminal_cost", GetTerminalCost, goal_index);
//     }
//
//     [[nodiscard]] std::size_t
//     GetGridSpaceSize() const {
//         PYBIND11_OVERLOAD_NAME(std::size_t, PlanningInterfaceBase, "get_grid_space_size", GetGridSpaceSize);
//     }
//
//     [[nodiscard]] int
//     IsMetricGoal(const std::shared_ptr<EnvironmentState> &state, bool ignore_reached = false) {
//         PYBIND11_OVERLOAD_NAME(int, PlanningInterfaceBase, "is_goal", IsMetricGoal, state, ignore_reached);
//     }
//
//     void
//     PlaceRobot() {
//         PYBIND11_OVERLOAD_NAME(void, PlanningInterfaceBase, "place_robot", PlaceRobot);
//     }
//
//     [[nodiscard]] std::size_t
//     StateHashing(const std::shared_ptr<EnvironmentState> &state) const {
//         PYBIND11_OVERLOAD_NAME(std::size_t, PlanningInterfaceBase, "state_hashing", StateHashing, state);
//     }
//
//     [[nodiscard]] std::vector<std::shared_ptr<EnvironmentState>>
//     GetPath(const std::shared_ptr<const EnvironmentState> &state, std::size_t action_index) const {
//         PYBIND11_OVERLOAD_NAME(std::vector<std::shared_ptr<EnvironmentState>>, PlanningInterfaceBase, "get_path", GetPath, state, action_index);
//     }
//
//     void
//     Reset() {
//         PYBIND11_OVERLOAD_NAME(void, PlanningInterfaceBase, "reset", Reset);
//     }
// };

template<class Base = HeuristicBase>
class PyHeuristic : public Base {
public:
    using Base::Base;

    [[nodiscard]] double
    operator()(const EnvironmentState &state) const override {
        PYBIND11_OVERLOAD_PURE_NAME(double, Base, "__call__", operator(), state);
    }
};

static void
BindHeuristics(py::module &m) {
    py::class_<HeuristicBase, PyHeuristic<>, std::shared_ptr<HeuristicBase>>(m, ERL_AS_STRING(HeuristicBase))
        .def(py::init_alias<>())
        .def(py::init_alias<Eigen::VectorXd, Eigen::VectorXd>(), py::arg("goal"), py::arg("goal_tolerance"))
        .def("__call__", &HeuristicBase::operator());
}

static void
BindPlanningInterfaces(py::module &m) {

    // py::class_<PlanningInterface, EnvironmentBase, PyPlanningInterface<>, std::shared_ptr<PlanningInterface>>(m, ERL_AS_STRING(PlanningInterface))
    //     .def(
    //         py::init_alias<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"))
    //     .def("connect_compute_cost_callback", &PlanningInterface::ConnectComputeCostCallback, py::arg("compute_cost_callback"))
    //     .def("connect_compute_heuristic_callback", &PlanningInterface::ConnectComputeHeuristicCallback, py::arg("compute_heuristic_callback"))
    //     .def("get_successors", &PlanningInterface::GetSuccessors, py::arg("current_metric_state"))
    //     .def("compute_heuristic", &PlanningInterface::ComputeHeuristic, py::arg("metric_state"))
    //     .def_property_readonly("num_goals", &PlanningInterface::GetNumGoals)
    //     .def("get_terminal_cost", &PlanningInterface::GetTerminalCost, py::arg("goal_index"))
    //     .def("is_goal", &PlanningInterface::IsMetricGoal, py::arg("metric_state"));

    py::class_<PlanningInterface, std::shared_ptr<PlanningInterface>>(m, ERL_AS_STRING(PlanningInterface))
        .def(
            py::init<std::shared_ptr<EnvironmentBase>, Eigen::VectorXd, std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>(),
            py::arg("env"),
            py::arg("metric_start_coords"),
            py::arg("metric_goals_coords"),
            py::arg("metric_goals_tolerances"))
        .def(
            py::init<std::shared_ptr<EnvironmentBase>, Eigen::VectorXd, std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>, Eigen::VectorXd>(),
            py::arg("env"),
            py::arg("metric_start_coords"),
            py::arg("metric_goals_coords"),
            py::arg("metric_goals_tolerance"),
            py::arg("terminal_costs"))
        .def_property_readonly("env", &PlanningInterface::GetEnvironment)
        .def("set_heuristic", &PlanningInterface::SetHeuristic, py::arg("heuristic"))
        .def("get_heuristic", &PlanningInterface::GetHeuristic, py::arg("state"))
        .def("get_successors", &PlanningInterface::GetSuccessors, py::arg("state"))
        .def_property_readonly("init_start", &PlanningInterface::GetInitStart)
        .def_property_readonly("metric_start", &PlanningInterface::GetMetricStart)
        .def_property_readonly("grid_start", &PlanningInterface::GetGridStart)
        .def("get_goal", &PlanningInterface::GetMetricGoal, py::arg("index"))
        .def("get_goal_tolerance", &PlanningInterface::GetGoalTolerance, py::arg("index"))
        .def_property_readonly("num_goals", &PlanningInterface::GetNumGoals)
        .def("get_terminal_cost", &PlanningInterface::GetTerminalCost, py::arg("goal_index"))
        .def("grid_space_size", &PlanningInterface::GetGridSpaceSize)
        .def("is_goal", &PlanningInterface::IsMetricGoal, py::arg("state"), py::arg("ignore_reached") = false)
        .def("place_robot", &PlanningInterface::PlaceRobot)
        .def("state_hashing", &PlanningInterface::StateHashing, py::arg("state"))
        .def("get_path", &PlanningInterface::GetPath, py::arg("state"), py::arg("action_index"))
        .def("reset", &PlanningInterface::Reset);

    /*
     *                     |----> PlanningInterface------|
     * EnvironmentBase ----|                             |----> Planning2D/DifferentialDriveControls
     *                     |----> Environment2D/DifferentialDriveControls -----|
     */

    // py::class_<Planning2D, Environment2D, PlanningInterface, std::shared_ptr<Planning2D>>(m, ERL_AS_STRING(Planning2D), py::multiple_inheritance())
    //     .def(
    //         py::init<
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &,
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &,
    //             const Eigen::Ref<const Eigen::VectorXd> &,
    //             bool,
    //             int,
    //             const std::shared_ptr<GridMapUnsigned2D> &>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"),
    //         py::arg("allow_diagonal"),
    //         py::arg("step_size"),
    //         py::arg("grid_map"))
    //     .def(
    //         py::init<
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &,
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &,
    //             const Eigen::Ref<const Eigen::VectorXd> &,
    //             bool,
    //             int,
    //             const std::shared_ptr<GridMapUnsigned2D> &,
    //             double,
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"),
    //         py::arg("allow_diagonal"),
    //         py::arg("step_size"),
    //         py::arg("grid_map"),
    //         py::arg("inflate_scale"),
    //         py::arg("shape_metric_vertices"));

    // py::class_<PlanningSe2, EnvironmentSe2, PlanningInterface, std::shared_ptr<PlanningSe2>>(m, ERL_AS_STRING(PlanningSe2), py::multiple_inheritance())
    //     .def(
    //         py::init<
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::VectorXd> &,
    //             double,
    //             std::vector<DdcMotionPrimitive>,
    //             std::shared_ptr<GridMapUnsigned2D>,
    //             int>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"),
    //         py::arg("collision_check_dt"),
    //         py::arg("motion_primitives"),
    //         py::arg("grid_map"),
    //         py::arg("num_thetas"))
    //     .def(
    //         py::init<
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::VectorXd> &,
    //             double,
    //             std::vector<DdcMotionPrimitive>,
    //             std::shared_ptr<GridMapUnsigned2D>,
    //             int,
    //             double,
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"),
    //         py::arg("collision_check_dt"),
    //         py::arg("motion_primitives"),
    //         py::arg("grid_map"),
    //         py::arg("num_thetas"),
    //         py::arg("inflate_scale"),
    //         py::arg("shape_metric_vertices"));
    //
    // py::class_<PlanningGridSe2, EnvironmentGridSe2, PlanningInterface, std::shared_ptr<PlanningGridSe2>>(
    //     m,
    //     ERL_AS_STRING(PlanningGridSe2),
    //     py::multiple_inheritance())
    //     .def(
    //         py::init<
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::VectorXd> &,
    //             int,
    //             const std::shared_ptr<GridMapUnsigned2D> &,
    //             int>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"),
    //         py::arg("step_size"),
    //         py::arg("grid_map"),
    //         py::arg("num_orientations"))
    //     .def(
    //         py::init<
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::VectorXd> &,
    //             int,
    //             const std::shared_ptr<GridMapUnsigned2D> &,
    //             int,
    //             double,
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"),
    //         py::arg("step_size"),
    //         py::arg("grid_map"),
    //         py::arg("num_orientations"),
    //         py::arg("inflate_scale"),
    //         py::arg("shape_metric_vertices"));

    // py::class_<PlanningGridSe2, EnvironmentGridSe2, PlanningInterface, std::shared_ptr<PlanningGridSe2>>(
    //     m,
    //     ERL_AS_STRING(PlanningGridSe2),
    //     py::multiple_inheritance())
    //     .def(
    //         py::init<
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::VectorXd> &,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             int,
    //             const std::shared_ptr<GridMapUnsigned2D> &,
    //             int>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"),
    //         py::arg("linear_velocity_min"),
    //         py::arg("linear_velocity_max"),
    //         py::arg("linear_velocity_step"),
    //         py::arg("euclidean_square_distance_cost_weight"),
    //         py::arg("angular_velocity_min"),
    //         py::arg("angular_velocity_max"),
    //         py::arg("angular_velocity_step"),
    //         py::arg("angular_square_distance_cost_weight"),
    //         py::arg("duration_step"),
    //         py::arg("duration"),
    //         py::arg("max_step_size"),
    //         py::arg("grid_map"),
    //         py::arg("num_orientations"))
    //     .def(
    //         py::init<
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::Matrix3Xd> &,
    //             const Eigen::Ref<const Eigen::VectorXd> &,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             double,
    //             int,
    //             const std::shared_ptr<GridMapUnsigned2D> &,
    //             int,
    //             double,
    //             const Eigen::Ref<const Eigen::Matrix2Xd> &>(),
    //         py::arg("metric_goals_coords"),
    //         py::arg("metric_goals_tolerance"),
    //         py::arg("terminal_costs"),
    //         py::arg("linear_velocity_min"),
    //         py::arg("linear_velocity_max"),
    //         py::arg("linear_velocity_step"),
    //         py::arg("euclidean_square_distance_cost_weight"),
    //         py::arg("angular_velocity_min"),
    //         py::arg("angular_velocity_max"),
    //         py::arg("angular_velocity_step"),
    //         py::arg("angular_square_distance_cost_weight"),
    //         py::arg("duration_step"),
    //         py::arg("duration"),
    //         py::arg("max_step_size"),
    //         py::arg("grid_map"),
    //         py::arg("num_orientations"),
    //         py::arg("inflate_scale"),
    //         py::arg("shape_metric_vertices"));
}

static void
BindAStar(py::module &m) {

    auto astar = m.def_submodule("astar");
    // Output
    py::class_<astar::Output, std::shared_ptr<astar::Output>>(astar, "Output")
        .def_readwrite("paths", &astar::Output::paths)
        .def_readwrite("action_ids", &astar::Output::action_ids)
        .def_readwrite("path_costs", &astar::Output::path_costs)
        .def_readwrite("opened_list", &astar::Output::opened_list)
        .def_readwrite("closed_list", &astar::Output::closed_list)
        .def_readwrite("inconsistent_list", &astar::Output::inconsistent_list);

    // Astar
    py::class_<astar::AStar>(astar, "AStar")
        .def(
            py::init<
                const std::shared_ptr<erl::search_planning::PlanningInterface> &,
                double,
                long,
                long,
                bool,
                bool>(),
            py::arg("planning_interface"),
            py::arg("eps") = 1.0,
            py::arg("max_num_reached_goals") = 1,
            py::arg("max_num_iterations") = -1,
            py::arg("reopen_inconsistent") = false,
            py::arg("log") = false)
        .def("plan", &astar::AStar::Plan);
}

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_search_planning";

    BindHeuristics(m);
    BindPlanningInterfaces(m);
    BindAStar(m);
}
