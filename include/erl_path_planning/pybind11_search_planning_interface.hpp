#pragma once

#include "search_planning_interface.hpp"

#include "erl_common/pybind11.hpp"

template<typename Dtype, int Dim>
void
BindSearchPlanningInterfaceImpl(py::module &m, const char *name) {
    using namespace erl::path_planning;
    using PlanningInterface = SearchPlanningInterface<Dtype, Dim>;
    using Env = typename PlanningInterface::Env;
    using MetricState = typename PlanningInterface::MetricState;
    using Heuristic = typename PlanningInterface::Heuristic;

    py::class_<PlanningInterface, std::shared_ptr<PlanningInterface>>(m, name)
        .def(
            py::init<
                std::shared_ptr<Env>,
                MetricState,
                const std::vector<MetricState> &,
                std::vector<MetricState>,
                std::vector<Dtype>,
                std::shared_ptr<Heuristic>>(),
            py::arg("env").none(false),
            py::arg("metric_start_coords"),
            py::arg("metric_goals_coords"),
            py::arg("metric_goals_tolerances"),
            py::arg("terminal_costs") = std::vector({0.0}),
            py::arg("heuristic") = nullptr)
        .def(
            py::init<
                std::shared_ptr<Env>,
                MetricState,
                MetricState,
                MetricState,
                Dtype,
                std::shared_ptr<Heuristic>>(),
            py::arg("env").none(false),
            py::arg("metric_start_coords"),
            py::arg("metric_goal_coords"),
            py::arg("metric_goal_tolerance"),
            py::arg("terminal_cost") = 0.,
            py::arg("heuristic") = nullptr)
        .def_property_readonly("environment", &PlanningInterface::GetEnvironment)
        .def("get_heuristic", &PlanningInterface::GetHeuristic, py::arg("env_state"))
        .def("get_successors", &PlanningInterface::GetSuccessors, py::arg("env_state"))
        .def_property_readonly("start_state", &PlanningInterface::GetStartState)
        .def("get_goal_state", &PlanningInterface::GetGoalState, py::arg("goal_index"))
        .def_property_readonly("num_goals", &PlanningInterface::GetNumGoals)
        .def("get_terminal_cost", &PlanningInterface::GetTerminalCost, py::arg("goal_index"))
        .def("is_metric_goal", &PlanningInterface::IsMetricGoal, py::arg("env_state"))
        .def_static("is_virtual_goal", &PlanningInterface::IsVirtualGoal, py::arg("env_state"))
        .def_static("reach_goal", &PlanningInterface::ReachGoal, py::arg("env_state"))
        .def("state_hashing", &PlanningInterface::StateHashing, py::arg("env_state"))
        .def(
            "get_path",
            &PlanningInterface::GetPath,
            py::arg("env_state"),
            py::arg("action_coords"));
}

template<typename Dtype, int Dim>
void
BindSearchPlanningInterfaceMultiResolutionImpl(py::module &m, const char *name) {
    using namespace erl::path_planning;
    using PlanningInterface = SearchPlanningInterfaceMultiResolutions<Dtype, Dim>;
    using Env = typename PlanningInterface::Env;
    using MetricState = typename PlanningInterface::MetricState;
    using Heuristic = typename PlanningInterface::Heuristic;

    py::class_<PlanningInterface, std::shared_ptr<PlanningInterface>>(m, name)
        .def(
            py::init<
                std::shared_ptr<Env>,
                std::vector<std::pair<std::shared_ptr<Heuristic>, std::size_t>>,
                MetricState,
                const std::vector<MetricState>,
                std::vector<MetricState>,
                std::vector<Dtype>>(),
            py::arg("env_multi_resolution"),
            py::arg("heuristics"),
            py::arg("metric_start_coords"),
            py::arg("metric_goals_coords"),
            py::arg("metric_goals_tolerances"),
            py::arg("terminal_costs") = std::vector{0.0})
        .def(
            py::init<
                std::shared_ptr<Env>,
                std::vector<std::pair<std::shared_ptr<Heuristic>, std::size_t>>,
                MetricState,
                MetricState,
                MetricState>(),
            py::arg("environment_multi_resolution"),
            py::arg("heuristics"),
            py::arg("metric_start_coords"),
            py::arg("metric_goal_coords"),
            py::arg("metric_goal_tolerance"))
        .def_property_readonly("num_heuristics", &PlanningInterface::GetNumHeuristics)
        .def_property_readonly("num_resolution_levels", &PlanningInterface::GetNumResolutionLevels)
        .def("get_heuristic", &PlanningInterface::GetHeuristic, py::arg("heuristic_id"))
        .def(
            "get_resolution_assignment",
            &PlanningInterface::GetResolutionAssignment,
            py::arg("heuristic_id"))
        .def(
            "get_resolution_heuristic_ids",
            &PlanningInterface::GetResolutionHeuristicIds,
            py::arg("resolution_level"))
        .def(
            "get_in_resolution_level_flags",
            &PlanningInterface::GetInResolutionLevelFlags,
            py::arg("env_state").none(false))
        .def(
            "get_successors",
            &PlanningInterface::GetSuccessors,
            py::arg("env_state").none(false),
            py::arg("env_resolution_level"))
        .def_property_readonly("start_state", &PlanningInterface::GetStartState)
        .def("get_goal_state", &PlanningInterface::GetGoalState, py::arg("goal_index"))
        .def_property_readonly("num_goals", &PlanningInterface::GetNumGoals)
        .def(
            "get_heuristic_values",
            &PlanningInterface::GetHeuristicValues,
            py::arg("env_state").none(false))
        .def("is_metric_goal", &PlanningInterface::IsMetricGoal, py::arg("env_state"))
        .def_static(
            "is_virtual_goal",
            &PlanningInterface::IsVirtualGoal,
            py::arg("env_state").none(false))
        .def_static("reach_goal", &PlanningInterface::ReachGoal, py::arg("env_state"))
        .def("state_hashing", &PlanningInterface::StateHashing, py::arg("env_state"))
        .def(
            "get_path",
            &PlanningInterface::GetPath,
            py::arg("env_state"),
            py::arg("env_level"),
            py::arg("action_idx"));
}
