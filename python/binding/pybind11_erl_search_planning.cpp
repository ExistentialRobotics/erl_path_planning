#include "erl_common/pybind11.hpp"
#include "erl_search_planning/amra_star.hpp"
#include "erl_search_planning/astar.hpp"
#include "erl_search_planning/planning_interface.hpp"
#include "erl_search_planning/planning_interface_multi_resolutions.hpp"

using namespace erl::common;
using namespace erl::search_planning;
using namespace erl::env;

template<class Base = HeuristicBase>
class PyHeuristic : public Base {
public:
    using Base::Base;

    [[nodiscard]] double
    operator()(const EnvironmentState &state) const override {
        PYBIND11_OVERRIDE_PURE_NAME(double, Base, "__call__", operator(), state);
    }
};

static void
BindHeuristics(const py::module &m) {
    py::class_<HeuristicBase, PyHeuristic<>, std::shared_ptr<HeuristicBase>>(m, "HeuristicBase")
        .def(py::init_alias<>())
        .def(py::init_alias<Eigen::VectorXd, Eigen::VectorXd, double>(), py::arg("goal"), py::arg("goal_tolerance"), py::arg("terminal_cost") = 0.0)
        .def("__call__", &HeuristicBase::operator(), py::arg("env_state"));

    py::class_<EuclideanDistanceHeuristic<2>, HeuristicBase, PyHeuristic<EuclideanDistanceHeuristic<2>>, std::shared_ptr<EuclideanDistanceHeuristic<2>>>(
        m,
        "EuclideanDistanceHeuristic2D")
        .def(py::init<Eigen::VectorXd, Eigen::VectorXd, double>(), py::arg("goal"), py::arg("goal_tolerance"), py::arg("terminal_cost") = 0.0);
    py::class_<ManhattanDistanceHeuristic<2>, HeuristicBase, PyHeuristic<ManhattanDistanceHeuristic<2>>, std::shared_ptr<ManhattanDistanceHeuristic<2>>>(
        m,
        "ManhattanDistanceHeuristic2D")
        .def(py::init<Eigen::VectorXd, Eigen::VectorXd, double>(), py::arg("goal"), py::arg("goal_tolerance"), py::arg("terminal_cost") = 0.0);
    py::class_<DictionaryHeuristic, HeuristicBase, PyHeuristic<DictionaryHeuristic>, std::shared_ptr<DictionaryHeuristic>>(m, "DictionaryHeuristic")
        .def(
            py::init<const std::string &, std::function<long(const erl::env::EnvironmentState &)>, bool>(),
            py::arg("csv_path"),
            py::arg("state_hashing_func"),
            py::arg("assert_on_missing") = true);
    py::class_<MultiGoalsHeuristic, HeuristicBase, PyHeuristic<MultiGoalsHeuristic>, std::shared_ptr<MultiGoalsHeuristic>>(m, "MultiGoalsHeuristic")
        .def(py::init<std::vector<std::shared_ptr<HeuristicBase>>>(), py::arg("heuristics"));
}

static void
BindPlanningInterfaces(const py::module &m) {
    py::class_<PlanningInterface, std::shared_ptr<PlanningInterface>>(m, "PlanningInterface")
        .def(
            py::init<
                std::shared_ptr<EnvironmentBase>,
                Eigen::VectorXd,
                const std::vector<Eigen::VectorXd> &,
                std::vector<Eigen::VectorXd>,
                std::vector<double>,
                std::shared_ptr<HeuristicBase>>(),
            py::arg("env").none(false),
            py::arg("metric_start_coords"),
            py::arg("metric_goals_coords"),
            py::arg("metric_goals_tolerances"),
            py::arg("terminal_costs") = std::vector<double>({0.}),
            py::arg("heuristic") = nullptr)
        .def(
            py::init<std::shared_ptr<EnvironmentBase>, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, double, std::shared_ptr<HeuristicBase>>(),
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
        .def("is_metric_goal", &PlanningInterface::IsMetricGoal, py::arg("env_state"))
        .def_static("is_virtual_goal", &PlanningInterface::IsVirtualGoal, py::arg("env_state"))
        .def("reach_goal", &PlanningInterface::ReachGoal, py::arg("env_state"))
        .def("state_hashing", &PlanningInterface::StateHashing, py::arg("env_state"))
        .def("get_path", &PlanningInterface::GetPath, py::arg("env_state"), py::arg("action_coords"));
}

static void
BindAStar(py::module &m) {

    const auto astar_module = m.def_submodule("astar");
    // Output
    py::class_<astar::Output, std::shared_ptr<astar::Output>>(astar_module, "Output")
        .def_readwrite("path", &astar::Output::path)
        .def_readwrite("action_coords", &astar::Output::action_coords)
        .def_readwrite("cost", &astar::Output::cost)
        .def_readwrite("opened_list", &astar::Output::opened_list)
        .def_readwrite("closed_list", &astar::Output::closed_list)
        .def_readwrite("inconsistent_list", &astar::Output::inconsistent_list);

    // Astar
    py::class_<astar::AStar> astar(astar_module, "AStar");
    py::class_<astar::AStar::Setting, YamlableBase, std::shared_ptr<astar::AStar::Setting>>(astar, "Setting")
        .def(py::init<>())
        .def_readwrite("eps", &astar::AStar::Setting::eps)
        .def_readwrite("max_num_iterations", &astar::AStar::Setting::max_num_iterations)
        .def_readwrite("log", &astar::AStar::Setting::log)
        .def_readwrite("reopen_inconsistent", &astar::AStar::Setting::reopen_inconsistent);
    astar
        .def(
            py::init<std::shared_ptr<PlanningInterface>, std::shared_ptr<astar::AStar::Setting>>(),
            py::arg("planning_interface").none(false),
            py::arg("setting") = nullptr)
        .def("plan", &astar::AStar::Plan);
}

static void
BindPlanningInterfaceMultiResolutions(const py::module &m) {
    py::class_<PlanningInterfaceMultiResolutions, std::shared_ptr<PlanningInterfaceMultiResolutions>>(m, "PlanningInterfaceMultiResolutions")
        .def(
            py::init<
                std::shared_ptr<EnvironmentMultiResolution>,
                std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>>,
                Eigen::VectorXd,
                const std::vector<Eigen::VectorXd>,
                std::vector<Eigen::VectorXd>,
                std::vector<double>>(),
            py::arg("environment_multi_resolution"),
            py::arg("heuristics"),
            py::arg("metric_start_coords"),
            py::arg("metric_goals_coords"),
            py::arg("metric_goals_tolerances"),
            py::arg("terminal_costs") = std::vector<double>{0.})
        .def(
            py::init<
                std::shared_ptr<EnvironmentMultiResolution>,
                std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>>,
                Eigen::VectorXd,
                Eigen::VectorXd,
                Eigen::VectorXd>(),
            py::arg("environment_multi_resolution"),
            py::arg("heuristics"),
            py::arg("metric_start_coords"),
            py::arg("metric_goal_coords"),
            py::arg("metric_goal_tolerance"))
        .def_property_readonly("num_heuristics", &PlanningInterfaceMultiResolutions::GetNumHeuristics)
        .def_property_readonly("num_resolution_levels", &PlanningInterfaceMultiResolutions::GetNumResolutionLevels)
        .def_property_readonly("start_state", &PlanningInterfaceMultiResolutions::GetStartState)
        .def_property_readonly("num_goals", &PlanningInterfaceMultiResolutions::GetNumGoals)
        .def("get_heuristic", &PlanningInterfaceMultiResolutions::GetHeuristic, py::arg("heuristic_id"))
        .def("get_resolution_assignment", &PlanningInterfaceMultiResolutions::GetResolutionAssignment, py::arg("heuristic_id"))
        .def("get_resolution_heuristic_ids", &PlanningInterfaceMultiResolutions::GetResolutionHeuristicIds, py::arg("resolution_level"))
        .def("get_in_resolution_level_flags", &PlanningInterfaceMultiResolutions::GetInResolutionLevelFlags, py::arg("env_state").none(false))
        .def("get_successors", &PlanningInterfaceMultiResolutions::GetSuccessors, py::arg("env_state").none(false), py::arg("env_resolution_level"))
        .def("get_heuristic_values", &PlanningInterfaceMultiResolutions::GetHeuristicValues, py::arg("env_state").none(false))
        .def("is_metric_goal", &PlanningInterfaceMultiResolutions::IsMetricGoal, py::arg("env_state").none(false))
        .def_static("is_virtual_goal", &PlanningInterfaceMultiResolutions::IsVirtualGoal, py::arg("env_state").none(false))
        .def("reach_goal", &PlanningInterfaceMultiResolutions::ReachGoal, py::arg("env_state").none(false))
        .def("state_hashing", &PlanningInterfaceMultiResolutions::StateHashing, py::arg("env_state").none(false))
        .def("get_path", &PlanningInterfaceMultiResolutions::GetPath, py::arg("env_state").none(false), py::arg("action_coords"));
}

static void
BindAmraStar(py::module &m) {
    const auto amra_star_module = m.def_submodule("amra_star");
    // Output
    py::class_<amra_star::Output, std::shared_ptr<amra_star::Output>>(amra_star_module, "Output")
        .def_readwrite("latest_plan_itr", &amra_star::Output::latest_plan_itr)
        .def_readwrite("goal_indices", &amra_star::Output::goal_indices)
        .def_readwrite("paths", &amra_star::Output::paths)
        .def_readwrite("actions_coords", &amra_star::Output::actions_coords)
        .def_readwrite("costs", &amra_star::Output::costs)
        .def_readwrite("num_heuristics", &amra_star::Output::num_heuristics)
        .def_readwrite("num_resolution_levels", &amra_star::Output::num_resolution_levels)
        .def_readwrite("num_expansions", &amra_star::Output::num_expansions)
        .def_readwrite("w1_solve", &amra_star::Output::w1_solve)
        .def_readwrite("w2_solve", &amra_star::Output::w2_solve)
        .def_readwrite("search_time", &amra_star::Output::search_time)
        .def_readwrite("w1_values", &amra_star::Output::w1_values)
        .def_readwrite("w2_values", &amra_star::Output::w2_values)
        .def_readwrite("opened_states", &amra_star::Output::opened_states)
        .def_readwrite("closed_states", &amra_star::Output::closed_states)
        .def_readwrite("inconsistent_states", &amra_star::Output::inconsistent_states)
        .def("save", &amra_star::Output::Save, py::arg("file_path").none(false));

    // AMRA*
    py::class_<amra_star::AmraStar> amra_star(amra_star_module, "AmraStar");
    py::class_<amra_star::AmraStar::Setting, YamlableBase, std::shared_ptr<amra_star::AmraStar::Setting>>(amra_star, "Setting")
        .def_readwrite("time_limit", &amra_star::AmraStar::Setting::time_limit)
        .def_readwrite("w1_init", &amra_star::AmraStar::Setting::w1_init)
        .def_readwrite("w2_init", &amra_star::AmraStar::Setting::w2_init)
        .def_readwrite("w1_final", &amra_star::AmraStar::Setting::w1_final)
        .def_readwrite("w2_final", &amra_star::AmraStar::Setting::w2_final)
        .def_readwrite("w1_decay_factor", &amra_star::AmraStar::Setting::w1_decay_factor)
        .def_readwrite("w2_decay_factor", &amra_star::AmraStar::Setting::w2_decay_factor)
        .def_readwrite("log", &amra_star::AmraStar::Setting::log);
    amra_star
        .def(
            py::init<std::shared_ptr<PlanningInterfaceMultiResolutions>, std::shared_ptr<amra_star::AmraStar::Setting>>(),
            py::arg("planning_interface").none(false),
            py::arg("setting") = nullptr)
        .def("plan", &amra_star::AmraStar::Plan);
}

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_search_planning";

    BindHeuristics(m);
    BindPlanningInterfaces(m);
    BindAStar(m);
    BindPlanningInterfaceMultiResolutions(m);
    BindAmraStar(m);
}
