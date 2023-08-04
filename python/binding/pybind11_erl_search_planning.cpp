#include "erl_search_planning/pybind11_erl_search_planning.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_search_planning/astar.hpp"
#include "erl_search_planning/planning_interface.hpp"
// #include "erl_search_planning/planning_2d.hpp"
// #include "erl_search_planning/planning_se2.hpp"
// #include "erl_search_planning/planning_grid_se2.hpp"

using namespace erl::common;
using namespace erl::search_planning;
using namespace erl::env;

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
        .def(py::init_alias<Eigen::VectorXd, Eigen::VectorXd, double>(), py::arg("goal"), py::arg("goal_tolerance"), py::arg("terminal_cost") = 0.0)
        .def("__call__", &HeuristicBase::operator());

    py::class_<EuclideanDistanceHeuristic, HeuristicBase, PyHeuristic<EuclideanDistanceHeuristic>, std::shared_ptr<EuclideanDistanceHeuristic>>(
        m,
        ERL_AS_STRING(EuclideanDistanceHeuristic))
        .def(py::init<Eigen::VectorXd, Eigen::VectorXd, double>(), py::arg("goal"), py::arg("goal_tolerance"), py::arg("terminal_cost") = 0.0);
    py::class_<ManhattanDistanceHeuristic, HeuristicBase, PyHeuristic<ManhattanDistanceHeuristic>, std::shared_ptr<ManhattanDistanceHeuristic>>(
        m,
        ERL_AS_STRING(ManhattanDistanceHeuristic))
        .def(py::init<Eigen::VectorXd, Eigen::VectorXd, double>(), py::arg("goal"), py::arg("goal_tolerance"), py::arg("terminal_cost") = 0.0);
    py::class_<DictionaryHeuristic, HeuristicBase, PyHeuristic<DictionaryHeuristic>, std::shared_ptr<DictionaryHeuristic>>(
        m,
        ERL_AS_STRING(DictionaryHeuristic))
        .def(
            py::init<const std::string &, std::function<long(const erl::env::EnvironmentState &)>, bool>(),
            py::arg("csv_path"),
            py::arg("state_hashing_func"),
            py::arg("assert_on_missing") = true);
    py::class_<MultiGoalsHeuristic, HeuristicBase, PyHeuristic<MultiGoalsHeuristic>, std::shared_ptr<MultiGoalsHeuristic>>(
        m,
        ERL_AS_STRING(MultiGoalsHeuristic))
        .def(py::init<std::vector<std::shared_ptr<HeuristicBase>>>(), py::arg("heuristics"));
}

static void
BindPlanningInterfaces(py::module &m) {
    py::class_<PlanningInterface, std::shared_ptr<PlanningInterface>>(m, ERL_AS_STRING(PlanningInterface))
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
            py::arg("metric_goals_tolerance"),
            py::arg("terminal_costs") = std::vector<double>({0.}),
            py::arg("heuristic") = nullptr)
        .def(
            py::init<std::shared_ptr<EnvironmentBase>, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>(),
            py::arg("env").none(false),
            py::arg("metric_start_coords"),
            py::arg("metric_goal_coords"),
            py::arg("metric_goal_tolerances"))
        .def_property_readonly("env", &PlanningInterface::GetEnvironment)
        .def("set_heuristic", &PlanningInterface::SetHeuristic, py::arg("heuristic"))
        .def("get_heuristic", &PlanningInterface::GetHeuristic, py::arg("state"))
        .def("get_successors", &PlanningInterface::GetSuccessors, py::arg("state"))
        .def_property_readonly("start_state", &PlanningInterface::GetStartState)
        .def("get_goal_state", &PlanningInterface::GetGoalState, py::arg("index"))
        .def_property_readonly("num_goals", &PlanningInterface::GetNumGoals)
        .def("is_metric_goal", &PlanningInterface::IsMetricGoal, py::arg("env_state"))
        .def("state_hashing", &PlanningInterface::StateHashing, py::arg("state"))
        .def("get_path", &PlanningInterface::GetPath, py::arg("state"), py::arg("action_index"));
}

static void
BindAStar(py::module &m) {

    auto astar = m.def_submodule("astar");
    // Output
    py::class_<astar::Output, std::shared_ptr<astar::Output>>(astar, "Output")
        .def_readwrite("path", &astar::Output::path)
        .def_readwrite("action_coords", &astar::Output::action_coords)
        .def_readwrite("cost", &astar::Output::cost)
        .def_readwrite("opened_list", &astar::Output::opened_list)
        .def_readwrite("closed_list", &astar::Output::closed_list)
        .def_readwrite("inconsistent_list", &astar::Output::inconsistent_list);

    // Astar
    py::class_<astar::AStar>(astar, "AStar")
        .def(
            py::init<std::shared_ptr<PlanningInterface>, std::shared_ptr<astar::AStar::Setting>>(),
            py::arg("planning_interface").none(false),
            py::arg("setting") = nullptr)
        .def("plan", &astar::AStar::Plan);
}

static void BindPlanningInterfaceMultiResolutions(py::module &m) {

}

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_search_planning";

    BindHeuristics(m);
    BindPlanningInterfaces(m);
    BindAStar(m);
}
