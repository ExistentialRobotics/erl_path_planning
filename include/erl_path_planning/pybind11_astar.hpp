#pragma once

#include "astar.hpp"

#include "erl_common/pybind11.hpp"

template<typename Dtype, int Dim>
void
BindAStarImpl(py::module &m, const char *name) {
    using namespace erl::path_planning;

    using Output = astar::Output<Dtype, Dim>;
    using AStar = astar::AStar<Dtype, Dim>;
    using AStarSetting = astar::AstarSetting<Dtype>;
    using PlanningInterface = SearchPlanningInterface<Dtype, Dim>;

    // Astar
    py::class_<AStar> astar(m, name);
    // Output
    py::class_<Output, PlanningOutput<Dtype, Dim>, std::shared_ptr<Output>>(astar, "Output")
        .def_readwrite("opened_list", &Output::opened_list)
        .def_readwrite("closed_list", &Output::closed_list)
        .def_readwrite("inconsistent_list", &Output::inconsistent_list);

    py::class_<AStarSetting, erl::common::YamlableBase, std::shared_ptr<AStarSetting>>(
        astar,
        "Setting",
        py::module_local())
        .def(py::init<>())
        .def_readwrite("eps", &AStarSetting::eps)
        .def_readwrite("max_num_iterations", &AStarSetting::max_num_iterations)
        .def_readwrite("log", &AStarSetting::log)
        .def_readwrite("reopen_inconsistent", &AStarSetting::reopen_inconsistent);
    astar
        .def(
            py::init<std::shared_ptr<PlanningInterface>, std::shared_ptr<AStarSetting>>(),
            py::arg("planning_interface").none(false),
            py::arg("setting") = nullptr)
        .def("plan", &AStar::Plan);
}
