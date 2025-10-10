#pragma once

#include "amra_star.hpp"

#include "erl_common/pybind11.hpp"

template<typename Dtype, int Dim>
void
BindAmraStarImpl(py::module &m, const char *name) {
    using namespace erl::path_planning;

    using Output = amra_star::Output<Dtype, Dim>;
    using AmraStar = amra_star::AmraStar<Dtype, Dim>;
    using AmraStarSetting = amra_star::AmraStarSetting<Dtype>;
    using PlanningInterface = SearchPlanningInterfaceMultiResolutions<Dtype, Dim>;

    // AMRA*
    py::class_<AmraStar> amra_star(m, name);
    py::class_<AmraStarSetting, erl::common::YamlableBase, std::shared_ptr<AmraStarSetting>>(
        amra_star,
        "Setting")
        .def_readwrite("time_limit", &AmraStar::Setting::time_limit)
        .def_readwrite("w1_init", &AmraStar::Setting::w1_init)
        .def_readwrite("w2_init", &AmraStar::Setting::w2_init)
        .def_readwrite("w1_final", &AmraStar::Setting::w1_final)
        .def_readwrite("w2_final", &AmraStar::Setting::w2_final)
        .def_readwrite("w1_decay_factor", &AmraStar::Setting::w1_decay_factor)
        .def_readwrite("w2_decay_factor", &AmraStar::Setting::w2_decay_factor)
        .def_readwrite("log", &AmraStar::Setting::log);
    // Output
    py::class_<Output, PlanningOutput<Dtype, Dim>, std::shared_ptr<Output>>(amra_star, "Output")
        .def_readwrite("num_heuristics", &Output::num_heuristics)
        .def_readwrite("num_resolution_levels", &Output::num_resolution_levels)
        .def_readwrite("num_expansions", &Output::num_expansions)
        .def_readwrite("w1_solve", &Output::w1_solve)
        .def_readwrite("w2_solve", &Output::w2_solve)
        .def_readwrite("search_time", &Output::search_time)
        .def_readwrite("w1_values", &Output::w1_values)
        .def_readwrite("w2_values", &Output::w2_values)
        .def_readwrite("opened_states", &Output::opened_states)
        .def_readwrite("closed_states", &Output::closed_states)
        .def_readwrite("inconsistent_states", &Output::inconsistent_states)
        .def("save", &Output::Save, py::arg("file_path").none(false));
    amra_star
        .def(
            py::init<std::shared_ptr<PlanningInterface>, std::shared_ptr<AmraStarSetting>>(),
            py::arg("planning_interface").none(false),
            py::arg("setting") = nullptr)
        .def("plan", &AmraStar::Plan);
}
