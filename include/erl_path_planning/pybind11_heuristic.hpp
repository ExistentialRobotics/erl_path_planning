#pragma once

#include "heuristic.hpp"
#include "llm_scene_graph_heuristic.hpp"
#include "ltl_2d_heuristic.hpp"
#include "ltl_3d_heuristic.hpp"

#include "erl_common/pybind11.hpp"

template<typename Dtype, int Dim>
void
BindHeuristicBaseImpl(py::module &m, const char *name) {
    using Heuristic = erl::path_planning::HeuristicBase<Dtype, Dim>;
    py::class_<Heuristic, std::shared_ptr<Heuristic>>(m, name).def(
        "__call__",
        &Heuristic::operator(),
        py::arg("env_state"));
}

template<typename Dtype, int Dim>
void
BindEuclideanDistanceHeuristicImpl(py::module &m, const char *name) {
    using Heuristic = erl::path_planning::EuclideanDistanceHeuristic<Dtype, Dim>;
    py::class_<
        Heuristic,
        erl::path_planning::HeuristicBase<Dtype, Dim>,
        std::shared_ptr<Heuristic>>(m, name)
        .def(
            py::init<
                typename Heuristic::MetricState,
                typename Heuristic::MetricState,
                const Dtype>(),
            py::arg("goal"),
            py::arg("goal_tolerance"),
            py::arg("terminal_cost") = static_cast<Dtype>(0.0));
}

template<typename Dtype>
void
BindSe2HeuristicImpl(py::module &m, const char *name) {
    using Heuristic = erl::path_planning::Se2Heuristic<Dtype>;
    py::class_<Heuristic, erl::path_planning::HeuristicBase<Dtype, 3>, std::shared_ptr<Heuristic>>(
        m,
        name)
        .def(
            py::init<
                typename Heuristic::MetricState,
                typename Heuristic::MetricState,
                Dtype,
                Dtype>(),
            py::arg("goal"),
            py::arg("goal_tolerance"),
            py::arg("terminal_cost") = static_cast<Dtype>(0.0),
            py::arg("w_theta") = static_cast<Dtype>(0.0))
        .def_readwrite("w_theta", &Heuristic::w_theta);
}

template<typename Dtype, int Dim>
void
BindManhattanDistanceHeuristicImpl(py::module &m, const char *name) {
    using Heuristic = erl::path_planning::ManhattanDistanceHeuristic<Dtype, Dim>;
    py::class_<
        Heuristic,
        erl::path_planning::HeuristicBase<Dtype, Dim>,
        std::shared_ptr<Heuristic>>(m, name)
        .def(
            py::init<
                typename Heuristic::MetricState,
                typename Heuristic::MetricState,
                const Dtype>(),
            py::arg("goal"),
            py::arg("goal_tolerance"),
            py::arg("terminal_cost") = static_cast<Dtype>(0.0));
}

template<typename Dtype, int Dim>
void
BindDictionaryHeuristicImpl(py::module &m, const char *name) {
    using Heuristic = erl::path_planning::DictionaryHeuristic<Dtype, Dim>;
    py::class_<
        Heuristic,
        erl::path_planning::HeuristicBase<Dtype, Dim>,
        std::shared_ptr<Heuristic>>(m, name)
        .def(
            py::init<
                const std::string &,
                std::function<long(const erl::env::EnvironmentState<Dtype, Dim> &)>,
                bool>(),
            py::arg("csv_path"),
            py::arg("state_hashing_func"),
            py::arg("assert_on_missing") = true);
}

template<typename Dtype, int Dim>
void
BindMultiGoalsHeuristicImpl(py::module &m, const char *name) {
    using Heuristic = erl::path_planning::MultiGoalsHeuristic<Dtype, Dim>;
    py::class_<
        Heuristic,
        erl::path_planning::HeuristicBase<Dtype, Dim>,
        std::shared_ptr<Heuristic>>(m, name)
        .def(
            py::init<std::vector<std::shared_ptr<erl::path_planning::HeuristicBase<Dtype, Dim>>>>(),
            py::arg("heuristics"))
        .def_readonly("heuristics", &Heuristic::heuristics);
}

template<typename Dtype>
void
BindLTL2DHeuristicImpl(py::module &m, const char *name) {
    using Heuristic = erl::path_planning::LinearTemporalLogicHeuristic2D<Dtype>;
    using GridMapInfo = typename Heuristic::GridMapInfo;

    py::class_<
        Heuristic,
        erl::path_planning::MultiGoalsHeuristic<Dtype, 3>,
        std::shared_ptr<Heuristic>>(m, name)
        .def(
            py::init<
                const std::shared_ptr<erl::env::FiniteStateAutomaton> &,
                const Eigen::Ref<const Eigen::MatrixX<uint32_t>> &,
                const std::shared_ptr<GridMapInfo> &>(),
            py::arg("fsa_in"),
            py::arg("label_map"),
            py::arg("grid_map_info"))
        .def_readonly("fsa", &Heuristic::fsa)
        .def_readonly("label_distance", &Heuristic::label_distance);
}

template<typename Dtype>
void
BindLTL3DHeuristicImpl(py::module &m, const char *name) {
    using Heuristic = erl::path_planning::LinearTemporalLogicHeuristic3D<Dtype>;
    using GridMapInfo = typename Heuristic::GridMapInfo;

    py::class_<
        Heuristic,
        erl::path_planning::MultiGoalsHeuristic<Dtype, 4>,
        std::shared_ptr<Heuristic>>(m, name)
        .def(
            py::init<
                const std::shared_ptr<erl::env::FiniteStateAutomaton> &,
                const absl::flat_hash_map<int, Eigen::MatrixX<uint32_t>> &,
                const std::shared_ptr<GridMapInfo> &>(),
            py::arg("fsa_in"),
            py::arg("label_maps"),
            py::arg("grid_map_info"))
        .def_readonly("fsa", &Heuristic::fsa)
        .def_readonly("label_distance", &Heuristic::label_distance);
}

// template<typename Dtype>
// void
// BindLlmSceneGraphHeuristicImpl(py::module &m, const char *name) {
//     using Heuristic = erl::path_planning::LlmSceneGraphHeuristic<Dtype>;
//     using LlmWaypoint = typename Heuristic::LlmWaypoint;
//     using Setting = typename Heuristic::Setting;
//
//     py::class_<LlmWaypoint>(m, "LlmWaypoint")
//         .def(py::init<>())
//         .def_readwrite("type", &LlmWaypoint::type)
//         .def_readwrite("uuid1", &LlmWaypoint::uuid1)
//         .def_readwrite("uuid2", &LlmWaypoint::uuid2);
//
//     py::class_<Heuristic, erl::path_planning::HeuristicBase<Dtype, 4>,
//     std::shared_ptr<Heuristic>>
//         heuristic(m, name);
//
//     py::class_<Setting, erl::common::Yamlable<Setting>, std::shared_ptr<Setting>>(
//         heuristic,
//         "Setting")
//         .def(py::init<>())
//         .def_readwrite("llm_paths", &Setting::llm_paths);
//
//     heuristic
//         .def(
//             py::init<
//                 const std::shared_ptr<erl::env::EnvironmentLTLSceneGraph<Dtype>> &,
//                 const std::shared_ptr<Setting> &,
//                 const std::shared_ptr<typename Heuristic::GridMapInfo> &>(),
//             py::arg("env"),
//             py::arg("setting"),
//             py::arg("grid_map_info"))
//         .def_readonly("fsa", &Heuristic::fsa)
//         .def_readonly("label_distance", &Heuristic::label_distance)
//         .def_readonly("llm_paths", &Heuristic::llm_paths);
// }
