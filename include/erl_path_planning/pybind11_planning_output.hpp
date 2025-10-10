#pragma once

#include "planning_output.hpp"

#include "erl_common/pybind11.hpp"

template<typename Dtype, int Dim>
void
BindPlanRecordImpl(py::module &m, const char *name) {
    using PlanRecord = erl::path_planning::PlanRecord<Dtype, Dim>;
    py::class_<PlanRecord>(m, name)
        .def_readonly("goal_index", &PlanRecord::goal_index)
        .def_readonly("path", &PlanRecord::path)
        .def_property_readonly(
            "env_action_indices",
            [](PlanRecord &p) {
                Eigen::Map<Eigen::Matrix2Xl> indices_map(
                    reinterpret_cast<long *>(p.env_action_indices.data()),
                    2,
                    p.env_action_indices.size());
                return indices_map;
            })
        .def_readonly("cost", &PlanRecord::cost);
}

template<typename Dtype, int Dim>
void
BindPlanningOutputImpl(py::module &m, const char *name) {
    using PlanningOutput = erl::path_planning::PlanningOutput<Dtype, Dim>;
    py::class_<PlanningOutput, std::shared_ptr<PlanningOutput>>(m, name)
        .def_readonly("latest_plan_itr", &PlanningOutput::latest_plan_itr)
        .def_property_readonly(
            "plan_records",
            [](PlanningOutput &p) {
                py::list plan_records;
                for (auto &kv: p.plan_records) {
                    plan_records.append(py::make_tuple(kv.first, kv.second));
                }
                return plan_records;
            })
        .def_property_readonly("get_latest_record", [](PlanningOutput &output) -> py::object {
            auto plan_record = output.GetLatestRecord();
            if (plan_record == nullptr) { return py::none(); }
            return py::cast(*plan_record);
        });
}
