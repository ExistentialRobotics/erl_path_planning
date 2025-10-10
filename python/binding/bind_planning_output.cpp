#include "erl_path_planning/pybind11_planning_output.hpp"

void
BindPlanningOutput(py::module &m) {
    BindPlanRecordImpl<float, 2>(m, "PlanRecord2Df");
    BindPlanRecordImpl<double, 2>(m, "PlanRecord2Dd");
    BindPlanRecordImpl<float, 3>(m, "PlanRecord3Df");
    BindPlanRecordImpl<double, 3>(m, "PlanRecord3Dd");
    BindPlanRecordImpl<float, 4>(m, "PlanRecord4Df");
    BindPlanRecordImpl<double, 4>(m, "PlanRecord4Dd");

    BindPlanningOutputImpl<float, 2>(m, "PlanningOutput2Df");
    BindPlanningOutputImpl<double, 2>(m, "PlanningOutput2Dd");
    BindPlanningOutputImpl<float, 3>(m, "PlanningOutput3Df");
    BindPlanningOutputImpl<double, 3>(m, "PlanningOutput3Dd");
    BindPlanningOutputImpl<float, 4>(m, "PlanningOutput4Df");
    BindPlanningOutputImpl<double, 4>(m, "PlanningOutput4Dd");
}
