#include "erl_path_planning/pybind11_search_planning_interface.hpp"

void
BindSearchPlanningInterface(py::module &m) {
    BindSearchPlanningInterfaceImpl<float, 2>(m, "SearchPlanningInterface2Df");
    BindSearchPlanningInterfaceImpl<double, 2>(m, "SearchPlanningInterface2Dd");
    BindSearchPlanningInterfaceImpl<float, 3>(m, "SearchPlanningInterface3Df");
    BindSearchPlanningInterfaceImpl<double, 3>(m, "SearchPlanningInterface3Dd");
    BindSearchPlanningInterfaceImpl<float, 4>(m, "SearchPlanningInterface4Df");
    BindSearchPlanningInterfaceImpl<double, 4>(m, "SearchPlanningInterface4Dd");

    BindSearchPlanningInterfaceMultiResolutionImpl<float, 2>(
        m,
        "SearchPlanningInterfaceMultiResolutions2Df");
    BindSearchPlanningInterfaceMultiResolutionImpl<double, 2>(
        m,
        "SearchPlanningInterfaceMultiResolutions2Dd");
    BindSearchPlanningInterfaceMultiResolutionImpl<float, 3>(
        m,
        "SearchPlanningInterfaceMultiResolutions3Df");
    BindSearchPlanningInterfaceMultiResolutionImpl<double, 3>(
        m,
        "SearchPlanningInterfaceMultiResolutions3Dd");
    BindSearchPlanningInterfaceMultiResolutionImpl<float, 4>(
        m,
        "SearchPlanningInterfaceMultiResolutions4Df");
    BindSearchPlanningInterfaceMultiResolutionImpl<double, 4>(
        m,
        "SearchPlanningInterfaceMultiResolutions4Dd");
}
