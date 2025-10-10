#include "erl_common/pybind11.hpp"

void
BindHeuristics(py::module &m);

void
BindSearchPlanningInterface(py::module &m);

void
BindPlanningOutput(py::module &m);

void
BindAStar(py::module &m);

void
BindAmraStar(py::module &m);

PYBIND11_MODULE(PYBIND_MODULE_NAME, m) {
    m.doc() = "Python 3 Interface of erl_path_planning";

    BindHeuristics(m);

    BindSearchPlanningInterface(m);
    BindPlanningOutput(m);

    BindAStar(m);
    BindAmraStar(m);
}
