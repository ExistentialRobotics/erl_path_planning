#include "erl_path_planning/pybind11_amra_star.hpp"

void
BindAmraStar(py::module &m) {
    py::module amra_star_module = m.def_submodule("amra_star");
    BindAmraStarImpl<float, 2>(amra_star_module, "AmraStar2Df");
    BindAmraStarImpl<float, 3>(amra_star_module, "AmraStar3Df");
    BindAmraStarImpl<double, 2>(amra_star_module, "AmraStar2Dd");
    BindAmraStarImpl<double, 3>(amra_star_module, "AmraStar3Dd");
    BindAmraStarImpl<float, 4>(amra_star_module, "AmraStar4Df");
    BindAmraStarImpl<double, 4>(amra_star_module, "AmraStar4Dd");
}
