#include "erl_path_planning/pybind11_astar.hpp"

void
BindAStar(py::module &m) {
    py::module astar_module = m.def_submodule("astar");
    BindAStarImpl<float, 2>(astar_module, "AStar2Df");
    BindAStarImpl<float, 3>(astar_module, "AStar3Df");
    BindAStarImpl<double, 2>(astar_module, "AStar2Dd");
    BindAStarImpl<double, 3>(astar_module, "AStar3Dd");
    BindAStarImpl<float, 4>(astar_module, "AStar4Df");
    BindAStarImpl<double, 4>(astar_module, "AStar4Dd");
}
