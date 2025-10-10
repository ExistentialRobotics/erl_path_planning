#include "erl_path_planning/ltl_3d_heuristic.hpp"

namespace erl::path_planning {
    template class LinearTemporalLogicHeuristic3D<float>;
    template class LinearTemporalLogicHeuristic3D<double>;
}  // namespace erl::path_planning
