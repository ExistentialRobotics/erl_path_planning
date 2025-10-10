#include "erl_path_planning/astar.hpp"

namespace erl::path_planning::astar {
    template class AStar<float, 2>;
    template class AStar<double, 2>;
    template class AStar<float, 3>;
    template class AStar<double, 3>;
    template class AStar<float, 4>;
    template class AStar<double, 4>;
}  // namespace erl::path_planning::astar
