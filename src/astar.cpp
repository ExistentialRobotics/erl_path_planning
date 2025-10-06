#include "erl_search_planning/astar.hpp"

namespace erl::search_planning::astar {
    template class AStar<float, 2>;
    template class AStar<double, 2>;
    template class AStar<float, 3>;
    template class AStar<double, 3>;
    template class AStar<float, 4>;
    template class AStar<double, 4>;
}  // namespace erl::search_planning::astar
