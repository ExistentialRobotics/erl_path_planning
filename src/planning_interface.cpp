#include "erl_search_planning/planning_interface.hpp"

namespace erl::search_planning {
    template class PlanningInterface<float, 2>;
    template class PlanningInterface<double, 2>;
    template class PlanningInterface<float, 3>;
    template class PlanningInterface<double, 3>;
    template class PlanningInterface<float, 4>;
    template class PlanningInterface<double, 4>;
}  // namespace erl::search_planning
