#include "erl_path_planning/planning_output.hpp"

namespace erl::path_planning {
    template struct PlanRecord<float, 2>;
    template struct PlanRecord<double, 2>;
    template struct PlanRecord<float, 3>;
    template struct PlanRecord<double, 3>;
    template struct PlanRecord<float, 4>;
    template struct PlanRecord<double, 4>;

    template class PlanningOutput<float, 2>;
    template class PlanningOutput<double, 2>;
    template class PlanningOutput<float, 3>;
    template class PlanningOutput<double, 3>;
    template class PlanningOutput<float, 4>;
    template class PlanningOutput<double, 4>;
}  // namespace erl::path_planning
