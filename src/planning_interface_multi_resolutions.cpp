#include "erl_search_planning/planning_interface_multi_resolutions.hpp"

namespace erl::search_planning {

    template class PlanningInterfaceMultiResolutions<float, 2>;
    template class PlanningInterfaceMultiResolutions<double, 2>;
    template class PlanningInterfaceMultiResolutions<float, 3>;
    template class PlanningInterfaceMultiResolutions<double, 3>;
    template class PlanningInterfaceMultiResolutions<float, 4>;
    template class PlanningInterfaceMultiResolutions<double, 4>;

}  // namespace erl::search_planning
