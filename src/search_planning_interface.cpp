#include "erl_path_planning/search_planning_interface.hpp"

namespace erl::path_planning {
    template class SearchPlanningInterface<float, 2>;
    template class SearchPlanningInterface<double, 2>;
    template class SearchPlanningInterface<float, 3>;
    template class SearchPlanningInterface<double, 3>;
    template class SearchPlanningInterface<float, 4>;
    template class SearchPlanningInterface<double, 4>;

    template class SearchPlanningInterfaceMultiResolutions<float, 2>;
    template class SearchPlanningInterfaceMultiResolutions<double, 2>;
    template class SearchPlanningInterfaceMultiResolutions<float, 3>;
    template class SearchPlanningInterfaceMultiResolutions<double, 3>;
    template class SearchPlanningInterfaceMultiResolutions<float, 4>;
    template class SearchPlanningInterfaceMultiResolutions<double, 4>;
}  // namespace erl::path_planning
