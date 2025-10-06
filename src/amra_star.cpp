#include "erl_search_planning/amra_star.hpp"

namespace erl::search_planning::amra_star {

    template class AmraStar<float, 2>;
    template class AmraStar<double, 2>;
    template class AmraStar<float, 3>;
    template class AmraStar<double, 3>;

}  // namespace erl::search_planning::amra_star
