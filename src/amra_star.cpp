#include "erl_path_planning/amra_star.hpp"

namespace erl::path_planning::amra_star {

    template class AmraStar<float, 2>;
    template class AmraStar<double, 2>;
    template class AmraStar<float, 3>;
    template class AmraStar<double, 3>;

}  // namespace erl::path_planning::amra_star
