#pragma once

#include "heuristic.hpp"

#include "erl_geometry/kdtree_eigen_adaptor.hpp"

#include <memory>

namespace erl::search_planning {

    struct LinearTemporalLogicHeuristic2D : public MultiGoalsHeuristic {

        std::shared_ptr<env::FiniteStateAutomaton> fsa = nullptr;
        std::vector<std::shared_ptr<geometry::KdTree2d>> label_to_kdtree = {};
        Eigen::MatrixXd label_distance = {};

        /**
         * @brief Construct a new Linear Temporal Logic Heuristic 3D object
         * @param fsa_in
         * @param label_map
         * @param grid_map_info cell size of label_maps_in
         */
        LinearTemporalLogicHeuristic2D(
            std::shared_ptr<env::FiniteStateAutomaton> fsa_in,
            const Eigen::Ref<const Eigen::MatrixX<uint32_t>> &label_map,
            const std::shared_ptr<common::GridMapInfo2D> &grid_map_info);

        /**
         * @brief Compute the heuristic value of the given state
         * @param env_state metric state (x, y, q) and grid state (i, j, q)
         * @return
         */
        [[nodiscard]] double
        operator()(const env::EnvironmentState &env_state) const override;
    };
}  // namespace erl::search_planning
