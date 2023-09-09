#pragma once

#include <memory>
#include "heuristic.hpp"

namespace erl::search_planning {

    struct LinearTemporalLogicHeuristic2D : public HeuristicBase {

        std::shared_ptr<erl::env::FiniteStateAutomaton> fsa;
        std::unordered_map<uint32_t, std::vector<Eigen::Vector2d>> label_to_metric_states;
        Eigen::MatrixXd label_distance;

        /**
         * @brief Construct a new Linear Temporal Logic Heuristic 3D object
         * @param fsa_in
         * @param label_maps_in
         * @param grid_map_info cell size of label_maps_in
         */
        LinearTemporalLogicHeuristic2D(
            std::shared_ptr<erl::env::FiniteStateAutomaton> fsa_in,
            const Eigen::Ref<const Eigen::MatrixX<uint64_t>> &label_map,
            const std::shared_ptr<erl::common::GridMapInfo2D> &grid_map_info);

        /**
         * @brief Compute the heuristic value of the given state
         * @param state metric state (x, y, q) and grid state (i, j, q)
         * @return
         */
        [[nodiscard]] double
        operator()(const env::EnvironmentState &state) const override;
    };
}  // namespace erl::search_planning
