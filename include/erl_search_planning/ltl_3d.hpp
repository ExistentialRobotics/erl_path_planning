#pragma once

#include <memory>
#include "erl_geometry/kdtree_eigen_adaptor.hpp"
#include "heuristic.hpp"

namespace erl::search_planning {

    struct LinearTemporalLogicHeuristic3D : public HeuristicBase {

        using KdTree = erl::geometry::KdTreeEigenAdaptor<double, 3>;
        std::shared_ptr<erl::env::FiniteStateAutomaton> fsa;
        std::vector<std::shared_ptr<KdTree>> label_to_kdtree;
        Eigen::MatrixXd label_distance;

        /**
         * @brief Construct a new Linear Temporal Logic Heuristic 3D object
         * @param fsa_in
         * @param label_maps_in
         * @param grid_map_info cell size of x, y and z axis
         */
        LinearTemporalLogicHeuristic3D(
            std::shared_ptr<erl::env::FiniteStateAutomaton> fsa_in,
            const std::unordered_map<int, Eigen::MatrixX<uint32_t>> &label_maps_in,
            const std::shared_ptr<erl::common::GridMapInfo3D> &grid_map_info);

        /**
         * @brief Compute the heuristic value of the given state
         * @param env_state metric state (x, y, z, q) and grid state (i, j, k, q)
         * @return
         */
        [[nodiscard]] double
        operator()(const env::EnvironmentState &env_state) const override;

    private:
        bool
        LoadFromCache(const std::filesystem::path &cache_dir);
        void
        SaveToCache(const std::filesystem::path &cache_dir) const;
    };

}  // namespace erl::search_planning
