#pragma once

#include "erl_common/grid_map_info.hpp"
#include "environment_anchor.hpp"

namespace erl::search_planning {

    template<int Dim>
    class EnvironmentGridAnchor : public EnvironmentAnchor {
        std::shared_ptr<common::GridMapInfo<Dim>> m_grid_map_info_;

    public:
        EnvironmentGridAnchor(std::vector<std::shared_ptr<env::EnvironmentBase>> environments, std::shared_ptr<common::GridMapInfo<Dim>> grid_map_info)
            : EnvironmentAnchor(std::move(environments)),
              m_grid_map_info_(std::move(grid_map_info)) {}

        [[nodiscard]] inline uint32_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &state) const override {
            ERL_DEBUG_ASSERT(state->metric.size() == Dim, "state dimension is not equal to grid map dimension.");
            return m_grid_map_info_->GridToIndex(state->grid, true);  // row major
        }

        [[nodiscard]] inline Eigen::VectorXi
        MetricToGrid(const Eigen::Ref<const Eigen::VectorXd> &metric_state) const override {
            Eigen::Vector<int, Dim> grid;
            for (int i = 0; i < Dim; ++i) { grid[i] = m_grid_map_info_->MeterToGridForValue(metric_state[i], i); }
            return grid;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GridToMetric(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const override {
            Eigen::Vector<double, Dim> metric;
            for (int i = 0; i < Dim; ++i) { metric[i] = m_grid_map_info_->GridToMeterForValue(grid_state[i], i); }
            return metric;
        }
    };
}  // namespace erl::search_planning
