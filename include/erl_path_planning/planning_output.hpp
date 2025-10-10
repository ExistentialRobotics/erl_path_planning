#pragma once

#include "erl_common/eigen.hpp"

namespace erl::path_planning {
    template<typename Dtype, int Dim>
    struct PlanRecord {
        int goal_index = -1;
        Eigen::Matrix<Dtype, Dim, Eigen::Dynamic> path = {};
        std::vector<std::pair<long, long>> env_action_indices = {};  // (env_id, action_idx)
        Dtype cost = std::numeric_limits<Dtype>::infinity();
    };

    template<typename Dtype, int Dim>
    struct PlanningOutput {
        long latest_plan_itr = -1;  // latest plan iteration
        // plan_itr -> plan_record
        std::unordered_map<long, PlanRecord<Dtype, Dim>> plan_records = {};

        [[nodiscard]] const PlanRecord<Dtype, Dim>*
        GetLatestRecord() const {
            if (latest_plan_itr < 0) { return nullptr; }
            auto itr = plan_records.find(latest_plan_itr);
            if (itr == plan_records.end()) { return nullptr; }
            return &itr->second;
        }
    };

    extern template struct PlanRecord<float, 2>;
    extern template struct PlanRecord<double, 2>;
    extern template struct PlanRecord<float, 3>;
    extern template struct PlanRecord<double, 3>;
    extern template struct PlanRecord<float, 4>;
    extern template struct PlanRecord<double, 4>;

    extern template class PlanningOutput<float, 2>;
    extern template class PlanningOutput<double, 2>;
    extern template class PlanningOutput<float, 3>;
    extern template class PlanningOutput<double, 3>;
    extern template class PlanningOutput<float, 4>;
    extern template class PlanningOutput<double, 4>;
}  // namespace erl::path_planning
