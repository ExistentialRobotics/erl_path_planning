#pragma once

#include "erl_env/environment_base.hpp"
#include "heuristic.hpp"

#include <map>
#include <vector>

namespace erl::search_planning {

    class PlanningInterfaceMultiResolutions {
    protected:
        std::vector<std::shared_ptr<env::EnvironmentBase>> m_envs_ = {};
        std::vector<std::pair<std::shared_ptr<HeuristicBase>, uint8_t>> m_heuristics_ = {};  // heuristic and its resolution level
        std::vector<std::vector<std::size_t>> m_heuristic_ids_by_resolution_level_ = {};

        Eigen::VectorXd m_init_start_;                     // used to store the initial start position
        Eigen::VectorXi m_grid_start_;                     // used to store the start position in grid space
        Eigen::VectorXd m_metric_start_;                   // used to store the start position in metric space
        std::vector<Eigen::VectorXd> m_goals_;             // used to store the goals in ComputeHeuristic and IsGoal
        std::vector<Eigen::VectorXd> m_goals_tolerances_;  // used to store the goals tolerance in ComputeHeuristic and IsGoal
        Eigen::VectorXd m_terminal_costs_;                 // used to store the terminal costs in ComputeHeuristic
        bool m_terminal_cost_set_ = false;

    public:
        /**
         *
         * @param environments environment instances for each resolution level
         * @param heuristics instances of heuristics
         * @param heuristic_assignments resolution level that each heuristic is assigned to
         */
        PlanningInterfaceMultiResolutions(
            std::vector<std::shared_ptr<env::EnvironmentBase>> environments,
            std::vector<std::pair<std::shared_ptr<HeuristicBase>, uint8_t>> heuristics,
            Eigen::VectorXd metric_start_coords,
            std::vector<Eigen::VectorXd> metric_goals_coords,
            std::vector<Eigen::VectorXd> metric_goals_tolerance)
            : m_envs_(std::move(environments)),
              m_heuristics_(std::move(heuristics)),
              m_metric_start_(std::move(metric_start_coords)),
              m_goals_(std::move(metric_goals_coords)),
              m_goals_tolerances_(std::move(metric_goals_tolerance)) {

            std::size_t num_resolution_levels = m_envs_.size();
            // check if all environments are valid
            for (std::size_t i = 0; i < num_resolution_levels; ++i) { ERL_ASSERTM(m_envs_[i] != nullptr, "environment %d is nullptr.", int(i)); }

            // check goals and goals tolerances
            int num_goals = GetNumGoals();
            ERL_ASSERTM(num_goals > 0, "at least one goal must be provided.");
            if (num_goals > 1) {
                if (m_goals_tolerances_.size() == 1) {
                    ERL_INFO("only one goal tolerance is provided, copying it to all goals.");
                    m_goals_tolerances_.resize(num_goals, m_goals_tolerances_[0]);
                } else {
                    ERL_FATAL("number of goal tolerances must be 1 or equal to the number of goals.");
                }
            }

            // check if each resolution level has at least one heuristic
            std::size_t num_heuristics = m_heuristics_.size();
            m_heuristic_ids_by_resolution_level_.clear();
            m_heuristic_ids_by_resolution_level_.resize(num_resolution_levels);
            for (std::size_t i = 0; i < num_heuristics; ++i) { m_heuristic_ids_by_resolution_level_[m_heuristics_[i].second].push_back(i); }
            for (std::size_t i = 0; i < num_resolution_levels; ++i) {
                ERL_ASSERTM(!m_heuristic_ids_by_resolution_level_[i].empty(), "resolution level %d has no heuristic.", int(i));
            }
        }

        [[nodiscard]] inline int
        GetNumGoals() const {
            return int(m_goals_.size());
        }

        [[nodiscard]] std::vector<env::Successor>
        GetSuccessors(const std::shared_ptr<const env::EnvironmentState> &state, uint8_t resolution_level) const {
            return m_envs_[resolution_level]->GetSuccessors(state);
        }
    };
}  // namespace erl::search_planning
