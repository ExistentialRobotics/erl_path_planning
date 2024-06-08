#pragma once

#include "heuristic.hpp"

#include "erl_env/environment_multi_resolution.hpp"

#include <vector>

namespace erl::search_planning {

    class PlanningInterfaceMultiResolutions {
    protected:
        /*
         * m_envs_ stores the environment instances for each resolution level. m_envs_[0] is the anchor-level
         * environment. Its state space is the union of state spaces of all other environments. And its action space is
         * the union of action spaces of all other environments. m_envs_[i] is the i-th resolution-level environment.
         */
        std::shared_ptr<env::EnvironmentMultiResolution> m_environment_multi_resolution_ = nullptr;
        std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> m_heuristics_ = {};  // heuristic and its resolution level
        std::vector<std::vector<std::size_t>> m_heuristic_ids_by_resolution_level_ = {};

        Eigen::VectorXd m_init_start_;                                 // used to store the initial start position
        std::shared_ptr<env::EnvironmentState> m_start_;               // used to store the start position in metric & grid space
        std::vector<std::shared_ptr<env::EnvironmentState>> m_goals_;  // used to store the goals in GetHeuristic and IsMetricGoal
        std::vector<Eigen::VectorXd> m_goals_tolerances_;              // used to store the goals tolerance in GetHeuristic and IsMetricGoal
        std::vector<double> m_terminal_costs_;                         // used to store the terminal costs in GetHeuristic
        std::vector<env::Successor> m_virtual_goal_successors_ = {};

    public:
        PlanningInterfaceMultiResolutions(
            std::shared_ptr<env::EnvironmentMultiResolution> environment_multi_resolution,
            std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics,
            Eigen::VectorXd metric_start_coords,
            const std::vector<Eigen::VectorXd> &metric_goals_coords,
            std::vector<Eigen::VectorXd> metric_goals_tolerances,
            std::vector<double> terminal_costs = std::vector<double>{0.0});

        PlanningInterfaceMultiResolutions(
            std::shared_ptr<env::EnvironmentMultiResolution> environment_multi_resolution,
            std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics,
            Eigen::VectorXd metric_start_coords,
            Eigen::VectorXd metric_goal_coords,
            Eigen::VectorXd metric_goal_tolerance)
            : PlanningInterfaceMultiResolutions(
                  std::move(environment_multi_resolution),
                  std::move(heuristics),
                  std::move(metric_start_coords),
                  std::vector({std::move(metric_goal_coords)}),
                  std::vector({std::move(metric_goal_tolerance)})) {}

        [[nodiscard]] std::size_t
        GetNumHeuristics() const {
            return m_heuristics_.size();
        }

        [[nodiscard]] std::size_t
        GetNumResolutionLevels() const {
            return m_environment_multi_resolution_->GetNumResolutionLevels();
        }

        [[nodiscard]] std::shared_ptr<HeuristicBase>
        GetHeuristic(const std::size_t heuristic_id) const {
            return m_heuristics_[heuristic_id].first;
        }

        [[nodiscard]] std::size_t
        GetResolutionAssignment(const std::size_t heuristic_id) const {
            return m_heuristics_[heuristic_id].second;
        }

        [[nodiscard]] const std::vector<std::size_t> &
        GetResolutionHeuristicIds(const std::size_t resolution_level) const {
            return m_heuristic_ids_by_resolution_level_[resolution_level];
        }

        [[nodiscard]] std::vector<bool>
        GetInResolutionLevelFlags(const std::shared_ptr<env::EnvironmentState> &env_state) const {
            const std::size_t num_resolution_levels = GetNumResolutionLevels();
            std::vector<bool> in_resolution_level_flags(num_resolution_levels, false);
            in_resolution_level_flags[0] = true;
            if (IsVirtualGoal(env_state)) { return in_resolution_level_flags; }
            for (std::size_t i = 1; i < num_resolution_levels; ++i) {
                in_resolution_level_flags[i] = m_environment_multi_resolution_->InStateSpaceAtLevel(env_state, i);
            }
            return in_resolution_level_flags;
        }

        [[nodiscard]] std::vector<env::Successor>
        GetSuccessors(const std::shared_ptr<env::EnvironmentState> &env_state, const std::size_t env_resolution_level) const {
            std::vector<env::Successor> successors = m_environment_multi_resolution_->GetSuccessorsAtLevel(env_state, env_resolution_level);
            if (const int goal_index = IsMetricGoal(env_state); goal_index >= 0) { successors.push_back(m_virtual_goal_successors_[goal_index]); }
            return successors;
        }

        [[nodiscard]] std::shared_ptr<env::EnvironmentState>
        GetStartState() const {
            return m_start_;
        }

        [[nodiscard]] std::shared_ptr<env::EnvironmentState>
        GetGoalState(const int index) const {
            return m_goals_[index];
        }

        [[nodiscard]] int
        GetNumGoals() const {
            return static_cast<int>(m_goals_.size());
        }

        [[nodiscard]] double
        GetTerminalCost(const int index) const {
            return m_terminal_costs_[index];
        }

        [[nodiscard]] std::vector<double>
        GetHeuristicValues(const std::shared_ptr<env::EnvironmentState> &env_state) const {
            std::vector<double> heuristic_values;
            const std::size_t num_heuristics = GetNumHeuristics();
            heuristic_values.resize(num_heuristics);
            for (std::size_t i = 0; i < num_heuristics; ++i) { heuristic_values[i] = (*m_heuristics_[i].first)(*env_state); }
            return heuristic_values;
        }

        [[nodiscard]] int  // return the index of the goal that is reached, -1 if none is reached
        IsMetricGoal(const std::shared_ptr<env::EnvironmentState> &env_state) const;

        [[nodiscard]] static bool
        IsVirtualGoal(const std::shared_ptr<env::EnvironmentState> &env_state) {
            return env_state->grid[0] == env::VirtualStateValue::kGoal;
        }

        [[nodiscard]] static bool
        ReachGoal(const std::shared_ptr<env::EnvironmentState> &env_state) {
            return IsVirtualGoal(env_state);  // only virtual goal can be reached
        }

        [[nodiscard]] long
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const {
            if (env_state->grid[0] == env::VirtualStateValue::kGoal) { return env::VirtualStateValue::kGoal; }  // virtual goal env_state
            // use the anchor-level environment to hash the state
            return m_environment_multi_resolution_->StateHashing(env_state);
        }

        [[nodiscard]] std::vector<std::shared_ptr<env::EnvironmentState>>
        GetPath(const std::shared_ptr<const env::EnvironmentState> &env_state, const std::vector<int> &action_coords) const {
            if (env_state->grid[0] == env::VirtualStateValue::kGoal) {
                ERL_DEBUG_ASSERT(action_coords[0] < 0, "virtual goal env_state can only be reached with action_coords[0] = -goal_index - 1.");
                return {m_goals_[-(action_coords[0] + 1)]};
            }
            return m_environment_multi_resolution_->ForwardAction(env_state, action_coords);
        }

    private:
        void
        SetStart() {
            m_start_ = std::make_shared<env::EnvironmentState>();
            m_start_->grid = m_environment_multi_resolution_->MetricToGrid(m_init_start_);
            m_start_->metric = m_environment_multi_resolution_->GridToMetric(m_start_->grid);
        }

        void
        SetGoals(const std::vector<Eigen::VectorXd> &metric_goals_coords) {
            m_goals_.reserve(metric_goals_coords.size());
            for (const auto &metric_goal_coords: metric_goals_coords) {
                auto goal = std::make_shared<env::EnvironmentState>();
                goal->metric = metric_goal_coords;
                goal->grid = m_environment_multi_resolution_->MetricToGrid(metric_goal_coords);
                m_goals_.push_back(goal);
            }
        }
    };
}  // namespace erl::search_planning
