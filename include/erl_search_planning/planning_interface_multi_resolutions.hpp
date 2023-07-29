#pragma once

#include "erl_env/environment_base.hpp"
#include "environment_anchor.hpp"
#include "heuristic.hpp"

#include <map>
#include <vector>

namespace erl::search_planning {

    class PlanningInterfaceMultiResolutions {
    protected:
        /*
         * m_envs_ stores the environment instances for each resolution level. m_envs_[0] is the anchor-level
         * environment. Its state space is the union of state spaces of all other environments. And its action space is
         * the union of action spaces of all other environments. m_envs_[i] is the i-th resolution-level environment.
         */
        std::vector<std::shared_ptr<env::EnvironmentBase>> m_envs_ = {};
        std::shared_ptr<EnvironmentAnchor> m_env_anchor_ = nullptr;
        // heuristic and its resolution level
        std::vector<std::pair<std::shared_ptr<HeuristicBase>, uint8_t>> m_heuristics_ = {};
        std::vector<std::vector<std::size_t>> m_heuristic_ids_by_resolution_level_ = {};

        Eigen::VectorXd m_init_start_;                                 // used to store the initial start position
        std::shared_ptr<env::EnvironmentState> m_start_;               // used to store the start position in metric & grid space
        std::vector<std::shared_ptr<env::EnvironmentState>> m_goals_;  // used to store the goals in GetHeuristic and IsMetricGoal
        std::vector<Eigen::VectorXd> m_goals_tolerances_;              // used to store the goals tolerance in GetHeuristic and IsMetricGoal
        Eigen::VectorXd m_terminal_costs_;                             // used to store the terminal costs in GetHeuristic
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
            std::vector<Eigen::VectorXd> metric_goals_tolerances)
            : m_envs_(std::move(environments)),
              m_heuristics_(std::move(heuristics)),
              m_init_start_(std::move(metric_start_coords)),
              m_goals_tolerances_(std::move(metric_goals_tolerances)),
              m_terminal_cost_set_(false) {

            std::size_t num_resolution_levels = m_envs_.size();
            ERL_ASSERTM(num_resolution_levels > 0, "at least one environment must be provided.");
            // check if all environments are valid
            m_env_anchor_ = std::dynamic_pointer_cast<EnvironmentAnchor>(m_envs_[0]);
            ERL_ASSERTM(m_env_anchor_ != nullptr, "the anchor-level environment must be derived from EnvironmentAnchor.");
            for (std::size_t i = 1; i < num_resolution_levels; ++i) { ERL_ASSERTM(m_envs_[i] != nullptr, "environment %d is nullptr.", int(i)); }

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
                ERL_WARN(
                    "multiple goals are provided, please use %s or %s if necessary.",
                    ERL_AS_STRING(MultiGoalsHeuristic),
                    ERL_AS_STRING(MultiGoalsWithCostHeuristic));
            }

            SetStart();
            SetGoals(metric_goals_coords);

            // check if each resolution level has at least one heuristic
            std::size_t num_heuristics = m_heuristics_.size();
            m_heuristic_ids_by_resolution_level_.clear();
            m_heuristic_ids_by_resolution_level_.resize(num_resolution_levels);
            for (std::size_t i = 0; i < num_heuristics; ++i) {
                ERL_ASSERTM(m_heuristics_[i].first != nullptr, "heuristic %d is nullptr.", int(i));
                m_heuristic_ids_by_resolution_level_[m_heuristics_[i].second].push_back(i);
            }
            for (std::size_t i = 0; i < num_resolution_levels; ++i) {
                ERL_ASSERTM(!m_heuristic_ids_by_resolution_level_[i].empty(), "resolution level %d has no heuristic.", int(i));
            }
            ERL_ASSERTM(m_heuristics_[0].second == 0, "the first heuristic must be assigned to the anchor level (resolution level 0).");

            // miscellaneous
            m_terminal_costs_.setZero(num_goals);
        }

        PlanningInterfaceMultiResolutions(
            std::vector<std::shared_ptr<env::EnvironmentBase>> environments,
            std::vector<std::pair<std::shared_ptr<HeuristicBase>, uint8_t>> heuristics,
            Eigen::VectorXd metric_start_coords,
            std::vector<Eigen::VectorXd> metric_goals_coords,
            std::vector<Eigen::VectorXd> metric_goals_tolerances,
            Eigen::VectorXd terminal_costs)
            : PlanningInterfaceMultiResolutions(
                  std::move(environments),
                  std::move(heuristics),
                  std::move(metric_start_coords),
                  std::move(metric_goals_coords),
                  std::move(metric_goals_tolerances)) {
            m_terminal_costs_ = std::move(terminal_costs);
            m_terminal_cost_set_ = true;
        }

        [[nodiscard]] inline std::size_t
        GetNumHeuristics() const {
            return m_heuristics_.size();
        }

        [[nodiscard]] inline std::size_t
        GetNumResolutionLevels() const {
            return m_envs_.size() - 1;
        }

        [[nodiscard]] inline std::shared_ptr<HeuristicBase>
        GetHeuristic(std::size_t heuristic_id) const {
            return m_heuristics_[heuristic_id].first;
        }

        [[nodiscard]] inline uint8_t
        GetResolutionAssignment(std::size_t heuristic_id) const {
            return m_heuristics_[heuristic_id].second;
        }

        [[nodiscard]] inline std::vector<std::size_t>
        GetResolutionHeuristicIds(uint8_t resolution_level) const {
            return m_heuristic_ids_by_resolution_level_[resolution_level];
        }

        [[nodiscard]] inline std::vector<uint8_t>
        GetContainedResolutionLevels(const std::shared_ptr<env::EnvironmentState> &state) const {
            std::vector<uint8_t> contained_resolution_levels;
            for (std::size_t i = 1; i < m_envs_.size(); ++i) {
                if (m_envs_[i]->InStateSpace(state)) { contained_resolution_levels.push_back(i); }
            }
            return contained_resolution_levels;
        }

        [[nodiscard]] inline std::vector<env::Successor>
        GetSuccessors(const std::shared_ptr<env::EnvironmentState> &state, uint8_t env_resolution_level) const {
            return m_env_anchor_->GetSuccessors(state, env_resolution_level);
        }

        [[nodiscard]] inline std::shared_ptr<env::EnvironmentState>
        GetStartState() const {
            return m_start_;
        }

        [[nodiscard]] inline std::shared_ptr<env::EnvironmentState>
        GetGoalState(int index) const {
            return m_goals_[index];
        }

        [[nodiscard]] inline int
        GetNumGoals() const {
            return int(m_goals_.size()) - m_terminal_cost_set_;
        }

        [[nodiscard]] inline std::vector<double>
        GetHeuristicValues(const std::shared_ptr<env::EnvironmentState> &state) const {
            std::vector<double> heuristic_values;
            std::size_t num_heuristics = GetNumHeuristics();
            heuristic_values.resize(num_heuristics);
            for (std::size_t i = 0; i < num_heuristics; ++i) { heuristic_values[i] = (*m_heuristics_[i].first)(*state); }
            return heuristic_values;
        }

        [[nodiscard]] int  // return the index of the goal that is reached, -1 if none is reached
        IsMetricGoal(const std::shared_ptr<env::EnvironmentState> &state) const {
            int num_goals = GetNumGoals();
            long dim = state->metric.size();
            for (int i = 0; i < num_goals; ++i) {
                if (m_terminal_cost_set_ && (m_terminal_costs_[i] >= std::numeric_limits<double>::max())) { continue; }
                bool goal_reached = true;
                for (long j = 0; j < dim; ++j) {
                    double err = std::abs(state->metric[j] - m_goals_[i]->metric[j]);
                    if (err > m_goals_tolerances_[i][j]) {
                        goal_reached = false;
                        break;
                    }
                }
                if (goal_reached) { return i; }
            }
            return -1;
        }

        [[nodiscard]] static inline bool
        IsVirtualGoal(const std::shared_ptr<env::EnvironmentState> &env_state) {
            return env_state->grid[0] == env::VirtualStateValue::kGoal;
        }

        [[nodiscard]] inline int
        ReachGoal(const std::shared_ptr<env::EnvironmentState> &env_state) const {
            auto num_goals = int(m_goals_.size());
            if (num_goals == 1 || !m_terminal_cost_set_) { return IsMetricGoal(env_state); }
            // When terminal cost is set, the virtual goal is used to check if the goal is reached, we should not return other goal indices.
            return env_state->grid[0] == env::VirtualStateValue::kGoal ? num_goals - 1 : -1;
        }

        [[nodiscard]] inline long
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const {
            if (env_state->grid[0] == env::VirtualStateValue::kGoal) { return env::VirtualStateValue::kGoal; }  // virtual goal env_state
            // use the anchor-level environment to hash the state
            return m_env_anchor_->StateHashing(env_state);
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<env::EnvironmentState>>
        GetPath(const std::shared_ptr<const env::EnvironmentState> &env_state, const std::vector<int> &action_coords) const {
            auto num_goals = int(m_goals_.size());
            if (num_goals == 1 || !m_terminal_cost_set_) { return m_env_anchor_->ForwardAction(env_state, action_coords); }
            if (env_state->grid[0] == env::VirtualStateValue::kGoal) {
                ERL_DEBUG_ASSERT(action_index < 0, "virtual goal env_state can only be reached with action_index = -goal_index - 1.");
                return {m_goals_[-(action_coords[0] + 1)]};
            }
            return m_env_anchor_->ForwardAction(env_state, action_coords);
        }

    private:
        inline void
        SetStart() {
            m_start_ = std::make_shared<env::EnvironmentState>();
            m_start_->grid = m_env_anchor_->MetricToGrid(m_init_start_);
            m_start_->metric = m_env_anchor_->GridToMetric(m_start_->grid);
        }

        inline void
        SetGoals(const std::vector<Eigen::VectorXd> &metric_goals_coords) {
            auto num_goals = int(metric_goals_coords.size());
            if (m_terminal_cost_set_) {
                m_goals_.reserve(num_goals + 1);
                for (const auto &metric_goal_coords: metric_goals_coords) {
                    auto goal = std::make_shared<env::EnvironmentState>();
                    goal->metric = metric_goal_coords;
                    goal->grid = m_env_anchor_->MetricToGrid(metric_goal_coords);
                    m_goals_.push_back(goal);
                }
                // virtual goal
                auto goal = std::make_shared<env::EnvironmentState>();
                goal->grid.resize(2);
                goal->grid[0] = env::VirtualStateValue::kGoal;
                m_goals_.push_back(goal);
            } else {
                m_goals_.reserve(num_goals);
                for (const auto &metric_goal_coords: metric_goals_coords) {
                    auto goal = std::make_shared<env::EnvironmentState>();
                    goal->metric = metric_goal_coords;
                    goal->grid = m_env_anchor_->MetricToGrid(metric_goal_coords);
                    m_goals_.push_back(goal);
                }
            }
        }
    };
}  // namespace erl::search_planning
