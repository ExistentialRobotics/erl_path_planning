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
         * m_envs_ stores the environment instances for each resolution level. m_envs_[0] is the anchor-level environment. Its state space is the union of
         * state spaces of all other environments. And its action space is the union of action spaces of all other environments. m_envs_[i] is the i-th
         * resolution-level environment.
         */
        std::vector<std::shared_ptr<env::EnvironmentBase>> m_envs_ = {nullptr};
        std::vector<std::pair<std::shared_ptr<HeuristicBase>, uint8_t>> m_heuristics_ = {};  // heuristic and its resolution level
        std::vector<std::vector<std::size_t>> m_heuristic_ids_by_resolution_level_ = {};

        Eigen::VectorXd m_init_start_;                     // used to store the initial start position
        Eigen::VectorXi m_grid_start_;                     // used to store the start position in grid space
        Eigen::VectorXd m_metric_start_;                   // used to store the start position in metric space
        std::vector<Eigen::VectorXd> m_goals_;             // used to store the goals in ComputeHeuristic and IsMetricGoal
        std::vector<Eigen::VectorXd> m_goals_tolerances_;  // used to store the goals tolerance in ComputeHeuristic and IsMetricGoal
        Eigen::VectorXd m_terminal_costs_;                 // used to store the terminal costs in ComputeHeuristic
        Eigen::VectorXb m_goals_reached_;                  // used to store the goals reached in IsMetricGoal
        bool m_terminal_cost_set_ = false;
        std::size_t m_max_num_actions_ = 0;

    public:
        struct Successor : env::Successor {
            uint8_t action_resolution_level = 0;

            Successor() = default;

            Successor(std::shared_ptr<env::EnvironmentState> state, double cost, std::size_t action_id, uint8_t resolution_level)
                : env::Successor(std::move(state), cost, action_id),
                  action_resolution_level(resolution_level) {}

            Successor(Eigen::VectorXd env_metric_state, Eigen::VectorXi env_grid_state, double cost, std::size_t action_id, uint8_t resolution_level)
                : env::Successor(std::move(env_metric_state), std::move(env_grid_state), cost, action_id),
                  action_resolution_level(resolution_level) {}

            Successor(env::Successor successor, uint8_t creation_resolution_level)
                : env::Successor(std::move(successor)),
                  action_resolution_level(creation_resolution_level) {}
        };

        /**
         *
         * @param environments environment instances for each resolution level
         * @param heuristics instances of heuristics
         * @param heuristic_assignments resolution level that each heuristic is assigned to
         */
        PlanningInterfaceMultiResolutions(
            const std::vector<std::shared_ptr<env::EnvironmentBase>> &environments,
            std::vector<std::pair<std::shared_ptr<HeuristicBase>, uint8_t>> heuristics,
            Eigen::VectorXd metric_start_coords,
            std::vector<Eigen::VectorXd> metric_goals_coords,
            std::vector<Eigen::VectorXd> metric_goals_tolerances)
            : m_heuristics_(std::move(heuristics)),
              m_metric_start_(std::move(metric_start_coords)),
              m_goals_(std::move(metric_goals_coords)),
              m_goals_tolerances_(std::move(metric_goals_tolerances)),
              m_goals_reached_(Eigen::VectorXb::Constant(GetNumGoals(), false)),
              m_terminal_cost_set_(false) {

            m_envs_.insert(m_envs_.end(), environments.begin(), environments.end());
            std::size_t num_resolution_levels = m_envs_.size();
            // check if all environments are valid
            for (std::size_t i = 1; i < num_resolution_levels; ++i) {
                ERL_ASSERTM(m_envs_[i] != nullptr, "environment %d is nullptr.", int(i));
                m_max_num_actions_ = std::max(m_max_num_actions_, m_envs_[i]->GetActionSpaceSize());
            }

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
            SetStart();
        }

        PlanningInterfaceMultiResolutions(
            const std::vector<std::shared_ptr<env::EnvironmentBase>> &environments,
            std::vector<std::pair<std::shared_ptr<HeuristicBase>, uint8_t>> heuristics,
            Eigen::VectorXd metric_start_coords,
            std::vector<Eigen::VectorXd> metric_goals_coords,
            std::vector<Eigen::VectorXd> metric_goals_tolerances,
            Eigen::VectorXd terminal_costs)
            : PlanningInterfaceMultiResolutions(
                  environments,
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

        [[nodiscard]] inline int
        GetNumGoals() const {
            return int(m_goals_.size());
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

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const std::shared_ptr<env::EnvironmentState> &state, uint8_t env_resolution_level) const {

            if (env_resolution_level == 0) {  // anchor-level, use the whole action space, i.e. union of all action spaces of all resolution levels
                std::vector<Successor> successors;
                for (auto it = m_envs_.begin() + 1; it < m_envs_.end(); ++it) {
                    std::vector<env::Successor> env_successors = (*it)->GetSuccessors(state);
                    if (env_successors.empty()) { continue; }
                    std::size_t resolution_level = std::distance(m_envs_.begin(), it);
                    for (auto &successor: env_successors) { successors.emplace_back(successor, resolution_level); }
                }
                return successors;
            }

            std::vector<Successor> successors;
            std::vector<env::Successor> env_successors = m_envs_[env_resolution_level]->GetSuccessors(state);
            for (auto &successor: env_successors) { successors.emplace_back(successor, env_resolution_level); }
            return successors;
        }

        inline void
        GetHeuristicValues(const std::shared_ptr<env::EnvironmentState> &state, std::vector<double> &heuristic_values) const {
            std::size_t num_heuristics = GetNumHeuristics();
            heuristic_values.resize(num_heuristics);
            for (std::size_t i = 0; i < num_heuristics; ++i) { heuristic_values[i] = (*m_heuristics_[i].first)(*state); }
        }

        [[nodiscard]] inline std::vector<double>
        GetHeuristicValues(const std::shared_ptr<env::EnvironmentState> &state) const {
            std::vector<double> heuristic_values;
            GetHeuristicValues(state, heuristic_values);
            return heuristic_values;
        }

        [[nodiscard]] int  // return the index of the goal that is reached, -1 if none is reached
        IsGoal(const std::shared_ptr<env::EnvironmentState> &state, bool ignore_reached = false) {
            int num_goals = GetNumGoals();
            long dim = state->metric.size();
            for (int i = 0; i < num_goals; ++i) {
                if (ignore_reached && m_goals_reached_[i]) { continue; }

                if (m_terminal_cost_set_ && (m_terminal_costs_[i] >= std::numeric_limits<double>::max())) { continue; }

                bool goal_reached = true;
                for (long j = 0; j < dim; ++j) {
                    double err = std::abs(state->metric[j] - m_goals_[i][j]);
                    if (err > m_goals_tolerances_[i][j]) {
                        goal_reached = false;
                        break;
                    }
                }

                if (goal_reached) {
                    m_goals_reached_[i] = true;
                    return i;
                }
            }

            return -1;
        }

        [[nodiscard]] inline std::size_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &state) const {
            // use the anchor-level environment to hash the state
            return m_envs_[0]->StateHashing(state);
        }

    private:
        inline void
        SetStart() {
            m_grid_start_ = m_envs_[0]->MetricToGrid(m_init_start_);
            m_metric_start_ = m_envs_[0]->GridToMetric(m_grid_start_);
        }
    };
}  // namespace erl::search_planning
