#pragma once

#include "heuristic.hpp"

#include "erl_common/eigen.hpp"
#include "erl_env/environment_base.hpp"

#include <memory>
#include <typeinfo>
#include <utility>
#include <vector>

namespace erl::search_planning {

    /**
     * PlanningInterface provides the minimum functionality for a typical planning algorithm that needs obtaining successors, computing heuristics, checking
     * a goal state and hashing a state as an index.
     */
    class PlanningInterface {  // virtual inheritance to avoid diamond problem
    protected:
        std::shared_ptr<env::EnvironmentBase> m_env_ = nullptr;        // used to store the environment in GetSuccessors and ComputeHeuristic
        std::shared_ptr<HeuristicBase> m_heuristic_ = nullptr;         // allow custom heuristic function to be used as callback in ComputeHeuristic
        Eigen::VectorXd m_init_start_;                                 // initial start state in metric space
        std::shared_ptr<env::EnvironmentState> m_start_;               // start state in metric & grid space
        std::vector<std::shared_ptr<env::EnvironmentState>> m_goals_;  // goal states in metric & grid space
        std::vector<Eigen::VectorXd> m_goals_tolerances_;              // used by IsMetricGoal to check if a goal is reached
        std::vector<double> m_terminal_costs_;                         // used by IsMetricGoal to check if a goal is reachable
        std::vector<env::Successor> m_virtual_goal_successors_ = {};

    public:
        PlanningInterface(
            std::shared_ptr<env::EnvironmentBase> env,
            Eigen::VectorXd metric_start_coords,
            const std::vector<Eigen::VectorXd> &metric_goals_coords,
            std::vector<Eigen::VectorXd> metric_goals_tolerances,
            std::vector<double> terminal_costs = {0.},
            std::shared_ptr<HeuristicBase> heuristic = nullptr);

        PlanningInterface(
            std::shared_ptr<env::EnvironmentBase> env,
            Eigen::VectorXd metric_start_coords,
            Eigen::VectorXd metric_goal_coords,
            Eigen::VectorXd metric_goal_tolerance,
            double terminal_cost = 0.,
            std::shared_ptr<HeuristicBase> heuristic = nullptr)
            : PlanningInterface(
                  std::move(env),
                  std::move(metric_start_coords),
                  std::vector({std::move(metric_goal_coords)}),
                  std::vector({std::move(metric_goal_tolerance)}),
                  {terminal_cost},
                  std::move(heuristic)) {}

        [[nodiscard]] std::shared_ptr<env::EnvironmentBase>
        GetEnvironment() const {
            return m_env_;
        }

        [[nodiscard]] double
        GetHeuristic(const std::shared_ptr<env::EnvironmentState> &env_state) const {
            return (*m_heuristic_)(*env_state);
        }

        [[nodiscard]] std::vector<env::Successor>
        GetSuccessors(const std::shared_ptr<env::EnvironmentState> &env_state) const {
            std::vector<env::Successor> successors = m_env_->GetSuccessors(env_state);
            if (const int goal_index = IsMetricGoal(env_state); goal_index >= 0) { successors.push_back(m_virtual_goal_successors_[goal_index]); }
            return successors;
        }

        [[nodiscard]] std::shared_ptr<env::EnvironmentState>
        GetStartState() const {
            return m_start_;
        }

        [[nodiscard]] std::shared_ptr<env::EnvironmentState>
        GetGoalState(const int goal_index) const {
            return m_goals_[goal_index];
        }

        [[nodiscard]] int
        GetNumGoals() const {
            return static_cast<int>(m_goals_.size());
        }

        [[nodiscard]] double
        GetTerminalCost(const int goal_index) const {
            return m_terminal_costs_[goal_index];
        }

        [[nodiscard]] int  // return the index of the goal that is reached, -1 if none is reached
        IsMetricGoal(const std::shared_ptr<env::EnvironmentState> &env_state) const;

        [[nodiscard]] static bool
        IsVirtualGoal(const std::shared_ptr<const env::EnvironmentState> &env_state) {
            return env_state->grid[0] == env::VirtualStateValue::kGoal;
        }

        [[nodiscard]] static bool
        ReachGoal(const std::shared_ptr<env::EnvironmentState> &env_state) {
            return IsVirtualGoal(env_state);  // only virtual goal can be reached
        }

        [[nodiscard]] long
        StateHashing(const std::shared_ptr<env::EnvironmentState> &env_state) const {
            if (IsVirtualGoal(env_state)) { return env::VirtualStateValue::kGoal; }  // virtual goal env_state
            return m_env_->StateHashing(env_state);
        }

        [[nodiscard]] std::vector<std::shared_ptr<env::EnvironmentState>>
        GetPath(const std::shared_ptr<const env::EnvironmentState> &env_state, const std::vector<int> &action_coords) const {
            if (IsVirtualGoal(env_state)) {
                ERL_DEBUG_ASSERT(action_coords[0] < 0, "virtual goal env_state can only be reached with action_coords[0] = -goal_index - 1.");
                return {m_goals_[-(action_coords[0] + 1)]};
            }
            return m_env_->ForwardAction(env_state, action_coords);
        }

    private:
        void
        SetStart() {
            m_start_ = std::make_shared<env::EnvironmentState>();
            m_start_->grid = m_env_->MetricToGrid(m_init_start_);
            m_start_->metric = m_env_->GridToMetric(m_start_->grid);
        }

        void
        SetGoals(const std::vector<Eigen::VectorXd> &metric_goals_coords) {
            m_goals_.reserve(metric_goals_coords.size());
            for (const auto &metric_goal_coords: metric_goals_coords) {
                auto goal = std::make_shared<env::EnvironmentState>();
                goal->metric = metric_goal_coords;
                goal->grid = m_env_->MetricToGrid(metric_goal_coords);
                m_goals_.push_back(goal);
            }
        }
    };

}  // namespace erl::search_planning
