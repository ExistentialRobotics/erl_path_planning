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
     * PlanningInterface provides the minimum functionality for a typical planning algorithm that
     * needs obtaining successors, computing heuristics, checking a goal state and hashing a state
     * as an index.
     */
    template<typename Dtype, int Dim>
    class PlanningInterface {  // virtual inheritance to avoid diamond problem
    public:
        using Env = env::EnvironmentBase<Dtype, Dim>;
        using EnvState = typename Env::State;
        using MetricState = typename EnvState::MetricState;
        using Successor = typename Env::Successor_t;
        using Heuristic = HeuristicBase<Dtype, Dim>;

    protected:
        // the environment used in GetSuccessors and ComputeHeuristic
        std::shared_ptr<Env> m_env_ = nullptr;
        // allow custom heuristic function to be used as callback in ComputeHeuristic
        std::shared_ptr<Heuristic> m_heuristic_ = nullptr;
        MetricState m_init_start_;       // initial start state in metric space
        EnvState m_start_;               // start state in metric & grid space
        std::vector<EnvState> m_goals_;  // goal states in metric & grid space
        // used by IsMetricGoal to check if a goal is reached
        std::vector<MetricState> m_goals_tolerances_;
        // used by IsMetricGoal to check if a goal is reachable
        std::vector<Dtype> m_terminal_costs_;
        std::vector<Successor> m_virtual_goal_successors_ = {};

    public:
        PlanningInterface(
            std::shared_ptr<Env> env,
            MetricState metric_start_coords,
            const std::vector<MetricState> &metric_goals_coords,
            std::vector<MetricState> metric_goals_tolerances,
            std::vector<Dtype> terminal_costs = {static_cast<Dtype>(0.0)},
            std::shared_ptr<Heuristic> heuristic = nullptr)
            : m_env_(std::move(env)),
              m_heuristic_(std::move(heuristic)),
              m_init_start_(std::move(metric_start_coords)),
              m_goals_tolerances_(std::move(metric_goals_tolerances)),
              m_terminal_costs_(std::move(terminal_costs)) {

            ERL_ASSERTM(m_env_ != nullptr, "env is nullptr");

            const std::size_t num_goals = metric_goals_coords.size();
            ERL_ASSERTM(num_goals > 0, "no goal is provided");
            if (metric_goals_coords.size() > 1) {
                if (m_goals_tolerances_.size() == 1) {
                    ERL_INFO("only one goal tolerance is provided, copying it to all goals.");
                    m_goals_tolerances_.resize(num_goals, m_goals_tolerances_[0]);
                } else if (m_goals_tolerances_.size() != num_goals) {
                    ERL_FATAL(
                        "number of goal tolerances must be 1 or equal to the number of goals.");
                }
                if (m_terminal_costs_.size() == 1) {
                    ERL_INFO("only one terminal cost is provided, copying it to all goals.");
                    m_terminal_costs_.resize(num_goals, m_terminal_costs_[0]);
                } else if (m_terminal_costs_.size() != num_goals) {
                    ERL_FATAL(
                        "number of terminal costs must be 1 or equal to the number of goals.");
                }
            }

            // to handle nonzero terminal cost for single goal or multiple goals
            m_virtual_goal_successors_.resize(num_goals);
            for (std::size_t i = 0; i < num_goals; ++i) {
                auto &successor = m_virtual_goal_successors_[i];
                successor.env_state.grid[0] = env::VirtualStateValue::kGoal;
                successor.cost = m_terminal_costs_[i];
                successor.action_idx = -static_cast<int>(i) - 1;
            }

            if (m_heuristic_ == nullptr) {
                ERL_INFO(
                    "use {} as default heuristic for single goal.",
                    type_name<EuclideanDistanceHeuristic<Dtype, Dim>>());
                using DefaultHeuristic = EuclideanDistanceHeuristic<Dtype, Dim>;
                if (num_goals == 1) {
                    m_heuristic_ = std::make_shared<DefaultHeuristic>(
                        metric_goals_coords[0],
                        m_goals_tolerances_[0],
                        m_terminal_costs_[0]);
                } else {
                    std::vector<std::shared_ptr<Heuristic>> heuristics;
                    heuristics.reserve(num_goals);
                    for (std::size_t i = 0; i < num_goals; ++i) {
                        heuristics.emplace_back(
                            std::make_shared<DefaultHeuristic>(
                                metric_goals_coords[i],
                                m_goals_tolerances_[i],
                                m_terminal_costs_[i]));
                    }
                    m_heuristic_ =
                        std::make_shared<MultiGoalsHeuristic<Dtype, Dim>>(std::move(heuristics));
                }
            }

            SetStart();
            SetGoals(metric_goals_coords);
        }

        PlanningInterface(
            std::shared_ptr<Env> env,
            MetricState metric_start_coords,
            MetricState metric_goal_coords,
            MetricState metric_goal_tolerance,
            Dtype terminal_cost = static_cast<Dtype>(0.0),
            std::shared_ptr<Heuristic> heuristic = nullptr)
            : PlanningInterface(
                  std::move(env),
                  std::move(metric_start_coords),
                  std::vector({std::move(metric_goal_coords)}),
                  std::vector({std::move(metric_goal_tolerance)}),
                  {terminal_cost},
                  std::move(heuristic)) {}

        [[nodiscard]] std::shared_ptr<Env>
        GetEnvironment() const {
            return m_env_;
        }

        [[nodiscard]] Dtype
        GetHeuristic(const EnvState &env_state) const {
            return (*m_heuristic_)(env_state);
        }

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const EnvState &env_state) const {
            std::vector<Successor> successors = m_env_->GetSuccessors(env_state);
            if (const int goal_index = IsMetricGoal(env_state); goal_index >= 0) {
                successors.push_back(m_virtual_goal_successors_[goal_index]);
            }
            return successors;
        }

        [[nodiscard]] EnvState
        GetStartState() const {
            return m_start_;
        }

        [[nodiscard]] EnvState
        GetGoalState(const int goal_index) const {
            return m_goals_[goal_index];
        }

        [[nodiscard]] std::size_t
        GetNumGoals() const {
            return m_goals_.size();
        }

        [[nodiscard]] Dtype
        GetTerminalCost(const int goal_index) const {
            return m_terminal_costs_[goal_index];
        }

        [[nodiscard]] int  // return the index of the goal that is reached, -1 if none is reached
        IsMetricGoal(const EnvState &env_state) const {
            if (IsVirtualGoal(env_state)) { return -1; }  // already a virtual goal
            const int num_goals = GetNumGoals();
            const long dim = env_state.metric.size();
            for (int i = 0; i < num_goals; ++i) {
                if (std::isinf(m_terminal_costs_[i])) { continue; }
                bool goal_reached = true;
                for (long j = 0; j < dim; ++j) {
                    if (std::abs(env_state.metric[j] - m_goals_[i].metric[j]) >
                        m_goals_tolerances_[i][j]) {
                        goal_reached = false;
                        break;
                    }
                }
                if (goal_reached) { return i; }
            }
            return -1;
        }

        [[nodiscard]] static bool
        IsVirtualGoal(const EnvState &env_state) {
            return env_state.grid[0] == env::VirtualStateValue::kGoal;
        }

        [[nodiscard]] static bool
        ReachGoal(const EnvState &env_state) {
            return IsVirtualGoal(env_state);  // only virtual goal can be reached
        }

        [[nodiscard]] long
        StateHashing(const EnvState &env_state) const {
            // virtual goal env_state
            if (IsVirtualGoal(env_state)) { return env::VirtualStateValue::kGoal; }
            return m_env_->StateHashing(env_state);
        }

        [[nodiscard]] std::vector<EnvState>
        GetPath(const EnvState &env_state, const long action_idx) const {
            if (IsVirtualGoal(env_state)) {
                ERL_DEBUG_ASSERT(
                    action_idx < 0,
                    "virtual goal env_state can only be reached with action_coords[0] = "
                    "-goal_index - 1.");
                return {m_goals_[-(action_idx + 1)]};
            }
            return m_env_->ForwardAction(env_state, action_idx);
        }

    private:
        void
        SetStart() {
            m_start_.grid = m_env_->MetricToGrid(m_init_start_);
            m_start_.metric = m_env_->GridToMetric(m_start_.grid);
        }

        void
        SetGoals(const std::vector<MetricState> &metric_goals_coords) {
            m_goals_.reserve(metric_goals_coords.size());
            for (const auto &metric_goal_coords: metric_goals_coords) {
                EnvState goal;
                goal.metric = metric_goal_coords;
                goal.grid = m_env_->MetricToGrid(metric_goal_coords);
                m_goals_.push_back(goal);
            }
        }
    };

    extern template class PlanningInterface<float, 2>;
    extern template class PlanningInterface<double, 2>;
    extern template class PlanningInterface<float, 3>;
    extern template class PlanningInterface<double, 3>;
    extern template class PlanningInterface<float, 4>;
    extern template class PlanningInterface<double, 4>;

}  // namespace erl::search_planning
