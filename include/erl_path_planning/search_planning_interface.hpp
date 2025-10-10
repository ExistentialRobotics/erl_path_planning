#pragma once

#include "heuristic.hpp"

#include "erl_env/environment_base.hpp"
#include "erl_env/environment_multi_resolution.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace erl::path_planning {

    /**
     * PlanningInterface provides the minimum functionality for a typical planning algorithm that
     * needs obtaining successors, computing heuristics, checking a goal state and hashing a state
     * as an index.
     */
    template<typename Dtype, int Dim>
    class SearchPlanningInterface {  // virtual inheritance to avoid diamond problem
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
        SearchPlanningInterface(
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

        SearchPlanningInterface(
            std::shared_ptr<Env> env,
            MetricState metric_start_coords,
            MetricState metric_goal_coords,
            MetricState metric_goal_tolerance,
            Dtype terminal_cost = static_cast<Dtype>(0.0),
            std::shared_ptr<Heuristic> heuristic = nullptr)
            : SearchPlanningInterface(
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

    extern template class SearchPlanningInterface<float, 2>;
    extern template class SearchPlanningInterface<double, 2>;
    extern template class SearchPlanningInterface<float, 3>;
    extern template class SearchPlanningInterface<double, 3>;
    extern template class SearchPlanningInterface<float, 4>;
    extern template class SearchPlanningInterface<double, 4>;

    template<typename Dtype, int Dim>
    class SearchPlanningInterfaceMultiResolutions {
    public:
        using Env = env::EnvironmentMultiResolution<Dtype, Dim>;
        using EnvState = typename Env::State;
        using MetricState = typename EnvState::MetricState;
        using Successor = typename Env::Successor_t;
        using Heuristic = HeuristicBase<Dtype, Dim>;

    protected:
        /*
         * m_envs_ stores the environment instances for each resolution level. m_envs_[0] is the
         * anchor-level environment. Its state space is the union of state spaces of all other
         * environments. And its action space is the union of action spaces of all other
         * environments. m_envs_[i] is the i-th resolution-level environment.
         */
        std::shared_ptr<Env> m_env_multi_resolution_ = nullptr;
        // heuristic and its resolution level
        std::vector<std::pair<std::shared_ptr<Heuristic>, std::size_t>> m_heuristics_ = {};
        std::vector<std::vector<std::size_t>> m_heuristic_ids_by_resolution_level_ = {};

        MetricState m_init_start_;       // initial start state in metric space
        EnvState m_start_;               // start state in metric & grid space
        std::vector<EnvState> m_goals_;  // goal states in metric & grid space
        // used by GetHeuristic and IsMetricGoal to check if a goal is reached
        std::vector<MetricState> m_goals_tolerances_;
        std::vector<Dtype> m_terminal_costs_;  // used to store the terminal costs in GetHeuristic
        std::vector<Successor> m_virtual_goal_successors_ = {};

    public:
        SearchPlanningInterfaceMultiResolutions(
            std::shared_ptr<Env> env_multi_resolution,
            std::vector<std::pair<std::shared_ptr<Heuristic>, std::size_t>> heuristics,
            MetricState metric_start_coords,
            const std::vector<MetricState> &metric_goals_coords,
            std::vector<MetricState> metric_goals_tolerances,
            std::vector<Dtype> terminal_costs = {static_cast<Dtype>(0.0)})
            : m_env_multi_resolution_(std::move(env_multi_resolution)),
              m_heuristics_(std::move(heuristics)),
              m_init_start_(std::move(metric_start_coords)),
              m_goals_tolerances_(std::move(metric_goals_tolerances)),
              m_terminal_costs_(std::move(terminal_costs)) {

            // check environments
            ERL_ASSERTM(m_env_multi_resolution_ != nullptr, "environment_anchor is nullptr.");
            ERL_ASSERTM(!metric_goals_coords.empty(), "no goal is provided.");

            // check goals and goals tolerances
            std::size_t num_goals = metric_goals_coords.size();
            ERL_ASSERTM(num_goals > 0, "at least one goal must be provided.");
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

            // check heuristics
            const std::size_t num_res_levels = m_env_multi_resolution_->GetNumResolutionLevels();
            const std::size_t num_heuristics = m_heuristics_.size();
            m_heuristic_ids_by_resolution_level_.clear();
            m_heuristic_ids_by_resolution_level_.resize(num_res_levels);
            for (std::size_t i = 0; i < num_heuristics; ++i) {
                ERL_ASSERTM(
                    m_heuristics_[i].first != nullptr,
                    "heuristics[{}] is nullptr.",
                    static_cast<int>(i));
                ERL_ASSERTM(
                    (metric_goals_coords.size() == 1 ||
                     std::dynamic_pointer_cast<MultiGoalsHeuristic<Dtype, Dim>>(
                         m_heuristics_[i].first) != nullptr),
                    "heuristics[{}] must be derived from MultiGoalsHeuristic when multiple goals "
                    "are provided.",
                    static_cast<int>(i));
                m_heuristic_ids_by_resolution_level_[m_heuristics_[i].second].push_back(i);
            }
            for (std::size_t i = 0; i < num_res_levels; ++i) {
                ERL_ASSERTM(
                    !m_heuristic_ids_by_resolution_level_[i].empty(),
                    "resolution level {} has no heuristic.",
                    static_cast<int>(i));
            }
            ERL_ASSERTM(
                m_heuristics_[0].second == 0,
                "the first heuristic must be assigned to the anchor level (resolution level 0).");
            ERL_ASSERTM(
                m_heuristic_ids_by_resolution_level_[0].size() == 1,
                "the anchor level must have exactly one heuristic.");

            SetStart();
            SetGoals(metric_goals_coords);
        }

        SearchPlanningInterfaceMultiResolutions(
            std::shared_ptr<Env> environment_multi_resolution,
            std::vector<std::pair<std::shared_ptr<Heuristic>, std::size_t>> heuristics,
            MetricState metric_start_coords,
            MetricState metric_goal_coords,
            MetricState metric_goal_tolerance)
            : SearchPlanningInterfaceMultiResolutions(
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
            return m_env_multi_resolution_->GetNumResolutionLevels();
        }

        [[nodiscard]] std::shared_ptr<Heuristic>
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

        [[nodiscard]] std::vector<uint8_t>
        GetInResolutionLevelFlags(const EnvState &env_state) const {
            const std::size_t num_resolution_levels = GetNumResolutionLevels();
            std::vector<uint8_t> in_resolution_level_flags(num_resolution_levels, false);
            in_resolution_level_flags[0] = true;
            if (IsVirtualGoal(env_state)) { return in_resolution_level_flags; }
            for (std::size_t i = 1; i < num_resolution_levels; ++i) {
                in_resolution_level_flags[i] = static_cast<uint8_t>(
                    m_env_multi_resolution_->InStateSpaceAtLevel(env_state, i));
            }
            return in_resolution_level_flags;
        }

        [[nodiscard]] std::vector<Successor>
        GetSuccessors(const EnvState &env_state, const std::size_t resolution_level) const {
            std::vector<Successor> successors =
                m_env_multi_resolution_->GetSuccessorsAtLevel(env_state, resolution_level);
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
        GetGoalState(const int index) const {
            return m_goals_[index];
        }

        [[nodiscard]] int
        GetNumGoals() const {
            return static_cast<int>(m_goals_.size());
        }

        [[nodiscard]] Dtype
        GetTerminalCost(const int index) const {
            return m_terminal_costs_[index];
        }

        [[nodiscard]] std::vector<Dtype>
        GetHeuristicValues(const EnvState &env_state) const {
            std::vector<Dtype> heuristic_values;
            const std::size_t num_heuristics = GetNumHeuristics();
            heuristic_values.resize(num_heuristics);
            for (std::size_t i = 0; i < num_heuristics; ++i) {
                heuristic_values[i] = (*m_heuristics_[i].first)(env_state);
            }
            return heuristic_values;
        }

        [[nodiscard]] int  // return the index of the goal that is reached, -1 if none is reached
        IsMetricGoal(const EnvState &env_state) const {
            if (IsVirtualGoal(env_state)) { return -1; }
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
            if (env_state.grid[0] == env::VirtualStateValue::kGoal) {
                return env::VirtualStateValue::kGoal;  // virtual goal env_state
            }
            // use the anchor-level environment to hash the state
            return m_env_multi_resolution_->StateHashing(env_state);
        }

        [[nodiscard]] std::vector<EnvState>
        GetPath(const EnvState &env_state, long level, long action_idx) const {
            if (env_state.grid[0] == env::VirtualStateValue::kGoal) {
                ERL_DEBUG_ASSERT(
                    action_idx < 0,
                    "virtual goal env_state can only be reached with action_coords[0] = "
                    "-goal_index - 1.");
                return {m_goals_[-(action_idx + 1)]};
            }
            return m_env_multi_resolution_->ForwardActionAtLevel(env_state, level, action_idx);
        }

    private:
        void
        SetStart() {
            m_start_.grid = m_env_multi_resolution_->MetricToGrid(m_init_start_);
            m_start_.metric = m_env_multi_resolution_->GridToMetric(m_start_.grid);
        }

        void
        SetGoals(const std::vector<MetricState> &metric_goals_coords) {
            m_goals_.reserve(metric_goals_coords.size());
            for (const auto &metric_goal_coords: metric_goals_coords) {
                EnvState goal;
                goal.metric = metric_goal_coords;
                goal.grid = m_env_multi_resolution_->MetricToGrid(metric_goal_coords);
                m_goals_.push_back(goal);
            }
        }
    };

    extern template class SearchPlanningInterfaceMultiResolutions<float, 2>;
    extern template class SearchPlanningInterfaceMultiResolutions<double, 2>;
    extern template class SearchPlanningInterfaceMultiResolutions<float, 3>;
    extern template class SearchPlanningInterfaceMultiResolutions<double, 3>;
    extern template class SearchPlanningInterfaceMultiResolutions<float, 4>;
    extern template class SearchPlanningInterfaceMultiResolutions<double, 4>;

}  // namespace erl::path_planning
