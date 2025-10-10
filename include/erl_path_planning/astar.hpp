#pragma once

#include "planning_output.hpp"
#include "search_planning_interface.hpp"

#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>
#include <boost/heap/d_ary_heap.hpp>

#include <limits>
#include <memory>
#include <utility>

namespace erl::path_planning::astar {

    template<typename Dtype, int Dim>
    struct State;

    template<typename Dtype, int Dim>
    struct PriorityQueueItem {
        Dtype f_value = std::numeric_limits<Dtype>::infinity();
        std::shared_ptr<State<Dtype, Dim>> state = nullptr;

        PriorityQueueItem() = default;

        PriorityQueueItem(const Dtype f, std::shared_ptr<State<Dtype, Dim>> s)
            : f_value(f),
              state(std::move(s)) {}
    };

    template<typename T>
    struct Greater {
        bool
        operator()(const std::shared_ptr<T>& s1, const std::shared_ptr<T>& s2) const {
            if (std::abs(s1->f_value - s2->f_value) < 1.e-6) {
                // f value is too close, compare g value
                return s1->state->g_value > s2->state->g_value;
            }
            return s1->f_value > s2->f_value;
        }
    };

    // min-heap because we use Greater as a comparer instead
    // clang-format off
    template<typename Dtype, int Dim>
    using PriorityQueue = boost::heap::d_ary_heap<
        std::shared_ptr<PriorityQueueItem<Dtype, Dim>>,
        boost::heap::mutable_<true>,
        boost::heap::stable<true>,  // guarantee the order of equal elements, last in first out
        boost::heap::arity<8>,
        boost::heap::compare<Greater<PriorityQueueItem<Dtype, Dim>>>>;

    // clang-format on

    template<typename Dtype, int Dim>
    struct State {
        using EnvState = env::EnvironmentState<Dtype, Dim>;

        EnvState env_state;
        std::shared_ptr<State> parent = nullptr;
        long action_idx = -1;
        typename PriorityQueue<Dtype, Dim>::handle_type heap_key = {};

        // ReSharper disable once CppInconsistentNaming
        Dtype g_value = std::numeric_limits<Dtype>::infinity();
        Dtype h_value = std::numeric_limits<Dtype>::infinity();

        std::size_t iteration_opened = 0;
        std::size_t iteration_closed = 0;

        State(EnvState env_state_in, const Dtype g, const Dtype h, const std::size_t iter_opened)
            : env_state(std::move(env_state_in)),
              g_value(g),
              h_value(h),
              iteration_opened(iter_opened) {}

        State(EnvState env_state_in, const Dtype h)
            : env_state(std::move(env_state_in)),
              h_value(h) {}

        [[nodiscard]] bool
        IsOpened() const {
            return iteration_opened > iteration_closed;
        }

        [[nodiscard]] bool
        IsClosed() const {
            return iteration_opened < iteration_closed;
        }
    };

    template<typename Dtype, int Dim>
    struct Output : public PlanningOutput<Dtype, Dim> {
        // logging
        using MetricState = typename env::EnvironmentState<Dtype, Dim>::MetricState;
        std::map<std::size_t, std::list<MetricState>> opened_list = {};
        std::map<std::size_t, MetricState> closed_list = {};
        std::map<std::size_t, std::list<MetricState>> inconsistent_list = {};
    };

    template<typename Dtype>
    struct AstarSetting : public common::Yamlable<AstarSetting<Dtype>> {
        Dtype eps = static_cast<Dtype>(1.0);
        long max_num_iterations = -1;
        bool log = false;
        bool reopen_inconsistent = false;

        struct YamlConvertImpl {
            static YAML::Node
            encode(const AstarSetting& setting) {
                YAML::Node node;
                ERL_YAML_SAVE_ATTR(node, setting, eps);
                ERL_YAML_SAVE_ATTR(node, setting, max_num_iterations);
                ERL_YAML_SAVE_ATTR(node, setting, log);
                ERL_YAML_SAVE_ATTR(node, setting, reopen_inconsistent);
                return node;
            }

            static bool
            decode(const YAML::Node& node, AstarSetting& setting) {
                ERL_YAML_LOAD_ATTR(node, setting, eps);
                ERL_YAML_LOAD_ATTR(node, setting, max_num_iterations);
                ERL_YAML_LOAD_ATTR(node, setting, log);
                ERL_YAML_LOAD_ATTR(node, setting, reopen_inconsistent);
                return true;
            }
        };
    };

    template<typename Dtype, int Dim>
    class AStar {

    public:
        using Setting = AstarSetting<Dtype>;
        using State_t = State<Dtype, Dim>;
        using EnvState = typename State_t::EnvState;
        using PlanningInterface_t = SearchPlanningInterface<Dtype, Dim>;
        using PriorityQueue_t = PriorityQueue<Dtype, Dim>;
        using PriorityQueueItem_t = PriorityQueueItem<Dtype, Dim>;
        using Output_t = Output<Dtype, Dim>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<State_t> m_start_state_ = nullptr;
        std::shared_ptr<State_t> m_current_ = nullptr;
        std::shared_ptr<PlanningInterface_t> m_planning_interface_ = nullptr;
        PriorityQueue_t m_priority_queue_;
        absl::flat_hash_map<long, std::shared_ptr<State_t>> m_astar_states_;
        long m_iterations_ = 0;
        // bool m_planned_ = false;
        absl::flat_hash_set<int> m_reached_goal_indices_;
        std::shared_ptr<Output_t> m_output_ = nullptr;

    public:
        explicit AStar(
            std::shared_ptr<PlanningInterface_t> planning_interface,
            std::shared_ptr<Setting> setting = nullptr)
            : m_setting_(std::move(setting)),
              m_planning_interface_(std::move(planning_interface)),
              m_output_(std::make_shared<Output_t>()) {

            if (!m_setting_) { m_setting_ = std::make_shared<Setting>(); }
            ERL_ASSERTM(m_setting_->eps > 0., "eps must be positive.");
            // initialize start node
            auto start_env_state = m_planning_interface_->GetStartState();
            auto& start = GetState(start_env_state);
            start = std::make_shared<State_t>(
                std::move(start_env_state),
                0.0f,
                m_planning_interface_->GetHeuristic(start_env_state),
                m_iterations_);
            m_current_ = start;
            m_start_state_ = start;
        }

        [[nodiscard]] std::size_t
        GetIterations() const {
            return m_iterations_;
        }

        [[nodiscard]] std::shared_ptr<Output_t>
        Plan() {
            if (m_reached_goal_indices_.size() == m_planning_interface_->GetNumGoals()) {
                return m_output_;  // all reachable goals are reached
            }

            // should not check if the start state is a goal here.
            // when there are multiple goals with different terminal costs, this may cause
            // suboptimal output.

            if (m_astar_states_.size() > 1 && !m_output_->plan_records.empty()) {
                // Plan() is already called, resume the search.
                // previous call of Plan() reached a goal
                if (m_priority_queue_.empty()) {
                    // Open is empty, the free space reachable from start is fully explored.
                    // get all remaining goals and their costs
                    std::vector<std::tuple<int, Dtype, std::shared_ptr<State_t>>> goal_cost_states;
                    auto num_goals = static_cast<int>(m_planning_interface_->GetNumGoals());
                    for (int i = 0; i < num_goals; ++i) {
                        if (m_reached_goal_indices_.contains(i)) { continue; }
                        if (auto goal_state = GetState(m_planning_interface_->GetGoalState(i));
                            goal_state != nullptr) {
                            goal_cost_states.emplace_back(i, goal_state->g_value, goal_state);
                        }
                    }
                    // all reachable goals are reached
                    if (goal_cost_states.empty()) { return m_output_; }

                    // order remaining goals by cost in ascending order
                    std::sort(
                        goal_cost_states.begin(),
                        goal_cost_states.end(),
                        [](const auto& a, const auto& b) {
                            return std::get<1>(a) < std::get<1>(b);
                        });
                    // get the goal with the smallest cost
                    const auto [goal_index, cost, goal_state] = goal_cost_states.front();
                    ++m_iterations_;
                    RecoverPath(goal_state, goal_index);
                    m_reached_goal_indices_.insert(goal_index);
                    return m_output_;
                }

                m_current_ = m_priority_queue_.top()->state;
                m_priority_queue_.pop();
            }

            while (m_setting_->max_num_iterations < 0 ||
                   m_iterations_ < m_setting_->max_num_iterations) {

                // increase the number of iterations
                ++m_iterations_;

                // check if done
                if (m_planning_interface_->ReachGoal(m_current_->env_state)) {
                    RecoverPath(m_current_, -1);
                    LogStates();
                    return m_output_;
                }

                // add current node to closed set
                m_current_->iteration_closed = m_iterations_;

                // perform the expansion of current iteration
                Expand();

                // Remove the node with the smallest cost
                if (m_priority_queue_.empty()) { break; }
                m_current_ = m_priority_queue_.top()->state;
                m_priority_queue_.pop();
            }

            // infeasible problem
            ERL_INFO("Reached maximum number of iterations.");
            LogStates();

            return m_output_;
        }

    private:
        std::shared_ptr<State_t>&
        GetState(const EnvState& env_state) {
            return m_astar_states_[m_planning_interface_->StateHashing(env_state)];
        }

        void
        Expand() {
            // get successors
            auto successors = m_planning_interface_->GetSuccessors(m_current_->env_state);

            // process successors
            for (auto& successor: successors) {
                // get child node
                auto& child = GetState(successor.env_state);
                if (!child) {  // new node
                    child = std::make_shared<State_t>(
                        successor.env_state,
                        m_planning_interface_->GetHeuristic(successor.env_state));
                }

                Dtype tentative_g_value = m_current_->g_value + successor.cost;
                Dtype old_g_value = child->g_value;
                if (tentative_g_value < child->g_value) {

                    child->parent = m_current_;
                    child->action_idx = successor.action_idx;
                    child->g_value = tentative_g_value;
                    Dtype f_value = tentative_g_value + m_setting_->eps * child->h_value;

                    if (child->IsOpened()) {  // already in open set
                        (*child->heap_key)->f_value = f_value;
                        m_priority_queue_.increase(child->heap_key);
                    } else if (child->IsClosed()) {  // already in closed set
                        // unexpected in A* with consistent heuristic because closed node has
                        // optimal g value this happens if m_interface_ does not provide consistent
                        // heuristic function
                        if (m_setting_->reopen_inconsistent) {
                            child->heap_key = m_priority_queue_.push(
                                std::make_shared<PriorityQueueItem_t>(f_value, child));
                            child->iteration_closed = 0;
                        } else {
                            ERL_WARN_ONCE(
                                "Inconsistent heuristic function is detected! (reopen_inconsistent "
                                "is false). g_value changes from {} to {}.",
                                old_g_value,
                                tentative_g_value);
                            if (m_setting_->log) {
                                m_output_->inconsistent_list[m_iterations_].push_back(
                                    child->env_state.metric);
                            }
                        }
                    } else {
                        // new node
                        child->heap_key = m_priority_queue_.push(
                            std::make_shared<PriorityQueueItem_t>(f_value, child));
                        child->iteration_opened = m_iterations_;
                    }
                }
            }
        }

        void
        RecoverPath(const std::shared_ptr<State_t>& goal_state, int goal_index) {
            auto [plan_record_itr, inserted] = m_output_->plan_records.insert({m_iterations_, {}});
            ERL_ASSERTM(
                inserted,
                "Planning record for iteration {} already exists.",
                m_iterations_);

            m_output_->latest_plan_itr = m_iterations_;
            auto& plan_record = plan_record_itr->second;

            // terminal cost is included in g_value if goal_state is virtual goal
            plan_record.cost = goal_state->g_value;
            std::shared_ptr<State_t> node = goal_state;
            if (m_planning_interface_->IsVirtualGoal(node->env_state)) {
                node = node->parent;
                plan_record.goal_index = m_planning_interface_->IsMetricGoal(node->env_state);
                ERL_DEBUG_ASSERT(plan_record.goal_index >= 0, "goal index is invalid.");
            } else {
                ERL_DEBUG_ASSERT(
                    goal_index >= 0,
                    "goal index is invalid when goal_state is not virtual goal.");
                plan_record.goal_index = goal_index;
                plan_record.cost += m_planning_interface_->GetTerminalCost(goal_index);
            }

            // if a goal was reached in previous call of Plan(), the goal is in CLOSED set,
            // and we should not get it again.
            const bool new_goal = m_reached_goal_indices_.insert(plan_record.goal_index).second;
            (void) new_goal;
            ERL_DEBUG_ASSERT(new_goal, "Goal {} is reached before.", plan_record.goal_index);

            if (node == nullptr) {}
            ERL_DEBUG_ASSERT(node != nullptr, "Goal state is not found.");

            ERL_DEBUG(
                "Reach goal[{}/{}] (metric: {}, grid: {}) from metric start {} at iteration {}.",
                plan_record.goal_index,
                m_planning_interface_->GetNumGoals(),
                common::EigenToNumPyFmtString(node->env_state.metric.transpose()),
                common::EigenToNumPyFmtString(node->env_state.grid.transpose()),
                common::EigenToNumPyFmtString(
                    m_planning_interface_->GetStartState().metric.transpose()),
                m_iterations_);

            plan_record.env_action_indices.clear();
            const long env_id = m_planning_interface_->GetEnvironment()->GetEnvId();
            while (node->parent != nullptr) {
                plan_record.env_action_indices.emplace_back(env_id, node->action_idx);
                node = node->parent;
            }
            std::reverse(
                plan_record.env_action_indices.begin(),
                plan_record.env_action_indices.end());
            std::vector<std::vector<EnvState>> path_segments;
            std::size_t num_path_states = 0;
            EnvState state = GetState(m_start_state_->env_state)->env_state;
            for (auto& [_, action_index]: plan_record.env_action_indices) {
                std::vector path_segment = m_planning_interface_->GetPath(state, action_index);
                if (path_segment.empty()) { continue; }
                path_segments.push_back(path_segment);
                num_path_states += path_segment.size();
                state = path_segment.back();
            }
            plan_record.path.resize(Eigen::NoChange, static_cast<long>(num_path_states + 1));
            plan_record.path.col(0) = m_planning_interface_->GetStartState().metric;
            long index = 1;
            for (auto& path_segment: path_segments) {
                const auto num_states = static_cast<long>(path_segment.size());
                for (long i = 0; i < num_states; ++i) {
                    plan_record.path.col(index++) = path_segment[i].metric;
                }
            }
        }

        void
        LogStates() const {
            if (!m_setting_->log) { return; }
            for (auto& [state_hashing, astar_state]: m_astar_states_) {
                if (astar_state->IsOpened()) {
                    m_output_->opened_list[astar_state->iteration_opened].push_back(
                        astar_state->env_state.metric);
                } else if (astar_state->IsClosed()) {
                    m_output_->closed_list[astar_state->iteration_closed] =
                        astar_state->env_state.metric;
                }
            }
        }
    };

    extern template class AStar<float, 2>;
    extern template class AStar<double, 2>;
    extern template class AStar<float, 3>;
    extern template class AStar<double, 3>;
    extern template class AStar<float, 4>;
    extern template class AStar<double, 4>;
}  // namespace erl::path_planning::astar

template<>
struct YAML::convert<erl::path_planning::astar::AstarSetting<float>>
    : public erl::path_planning::astar::AstarSetting<float>::YamlConvertImpl {};

template<>
struct YAML::convert<erl::path_planning::astar::AstarSetting<double>>
    : public erl::path_planning::astar::AstarSetting<double>::YamlConvertImpl {};
