#pragma once

// Implementation of AMRA*: Anytime Multi-Resolution Multi-HeuristicBase A*

#include "heuristic.hpp"
#include "planning_output.hpp"
#include "search_planning_interface.hpp"

#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>
#include <boost/heap/d_ary_heap.hpp>

#include <cstdint>
#include <limits>
#include <memory>

using namespace std::chrono_literals;

namespace erl::path_planning::amra_star {

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
        operator()(const std::shared_ptr<T> &s1, const std::shared_ptr<T> &s2) const {
            if (std::abs(s1->f_value - s2->f_value) < 1.e-6) {
                // f value is too close, compare g value
                return s1->state->g_value > s2->state->g_value;
            }
            return s1->f_value > s2->f_value;
        }
    };

    template<typename Dtype, int Dim>
    using PriorityQueue = boost::heap::d_ary_heap<
        std::shared_ptr<PriorityQueueItem<Dtype, Dim>>,
        boost::heap::mutable_<true>,
        boost::heap::arity<8>,
        boost::heap::compare<Greater<PriorityQueueItem<Dtype, Dim>>>>;

    template<typename Dtype, int Dim>
    struct State {
        using EnvState = env::EnvironmentState<Dtype, Dim>;

        uint32_t plan_itr = 0;  // iteration of the plan that generated/updated this state
        EnvState env_state;
        std::vector<uint64_t> iteration_opened;
        std::vector<typename PriorityQueue<Dtype, Dim>::handle_type> open_queue_keys;
        std::vector<uint64_t> iteration_closed;
        Dtype g_value = std::numeric_limits<Dtype>::infinity();
        std::vector<Dtype> h_values;                // heuristic values for all heuristics
        std::vector<uint8_t> in_resolution_levels;  // flags of the levels the state is in
        std::shared_ptr<State> parent = nullptr;    // parent state
        long env_level = -1;  // the env level that generated this state from its parent
        // the action index at env_level that generated this state from its parent
        long action_idx = -1;

        State(
            const uint32_t plan_itr_in,
            EnvState env_state_in,
            const std::size_t num_resolution_levels,
            std::vector<uint8_t> in_resolution_level_flags,
            std::vector<Dtype> h_vals)
            : plan_itr(plan_itr_in),
              env_state(std::move(env_state_in)),
              iteration_opened(h_vals.size(), 0),
              open_queue_keys(h_vals.size()),
              iteration_closed(num_resolution_levels, 0),
              h_values(std::move(h_vals)),
              in_resolution_levels(std::move(in_resolution_level_flags)) {
            ERL_DEBUG_ASSERT(
                this->in_resolution_levels.size() == num_resolution_levels,
                "in_resolution_level_flags.size() == %zu, num_resolution_levels = %zu",
                this->in_resolution_levels.size(),
                num_resolution_levels);
            ERL_DEBUG_ASSERT(this->in_resolution_levels[0], "in_resolution_level_flags[0] != true");
        }

        [[nodiscard]] bool
        InResolutionLevel(const std::size_t resolution_level) const {
            return in_resolution_levels[resolution_level] > 0;
        }

        [[nodiscard]] bool
        InOpened(const std::size_t open_set_id, const std::size_t close_set_id) const {
            // 1. if state is just moved into OPEN_i, then iteration_opened[i] > 0 and for other
            // heuristics assigned to the same resolution, iteration_opened[j] == 0 and any
            // iteration_closed[j] == 0, i.e. iteration_opened[i] > any iteration_closed[j] of the
            // same resolution.
            // 2. if state is moved into OPEN_i because it has not been in CLOSE_res(i), then
            // iteration_opened[i] > iteration_closed[res(i)]
            return iteration_opened[open_set_id] > iteration_closed[close_set_id];
        }

        [[nodiscard]] bool
        InClosed(const std::size_t closed_set_id) const {
            // as long as the state is moved into CLOSE_res(i), then iteration_closed[res(i)] > 0,
            // where close_set_id = res(i).
            return iteration_closed[closed_set_id] > 0;
        }

        void
        SetOpened(const std::size_t open_set_id, const uint64_t opened_itr) {
            iteration_opened[open_set_id] = opened_itr;
        }

        void
        SetClosed(const std::size_t close_set_id, const uint64_t closed_itr) {
            iteration_closed[close_set_id] = closed_itr;
        }

        void
        RemoveFromClosed(
            const std::size_t close_set_id,
            const std::vector<std::size_t> &open_set_ids) {
            if (!InClosed(close_set_id)) { return; }
            for (const auto open_set_id: open_set_ids) { iteration_opened[open_set_id] = 0; }
            iteration_closed[close_set_id] = 0;
        }

        void
        SetParent(std::shared_ptr<State> parent_in, long level, long action_idx_in) {
            parent = std::move(parent_in);
            env_level = level;
            action_idx = action_idx_in;
        }

        void
        Reset() {
            iteration_opened.resize(iteration_opened.size(), 0);
            iteration_closed.resize(iteration_closed.size(), 0);
            g_value = std::numeric_limits<Dtype>::infinity();
            parent = nullptr;
            env_level = -1;
            action_idx = -1;
        }
    };

    template<typename Dtype, int Dim>
    struct Output : public PlanningOutput<Dtype, Dim> {
        // statistics
        uint32_t num_heuristics = 0;
        uint32_t num_resolution_levels = 0;
        uint64_t num_expansions = 0;
        Dtype w1_solve = -1.0;
        Dtype w2_solve = -1.0;
        Dtype search_time = 0.;

        // logging
        std::unordered_map<uint32_t, Dtype> w1_values;  // plan_itr -> w1
        std::unordered_map<uint32_t, Dtype> w2_values;  // plan_itr -> w2
        using MetricState = typename env::EnvironmentState<Dtype, Dim>::MetricState;
        // total_expand_itr -> heuristic_id -> list of states
        std::unordered_map<uint32_t, std::unordered_map<std::size_t, std::list<MetricState>>>
            opened_states;
        // total_expand_itr -> action_resolution_level -> list of states
        std::unordered_map<uint32_t, std::unordered_map<std::size_t, MetricState>> closed_states;
        // total_expand_itr -> list of states
        std::unordered_map<uint32_t, std::list<MetricState>> inconsistent_states;

        void
        Save(const std::filesystem::path &file_path) const {
            std::ofstream ofs(file_path);
            ERL_ASSERTM(ofs.is_open(), "Failed to open file: {}", file_path.string());

            std::size_t num_successful_plans = this->plan_records.size();
            ofs << "AMRA* solution" << std::endl
                << "num_successful_plans: " << num_successful_plans << std::endl
                << "latest_plan_itr: " << this->latest_plan_itr << std::endl
                << "num_heuristics: " << num_heuristics << std::endl
                << "num_resolution_levels: " << num_resolution_levels << std::endl
                << "num_expansions: " << num_expansions << std::endl
                << "w1_solve: " << w1_solve << std::endl
                << "w2_solve: " << w2_solve << std::endl
                << "search_time: " << search_time << std::endl;

            // save solutions and their cost, actions, etc. for each plan iteration
            long d = 0;
            for (auto &[plan_itr, plan_record]: this->plan_records) {
                auto &[goal_index, path, env_level_action_indices, cost] = plan_record;

                d = path.rows();  // dimension of the state space
                long n = path.cols();

                ofs << "plan_itr: " << plan_itr << std::endl
                    << "w1: " << w1_values.at(plan_itr) << std::endl
                    << "w2: " << w2_values.at(plan_itr) << std::endl
                    << "goal_index: " << goal_index << std::endl
                    << "cost: " << cost << std::endl
                    << "num_waypoints: " << n << std::endl
                    << "path: " << std::endl;
                ofs << "pos[0]";
                for (int i = 1; i < d; ++i) { ofs << ", pos[" << i << "]"; }
                ofs << std::endl;
                for (int i = 0; i < n; ++i) {
                    ofs << path(0, i);
                    for (int j = 1; j < d; ++j) { ofs << ", " << path(j, i); }
                    ofs << std::endl;
                }
                ofs << "num_actions: " << env_level_action_indices.size() << std::endl
                    << "env_levels, action_indices" << std::endl;
                for (const auto &[env_level, action_idx]: env_level_action_indices) {
                    ofs << env_level << ", " << action_idx << std::endl;
                }
            }

            // save opened_states, closed_states, inconsistent_states
            std::size_t cnt = 1;
            for (const auto &[plan_itr, opened_states_at_plan_itr]: opened_states) {
                for (const auto &[heuristic_id, opened_states_at_heuristic_id]:
                     opened_states_at_plan_itr) {
                    cnt += opened_states_at_heuristic_id.size();
                }
            }
            ofs << "opened_states: " << std::endl
                << cnt << std::endl
                << "expand_itr, heuristic_id, pos[0]";
            for (long i = 1; i < d; ++i) { ofs << ", pos[" << i << "]"; }
            ofs << std::endl;
            for (const auto &[expand_itr, opened_states_at_expand_itr]: opened_states) {
                for (const auto &[heuristic_id, opened_states_at_heuristic_id]:
                     opened_states_at_expand_itr) {
                    for (const auto &state: opened_states_at_heuristic_id) {
                        // if (state.size() == 0) { continue; }
                        // skip empty state (e.g., virtual goal)
                        ofs << expand_itr << ", " << heuristic_id << ", " << state[0];
                        for (long i = 1; i < d; ++i) { ofs << ", " << state[i]; }
                        ofs << std::endl;
                    }
                }
            }

            cnt = 1;
            for (const auto &[expand_itr, closed_states_at_expand_itr]: closed_states) {
                cnt += closed_states_at_expand_itr.size();
            }
            ofs << "closed_states: " << std::endl
                << cnt << std::endl
                << "expand_itr, action_resolution_level, pos[0]";
            for (long i = 1; i < d; ++i) { ofs << ", pos[" << i << "]"; }
            ofs << std::endl;
            for (const auto &[expand_itr, closed_states_at_expand_itr]: closed_states) {
                for (const auto &[action_resolution_level, state]: closed_states_at_expand_itr) {
                    ofs << expand_itr << ", " << action_resolution_level << ", " << state[0];
                    for (long i = 1; i < d; ++i) { ofs << ", " << state[i]; }
                    ofs << std::endl;
                }
            }

            cnt = 1;
            for (const auto &[expand_itr, inconsistent_states_at_expand_itr]: inconsistent_states) {
                cnt += inconsistent_states_at_expand_itr.size();
            }
            ofs << "inconsistent_states: " << std::endl << cnt << std::endl << "expand_itr, pos[0]";
            for (long i = 1; i < d; ++i) { ofs << ", pos[" << i << "]"; }
            ofs << std::endl;
            for (const auto &[expand_itr, inconsistent_states_at_expand_itr]: inconsistent_states) {
                for (const auto &state: inconsistent_states_at_expand_itr) {
                    ofs << expand_itr << ", " << state[0];
                    for (long i = 1; i < d; ++i) { ofs << ", " << state[i]; }
                    ofs << std::endl;
                }
            }
            ofs.close();
        }
    };

    template<typename Dtype>
    struct AmraStarSetting : public common::Yamlable<AmraStarSetting<Dtype>> {
        std::chrono::nanoseconds time_limit = 10000000s;
        Dtype w1_init = 10.0f;
        Dtype w2_init = 20.0f;
        Dtype w1_final = 1.0f;
        Dtype w2_final = 1.0f;
        Dtype w1_decay_factor = 0.5f;
        Dtype w2_decay_factor = 0.5f;
        bool log = false;

        ERL_REFLECT_SCHEMA(
            AmraStarSetting,
            ERL_REFLECT_MEMBER(AmraStarSetting, time_limit),
            ERL_REFLECT_MEMBER(AmraStarSetting, w1_init),
            ERL_REFLECT_MEMBER(AmraStarSetting, w2_init),
            ERL_REFLECT_MEMBER(AmraStarSetting, w1_final),
            ERL_REFLECT_MEMBER(AmraStarSetting, w2_final),
            ERL_REFLECT_MEMBER(AmraStarSetting, w1_decay_factor),
            ERL_REFLECT_MEMBER(AmraStarSetting, w2_decay_factor),
            ERL_REFLECT_MEMBER(AmraStarSetting, log));
    };

    template<typename Dtype, int Dim>
    class AmraStar {

    public:
        using Setting = AmraStarSetting<Dtype>;
        using State_t = State<Dtype, Dim>;
        using EnvState = typename State_t::EnvState;
        using PlanningInterface = SearchPlanningInterfaceMultiResolutions<Dtype, Dim>;
        using PriorityQueue_t = PriorityQueue<Dtype, Dim>;
        using PriorityQueueItem_t = PriorityQueueItem<Dtype, Dim>;
        using Output_t = Output<Dtype, Dim>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<PlanningInterface> m_planning_interface_ = nullptr;
        uint32_t m_plan_itr_ = 0;
        uint64_t m_total_expand_itr_ = 0;
        Eigen::VectorX<uint64_t> m_expand_itr_;

        Dtype m_w1_ = 0;
        Dtype m_w2_ = 0;

        std::chrono::nanoseconds m_search_time_ = 0ns;

        std::vector<PriorityQueue_t> m_open_queues_ = {};
        absl::flat_hash_map<std::size_t, std::shared_ptr<State_t>> m_states_hash_map_ = {};
        absl::flat_hash_set<std::shared_ptr<State_t>> m_inconsistent_states_ = {};
        std::shared_ptr<State_t> m_start_state_ = nullptr;
        std::vector<std::shared_ptr<State_t>> m_goal_states_ = {};
        std::shared_ptr<Output_t> m_output_ = nullptr;

    public:
        explicit AmraStar(
            std::shared_ptr<PlanningInterface> planning_interface,
            std::shared_ptr<Setting> setting = nullptr)
            : m_setting_(std::move(setting)),
              m_planning_interface_(std::move(planning_interface)),
              m_expand_itr_(m_planning_interface_->GetNumHeuristics()),
              m_open_queues_(m_planning_interface_->GetNumHeuristics()),
              m_output_(std::make_shared<Output_t>()) {

            if (!m_setting_) { m_setting_ = std::make_shared<Setting>(); }

            std::size_t num_resolution_levels = m_planning_interface_->GetNumResolutionLevels();

            EnvState start_env_state = m_planning_interface_->GetStartState();
            m_start_state_ = std::make_shared<State_t>(
                m_plan_itr_,
                start_env_state,
                num_resolution_levels,
                m_planning_interface_->GetInResolutionLevelFlags(start_env_state),
                m_planning_interface_->GetHeuristicValues(start_env_state));
            GetState(start_env_state) = m_start_state_;

            const int num_goals = m_planning_interface_->GetNumGoals();
            m_goal_states_.reserve(num_goals);
            for (int i = 0; i < num_goals; ++i) {
                EnvState goal_env_state = m_planning_interface_->GetGoalState(i);
                m_goal_states_.push_back(
                    std::make_shared<State_t>(
                        m_plan_itr_,
                        goal_env_state,
                        num_resolution_levels,
                        m_planning_interface_->GetInResolutionLevelFlags(goal_env_state),
                        m_planning_interface_->GetHeuristicValues(goal_env_state)));
                GetState(goal_env_state) = m_goal_states_.back();
            }

            for (auto &queue: m_open_queues_) { queue.reserve(20000); }
        }

        std::shared_ptr<Output_t>
        Plan() {
            // check if the start state is already a goal
            if (int goal_index = m_planning_interface_->IsMetricGoal(m_start_state_->env_state);
                goal_index >= 0) {
                SaveOutput(m_start_state_, goal_index);
                return m_output_;
            }

            m_w1_ = m_setting_->w1_init;  // L43
            m_w2_ = m_setting_->w2_init;  // L43

            std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
            std::size_t num_resolution_levels = m_planning_interface_->GetNumResolutionLevels();
            m_expand_itr_.setZero();
            m_total_expand_itr_ = 1;

            m_plan_itr_++;
            for (auto &goal_state: m_goal_states_) { ReinitState(goal_state); }
            ReinitState(m_start_state_);                                    // L45
            m_start_state_->g_value = 0.;                                   // L44
            for (auto &open_queue: m_open_queues_) { open_queue.clear(); }  // L46-47
            m_inconsistent_states_.clear();
            m_inconsistent_states_.insert(m_start_state_);  // L48

            m_search_time_ = 0ns;
            std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
            while (m_search_time_ < m_setting_->time_limit &&
                   (m_w1_ >= m_setting_->w1_final && m_w2_ >= m_setting_->w2_final)) {  // L49
                if (m_setting_->log) {
                    m_output_->w1_values[m_plan_itr_] = m_w1_;
                    m_output_->w2_values[m_plan_itr_] = m_w2_;
                }
                for (auto &state: m_inconsistent_states_) {  // L50
                    state->RemoveFromClosed(0, m_planning_interface_->GetResolutionHeuristicIds(0));
                    InsertOrUpdate(state, 0, GetFvalue(state, 0));  // L51
                }
                m_inconsistent_states_.clear();  // L52
                // update other open queues using the anchor-level open queue
                for (auto &queue_item: m_open_queues_[0]) {  // L53
                    std::shared_ptr<State_t> &state = queue_item->state;
                    for (std::size_t heuristic_id = 1; heuristic_id < num_heuristics;
                         ++heuristic_id) {  // L54
                        const std::size_t resolution_level =
                            m_planning_interface_->GetResolutionAssignment(heuristic_id);
                        if (!state->InResolutionLevel(resolution_level)) { continue; }  // L55
                        state->RemoveFromClosed(
                            resolution_level,
                            m_planning_interface_->GetResolutionHeuristicIds(resolution_level));
                        InsertOrUpdate(state, heuristic_id, GetFvalue(state, heuristic_id));  // L56
                    }
                }

                // m_w1_ may be changed, so we need to update the open queues
                for (std::size_t heuristic_id = 0; heuristic_id < num_heuristics; ++heuristic_id) {
                    RebuildOpenQueue(heuristic_id);
                }

                // empty all close sets
                for (auto &[state_hashing, state]: m_states_hash_map_) {
                    for (std::size_t res_level = 0; res_level < num_resolution_levels;
                         ++res_level) {  // L57
                        state->RemoveFromClosed(
                            res_level,
                            m_planning_interface_->GetResolutionHeuristicIds(res_level));  // L58
                    }
                }

                // improve path
                std::chrono::nanoseconds elapsed_time;
                auto [goal_state, goal_index] = ImprovePath(start_time, elapsed_time);  // L59
                m_search_time_ += elapsed_time;
                start_time = std::chrono::system_clock::now();
                SaveOutput(goal_state, goal_index);  // L60

                // fail to find a solution or time out
                if (goal_state == nullptr || m_search_time_ > m_setting_->time_limit) { break; }

                // goal is reached or still have time to solve
                ERL_INFO(
                    "Solved with (w1, w2) = ({}, {}) | expansions = {} | time = {} sec | cost = {}",
                    m_w1_,
                    m_w2_,
                    common::EigenToNumPyFmtString(m_expand_itr_.transpose()).c_str(),
                    m_search_time_.count() / 1e9,
                    m_output_->plan_records[m_plan_itr_].cost);

                if (m_w1_ == m_setting_->w1_final && m_w2_ == m_setting_->w2_final) {
                    break;
                }  // L61-62
                m_w1_ = std::max(m_w1_ * m_setting_->w1_decay_factor, m_setting_->w1_final);  // L63
                m_w2_ = std::max(m_w2_ * m_setting_->w2_decay_factor, m_setting_->w2_final);  // L63
                m_plan_itr_++;
            }
            return m_output_;
        }

    private:
        std::pair<std::shared_ptr<State_t>, int>
        ImprovePath(
            const std::chrono::system_clock::time_point &start_time,
            std::chrono::nanoseconds &elapsed_time) {

            elapsed_time = 0ns;
            const std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
            auto &open_anchor = m_open_queues_[0];
            while (!open_anchor.empty() &&
                   (open_anchor.top()->f_value < std::numeric_limits<double>::infinity())) {  // L25
                elapsed_time = std::chrono::system_clock::now() - start_time;
                if (elapsed_time + m_search_time_ > m_setting_->time_limit) {
                    return {nullptr, -1};  // time out
                }

                for (std::size_t heuristic_id = 1; heuristic_id < num_heuristics; ++heuristic_id) {
                    // L26
                    if (open_anchor.empty()) { return {nullptr, -1}; }  // no solution
                    const double f_check = m_w2_ * open_anchor.top()->f_value;
                    if (auto min_goal_itr = std::min_element(  // get the goal of minimum g_value
                        m_goal_states_.begin(),
                        m_goal_states_.end(),
                        [](const std::shared_ptr<State_t> &a, const std::shared_ptr<State_t> &b) { return a->g_value < b->g_value; });
                    f_check >= (*min_goal_itr)->g_value) {
                        // no improvement can be made, return the goal with the minimum g_value
                        return {
                            *min_goal_itr,
                            static_cast<int>(std::distance(m_goal_states_.begin(), min_goal_itr))};
                    }

                    // state in the open queue of heuristic_id may be invalid when it is moved to
                    // CLOSE by another heuristic
                    auto &open = m_open_queues_[heuristic_id];
                    while (!open.empty() || !open_anchor.empty()) {
                        if (!open.empty() && open.top()->f_value <= f_check) {
                            std::shared_ptr<State_t> state = open.top()->state;
                            if (m_planning_interface_->ReachGoal(state->env_state)) {
                                return {state, -1};
                            }
                            open.pop();
                            if (!state->InOpened(
                                    heuristic_id,
                                    m_planning_interface_->GetResolutionAssignment(heuristic_id))) {
                                continue;  // state is moved to CLOSED by another heuristic
                            }
                            Expand(state, heuristic_id);
                            m_expand_itr_[static_cast<long>(heuristic_id)]++;
                            m_total_expand_itr_++;
                            break;
                        }
                        std::shared_ptr<State_t> state = open_anchor.top()->state;
                        if (m_planning_interface_->ReachGoal(state->env_state)) {
                            return {state, -1};
                        }
                        open_anchor.pop();
                        Expand(state, 0);
                        m_expand_itr_[0]++;
                        m_total_expand_itr_++;
                        break;
                    }
                }
            }
            return {nullptr, -1};  // no solution
        }

        void
        Expand(const std::shared_ptr<State_t> &parent, std::size_t heuristic_id) {

            // L4
            std::size_t res_level = m_planning_interface_->GetResolutionAssignment(heuristic_id);

            if (heuristic_id == 0) {
                // the heuristic for the anchor level must be consistent
                ERL_DEBUG_ASSERT(
                    !parent->InClosed(0),
                    "parent is already in anchor-level closed set.");
                parent->SetClosed(0, m_total_expand_itr_);  // anchor level, consistent heuristic
            } else {                                        // L5
                // comment the following assert because the heuristic may be inconsistent
                // ERL_DEBUG_ASSERT(!parent->InClosed(resolution_level), "parent is already in
                // CLOSED of resolution level {}.", int(resolution_level));
                ERL_DEBUG_ASSERT(
                    parent->InOpened(heuristic_id, res_level),  // L6 to L8
                    "parent is not in OPENED of heuristic {}.",
                    static_cast<int>(heuristic_id));
                // parent is also removed from other opened sets for the same resolution level
                parent->SetClosed(res_level, m_total_expand_itr_);
            }
            if (m_setting_->log) {
                ERL_ASSERTM(
                    m_output_->closed_states[m_total_expand_itr_]
                        .insert({res_level, parent->env_state.metric})
                        .second,
                    "state already exists in closed set.");
            }

            ERL_DEBUG_ASSERT(
                !m_planning_interface_->ReachGoal(parent->env_state),
                "should not expand from a goal parent.");
            auto successors = m_planning_interface_->GetSuccessors(parent->env_state, res_level);
            const std::size_t num_resolution_levels =
                m_planning_interface_->GetNumResolutionLevels();
            const std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
            for (auto &successor: successors) {  // L9
                std::shared_ptr<State_t> &child = GetState(successor.env_state);
                if (!child) {
                    child = std::make_shared<State_t>(
                        m_plan_itr_,
                        successor.env_state,
                        num_resolution_levels,
                        m_planning_interface_->GetInResolutionLevelFlags(successor.env_state),
                        m_planning_interface_->GetHeuristicValues(successor.env_state));
                }

                if (const double tentative_g_value = parent->g_value + successor.cost;
                    tentative_g_value < child->g_value) {                              // L10
                    child->g_value = tentative_g_value;                                // L11
                    child->SetParent(parent, successor.env_id, successor.action_idx);  // L12
                    // in anchor-level closed set, inconsistency detected, re-open it
                    if (child->InClosed(0)) {                  // L13
                        m_inconsistent_states_.insert(child);  // L14
                        if (m_setting_->log) {
                            m_output_->inconsistent_states[m_total_expand_itr_].push_back(
                                child->env_state.metric);
                        }
                        continue;
                    }
                    const double f0 = GetFvalue(child, 0);
                    InsertOrUpdate(child, 0, f0);                                // L16
                    for (std::size_t h_id = 1; h_id < num_heuristics; ++h_id) {  // L17
                        const std::size_t res_level_for_h_id =
                            m_planning_interface_->GetResolutionAssignment(h_id);         // L18
                        if (!child->InResolutionLevel(res_level_for_h_id)) { continue; }  // L19-20
                        if (child->InClosed(res_level_for_h_id)) { continue; }            // L21
                        const double f_h_id = GetFvalue(child, h_id);
                        if (f_h_id > m_w2_ * f0) { continue; }  // L22
                        InsertOrUpdate(child, h_id, f_h_id);    // L23
                    }
                }
            }
        }

        std::shared_ptr<State_t> &
        GetState(const EnvState &env_state) {
            return m_states_hash_map_[m_planning_interface_->StateHashing(env_state)];
        }

        void
        ReinitState(const std::shared_ptr<State_t> &state) const {
            if (state->plan_itr == m_plan_itr_) { return; }
            state->Reset();
            state->plan_itr = m_plan_itr_;
            // recompute h-value
            const std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
            for (std::size_t heuristic_id = 0; heuristic_id < num_heuristics; ++heuristic_id) {
                state->h_values[heuristic_id] =
                    (*m_planning_interface_->GetHeuristic(heuristic_id))(state->env_state);
            }
        }

        [[nodiscard]] Dtype
        GetFvalue(const std::shared_ptr<State_t> &state, const std::size_t heuristic_id) const {
            return state->g_value + m_w1_ * state->h_values[heuristic_id];
        }

        void
        InsertOrUpdate(
            const std::shared_ptr<State_t> &state,
            std::size_t heuristic_id,
            Dtype f_value) {
            if (std::size_t level = m_planning_interface_->GetResolutionAssignment(heuristic_id);
                state->InOpened(heuristic_id, level)) {
                // state is already in open, update its f-value
                (*state->open_queue_keys[heuristic_id])->f_value = f_value;
                m_open_queues_[heuristic_id].increase(state->open_queue_keys[heuristic_id]);
            } else {
                // state is not in open, insert it into open
                state->open_queue_keys[heuristic_id] = m_open_queues_[heuristic_id].push(
                    std::make_shared<PriorityQueueItem_t>(f_value, state));
                state->SetOpened(heuristic_id, m_total_expand_itr_);
                if (m_setting_->log) {
                    m_output_->opened_states[m_total_expand_itr_][heuristic_id].push_back(
                        state->env_state.metric);
                }
            }
        }

        void
        RebuildOpenQueue(std::size_t heuristic_id) {
            auto &open_queue = m_open_queues_[heuristic_id];
            PriorityQueue_t new_open_queue;
            new_open_queue.reserve(20000);
            for (auto &queue_item: open_queue) {
                queue_item->f_value = GetFvalue(queue_item->state, heuristic_id);
                queue_item->state->open_queue_keys[heuristic_id] = new_open_queue.push(queue_item);
            }
            open_queue.swap(new_open_queue);
        }

        /**
         * @brief Recover the path from the start state to the reached goal state
         * @param goal_state
         * @param goal_index
         */
        void
        RecoverPath(const std::shared_ptr<State_t> &goal_state, int goal_index) {
            m_output_->latest_plan_itr = m_plan_itr_;
            m_output_->w1_solve = m_w1_;
            m_output_->w2_solve = m_w2_;

            auto [plan_record_itr, inserted] =
                m_output_->plan_records.insert({m_plan_itr_, PlanRecord<Dtype, Dim>()});
            ERL_ASSERTM(inserted, "path already exists for plan iteration {}.", m_plan_itr_);

            auto &plan_record = plan_record_itr->second;

            plan_record.cost = goal_state->g_value;
            plan_record.goal_index = goal_index;

            std::shared_ptr<State_t> node = goal_state;
            if (m_planning_interface_->IsVirtualGoal(goal_state->env_state)) {
                // virtual goal state is used!
                node = node->parent;
                plan_record.goal_index = m_planning_interface_->IsMetricGoal(node->env_state);
                ERL_DEBUG_ASSERT(plan_record.goal_index >= 0, "goal index is invalid.");
            } else {
                plan_record.cost += m_planning_interface_->GetTerminalCost(goal_index);
            }

            ERL_DEBUG(
                "Reach goal[{}/{}] (metric: {}, grid: {}) from metric start {} at expansion_itr {} "
                "plan_itr {}.",
                plan_record.goal_index,
                m_planning_interface_->GetNumGoals(),
                common::EigenToNumPyFmtString(node->env_state.metric.transpose()),
                common::EigenToNumPyFmtString(node->env_state.grid.transpose()),
                common::EigenToNumPyFmtString(
                    m_planning_interface_->GetStartState().metric.transpose()),
                m_total_expand_itr_,
                m_plan_itr_);

            auto &actions_coords = plan_record.env_action_indices;
            actions_coords.clear();
            while (node->parent != nullptr) {
                actions_coords.emplace_back(node->env_level, node->action_idx);
                node = node->parent;
            }
            std::reverse(actions_coords.begin(), actions_coords.end());
            std::vector<std::vector<EnvState>> path_segments;
            path_segments.reserve(actions_coords.size());
            long num_path_states = 0;
            auto state = m_start_state_->env_state;
            for (auto &[env_level, action_idx]: actions_coords) {
                std::vector<EnvState> path_segment =
                    m_planning_interface_->GetPath(state, env_level, action_idx);
                if (path_segment.empty()) { continue; }
                path_segments.push_back(path_segment);
                num_path_states += static_cast<long>(path_segment.size());
                state = path_segment.back();
            }

            Eigen::Matrix<Dtype, Dim, Eigen::Dynamic> &path = plan_record.path;
            path.resize(Eigen::NoChange, num_path_states + 1);
            path.col(0) = m_planning_interface_->GetStartState().metric;
            long index = 1;
            for (auto &path_segment: path_segments) {
                const auto num_states = static_cast<long>(path_segment.size());
                for (long i = 0; i < num_states; ++i) {
                    path.col(index++) = path_segment[i].metric;
                }
            }
        }

        /**
         * @brief Save the output of the search
         * @param goal_state
         * @param goal_index
         */
        void
        SaveOutput(const std::shared_ptr<State_t> &goal_state, int goal_index) {
            if (goal_state != nullptr) { RecoverPath(goal_state, goal_index); }
            m_output_->num_heuristics = m_planning_interface_->GetNumHeuristics();
            m_output_->num_resolution_levels = m_planning_interface_->GetNumResolutionLevels();
            m_output_->num_expansions = m_total_expand_itr_;
            m_output_->search_time = static_cast<Dtype>(m_search_time_.count()) / 1.e9f;
        }
    };

    extern template class AmraStar<float, 2>;
    extern template class AmraStar<float, 3>;
    extern template class AmraStar<double, 2>;
    extern template class AmraStar<double, 3>;

}  // namespace erl::path_planning::amra_star
