#include <chrono>
#include <cstddef>
#include <memory>
#include "erl_search_planning/amra_star.hpp"

namespace erl::search_planning::amra_star {

    std::shared_ptr<Output>
    AMRAStar::Plan() {
        if (int goal_index = m_planning_interface_->IsGoal(m_start_state_->env_state) >= 0) {
            RecoverPath(goal_index);
            return m_output_;
        }

        m_w1_ = m_setting_->w1_init;  // L43
        m_w2_ = m_setting_->w2_init;  // L43
        m_w1_solve_ = -1.;
        m_w2_solve_ = -1.;

        std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
        m_expand_itr_.setZero();
        m_total_expand_itr_ = 0;

        m_plan_itr_++;
        for (auto &goal_state: m_goal_states_) { ReinitState(goal_state); }
        ReinitState(m_start_state_);                                    // L45
        m_start_state_->g_value = 0.;                                   // L44
        for (auto &open_queue: m_open_queues_) { open_queue.clear(); }  // L46-47
        m_inconsistent_states_.clear();
        m_inconsistent_states_.insert(m_start_state_);                  // L48

        m_search_time_ = 0ns;
        while (m_search_time_ < m_setting_->time_limit && (m_w1_ >= m_setting_->w1_final && m_w2_ >= m_setting_->w2_final)) {  // L49
            std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
            for (auto &state: m_inconsistent_states_) {                                                                        // L50
                state->RemoveFromClosed(0);
                InsertOrUpdate(state, 0, GetKeyValue(state, 0));                                                               // L51
            }
            m_inconsistent_states_.clear();                                                                                    // L52
            // update other open queues using the anchor-level open queue
            for (auto &queue_item: m_open_queues_[0]) {                                              // L53
                std::shared_ptr<State> &state = queue_item->state;
                for (std::size_t heuristic_id = 1; heuristic_id < num_heuristics; ++heuristic_id) {  // L54
                    uint8_t resolution_level = m_planning_interface_->GetResolutionAssignment(heuristic_id);
                    if (!state->InResolutionLevel(resolution_level)) { continue; }                   // L55
                    state->RemoveFromClosed(resolution_level);
                    InsertOrUpdate(state, heuristic_id, GetKeyValue(state, heuristic_id));           // L56
                }
            }

            // m_w1_ may be changed, so we need to update the open queues
            for (std::size_t heuristic_id = 0; heuristic_id < num_heuristics; ++heuristic_id) { RebuildOpenQueue(heuristic_id); }

            // empty all close sets
            for (auto &item: m_states_hash_map_) { item.second->RemoveFromAllClosed(); }  // L57-58

            // improve path
            std::chrono::nanoseconds elapsed_time;
            int goal_index = ImprovePath(start_time, elapsed_time);                    // L59
            m_search_time_ += elapsed_time;
            if (goal_index < 0 || m_search_time_ > m_setting_->time_limit) { break; }  // fail to find a solution or time out
            RecoverPath(goal_index);                                                   // L60
            ERL_DEBUG(
                "Solution found with (%f, %f) | expansions = %s | time = %f sec",
                m_w1_,
                m_w2_,
                common::EigenToNumPyFmtString(m_expand_itr_.transpose()).c_str(),
                m_search_time_.count() / 1e9);

            if (m_w1_ == m_setting_->w1_final && m_w2_ == m_setting_->w2_final) { break; }  // L61-62
            m_w1_ = std::max(m_w1_ * m_setting_->w1_decay_factor, m_setting_->w1_final);    // L63
            m_plan_itr_++;
        }
    }

    int
    AMRAStar::ImprovePath(const std::chrono::system_clock::time_point &start_time, std::chrono::nanoseconds &elapsed_time) {
        elapsed_time = 0ns;
        std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
        while (!m_open_queues_[0].empty() && (m_open_queues_[0].top()->f_value < std::numeric_limits<double>::max())) {  // L25
            elapsed_time = std::chrono::system_clock::now() - start_time;
            if (elapsed_time + m_search_time_ > m_setting_->time_limit) { return -1; }

            for (std::size_t heuristic_id = 1; heuristic_id < num_heuristics; ++heuristic_id) {  // L26
                if (m_open_queues_[0].empty()) { return -1; }
                double f_check = m_w2_ * m_open_queues_[0].top()->f_value;
                auto min_goal_itr =
                    std::min_element(m_goal_states_.begin(), m_goal_states_.end(), [](const std::shared_ptr<State> &a, const std::shared_ptr<State> &b) {
                        return a->g_value < b->g_value;
                    });
                double min_goal_g_value = (*min_goal_itr)->g_value;
                if (f_check >= min_goal_g_value) { return int(std::distance(m_goal_states_.begin(), min_goal_itr)); }

                if (!m_open_queues_[heuristic_id].empty() && m_open_queues_[heuristic_id].top()->f_value <= f_check) {
                    std::shared_ptr<State> state = m_open_queues_[heuristic_id].top()->state;
                    if (int goal_index = m_planning_interface_->IsGoal(state->env_state) >= 0) { return goal_index; }
                    Expand(state, heuristic_id);
                    m_expand_itr_[long(heuristic_id)]++;
                    m_total_expand_itr_++;
                } else {
                    std::shared_ptr<State> state = m_open_queues_[0].top()->state;
                    if (int goal_index = m_planning_interface_->IsGoal(state->env_state) >= 0) { return goal_index; }
                    Expand(state, 0);
                    m_expand_itr_[0]++;
                    m_total_expand_itr_++;
                }
            }
        }
    }

    void
    AMRAStar::Expand(const std::shared_ptr<State> &parent, std::size_t heuristic_id) {
        uint8_t resolution_level = m_planning_interface_->GetResolutionAssignment(heuristic_id);  // L4
        if (heuristic_id == 0) {
            ERL_DEBUG_ASSERT(!parent->InClosed(0), "parent is already in anchor-level closed set.");
            parent->SetClosed(0, m_total_expand_itr_);  // anchor level, consistent heuristic
        } else {                                        // L5
            ERL_DEBUG_ASSERT(!parent->InClosed(resolution_level), "parent is already in closed set of resolution level %d.", int(resolution_level));
            ERL_DEBUG_ASSERT(parent->InOpened(heuristic_id, resolution_level), "parent is not in opened set of heuristic %d.", int(heuristic_id));
            // L6 to L8
            parent->SetClosed(resolution_level, m_total_expand_itr_);  // parent is also removed from other opened sets assigned to the same resolution level
        }

        ERL_DEBUG_ASSERT(!m_planning_interface_->IsGoal(parent->env_state), "should not expand from a goal parent.");
        std::vector<PlanningInterfaceMultiResolutions::Successor> successors = m_planning_interface_->GetSuccessors(parent->env_state, resolution_level);
        std::size_t num_resolution_levels = m_planning_interface_->GetNumResolutionLevels();
        std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
        for (auto &successor: successors) {  // L9
            std::shared_ptr<State> &child = GetState(successor.env_state);
            if (!child) {
                child.reset(new State(
                    m_plan_itr_,
                    successor.env_state,
                    num_resolution_levels,
                    m_planning_interface_->GetContainedResolutionLevels(successor.env_state),
                    m_planning_interface_->GetHeuristicValues(successor.env_state)));
            }

            double tentative_g_value = parent->g_value + successor.cost;
            if (tentative_g_value < child->g_value) {                                              // L10
                child->g_value = tentative_g_value;                                                // L11
                child->SetParent(parent, successor.action_id, successor.action_resolution_level);  // L12
                if (child->InClosed(0)) {                                                          // L13
                    m_inconsistent_states_.insert(child);                                          // L14
                    continue;  // in anchor-level closed set, inconsistency detected, re-open it
                }
                double f0 = GetKeyValue(child, 0);
                InsertOrUpdate(child, 0, f0);                                                  // L16
                for (std::size_t h_id = 1; h_id < num_heuristics; ++h_id) {                    // L17
                    uint8_t res_level = m_planning_interface_->GetResolutionAssignment(h_id);  // L18
                    if (!child->InResolutionLevel(res_level)) { continue; }                    // L19-20
                    if (child->InClosed(res_level)) { continue; }                              // L21
                    double f_h_id = GetKeyValue(child, h_id);
                    if (f_h_id > m_w2_ * f0) { continue; }                                     // L22
                    InsertOrUpdate(child, h_id, f_h_id);                                       // L23
                }
            }
        }
    }

    void
    AMRAStar::RecoverPath(int goal_index) {
        m_w1_solve_ = m_w1_;
        m_w2_solve_ = m_w2_;

        std::size_t num_goals = m_planning_interface_->GetNumGoals();

    }
}  // namespace erl::search_planning::amra_star
