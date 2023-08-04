#include <chrono>
#include <memory>
#include "erl_search_planning/amra_star.hpp"

namespace erl::search_planning::amra_star {

    void
    Output::Save(const std::filesystem::path &file_path) const {
        std::ofstream ofs(file_path);
        ERL_ASSERTM(ofs.is_open(), "Failed to open file: %s", file_path.string().c_str());
        long n = path.cols();
        ofs << "AMRA* solution" << std::endl
            << "goal_index: " << goal_index << std::endl
            << "cost: " << cost << std::endl
            << "w1_solve: " << w1_solve << std::endl
            << "w2_solve: " << w2_solve << std::endl
            << "search_time: " << search_time << std::endl
            << "num waypoints: " << n << std::endl
            << "path: " << std::endl
            << "x, y" << std::endl;
        for (int i = 0; i < n; ++i) { ofs << path(0, i) << ", " << path(1, i) << std::endl; }
        ofs << "action_coords: " << std::endl;
        for (const auto &action_coord: action_coords) {
            std::size_t m = action_coord.size();
            ERL_ASSERTM(m > 0, "action_coord is empty");
            ofs << action_coord[0];
            for (std::size_t i = 1; i < m; ++i) { ofs << ", " << action_coord[i]; }
            ofs << std::endl;
        }
        ofs << "opened_states: " << std::endl << "plan_itr, heuristic_id, x, y" << std::endl;
        for (const auto &[plan_itr, opened_states_at_plan_itr]: opened_states) {
            for (const auto &[heuristic_id, opened_states_at_heuristic_id]: opened_states_at_plan_itr) {
                for (const auto &state: opened_states_at_heuristic_id) {
                    ofs << plan_itr << ", " << heuristic_id << ", " << state[0] << ", " << state[1] << std::endl;
                }
            }
        }
        ofs << "closed_states: " << std::endl << "plan_itr, action_resolution_level, x, y" << std::endl;
        for (const auto &[plan_itr, closed_states_at_plan_itr]: closed_states) {
            for (const auto &[action_resolution_level, closed_states_at_action_resolution_level]: closed_states_at_plan_itr) {
                for (const auto &state: closed_states_at_action_resolution_level) {
                    ofs << plan_itr << ", " << action_resolution_level << ", " << state[0] << ", " << state[1] << std::endl;
                }
            }
        }
        ofs << "inconsistent_states: " << std::endl << "plan_itr, x, y" << std::endl;
        for (const auto &[plan_itr, inconsistent_states_at_plan_itr]: inconsistent_states) {
            for (const auto &state: inconsistent_states_at_plan_itr) { ofs << plan_itr << ", " << state[0] << ", " << state[1] << std::endl; }
        }
        ofs.close();
    }

    AMRAStar::AMRAStar(std::shared_ptr<PlanningInterfaceMultiResolutions> planning_interface, std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)),
          m_planning_interface_(std::move(planning_interface)),
          m_expand_itr_(m_planning_interface_->GetNumHeuristics()),
          m_open_queues_(m_planning_interface_->GetNumHeuristics()),
          m_output_(std::make_shared<Output>()) {

        if (!m_setting_) { m_setting_ = std::make_shared<Setting>(); }

        std::size_t num_resolution_levels = m_planning_interface_->GetNumResolutionLevels();

        auto start_env_state = m_planning_interface_->GetStartState();
        m_start_state_ = std::make_shared<State>(
            m_plan_itr_,
            start_env_state,
            num_resolution_levels,
            m_planning_interface_->GetInResolutionLevelFlags(start_env_state),
            m_planning_interface_->GetHeuristicValues(start_env_state));
        GetState(start_env_state) = m_start_state_;

        int num_goals = m_planning_interface_->GetNumGoals();
        m_goal_states_.reserve(num_goals);
        for (int i = 0; i < num_goals; ++i) {
            auto goal_env_state = m_planning_interface_->GetGoalState(i);
            m_goal_states_.push_back(std::make_shared<State>(
                m_plan_itr_,
                goal_env_state,
                num_resolution_levels,
                m_planning_interface_->GetInResolutionLevelFlags(goal_env_state),
                m_planning_interface_->GetHeuristicValues(goal_env_state)));
            GetState(goal_env_state) = m_goal_states_.back();
        }

        for (auto &queue : m_open_queues_) { queue.reserve(20000); }
    }

    std::shared_ptr<Output>
    AMRAStar::Plan() {
        int goal_index = m_planning_interface_->ReachGoal(m_start_state_->env_state);
        if (goal_index >= 0) {
            RecoverPath(goal_index);
            return m_output_;
        }

        m_w1_ = m_setting_->w1_init;  // L43
        m_w2_ = m_setting_->w2_init;  // L43

        std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
        std::size_t num_resolution_levels = m_planning_interface_->GetNumResolutionLevels();
        m_expand_itr_.setZero();
        m_expand_itr_[0] = 1;
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
        while (m_search_time_ < m_setting_->time_limit && (m_w1_ >= m_setting_->w1_final && m_w2_ >= m_setting_->w2_final)) {  // L49
            for (auto &state: m_inconsistent_states_) {  // L50
                state->RemoveFromClosed(0, m_planning_interface_->GetResolutionHeuristicIds(0));
                InsertOrUpdate(state, 0, GetKeyValue(state, 0));  // L51
            }
            m_inconsistent_states_.clear();  // L52
            // update other open queues using the anchor-level open queue
            for (auto &queue_item: m_open_queues_[0]) {  // L53
                std::shared_ptr<State> &state = queue_item->state;
                for (std::size_t heuristic_id = 1; heuristic_id < num_heuristics; ++heuristic_id) {  // L54
                    std::size_t resolution_level = m_planning_interface_->GetResolutionAssignment(heuristic_id);
                    if (!state->InResolutionLevel(resolution_level)) { continue; }  // L55
                    state->RemoveFromClosed(resolution_level, m_planning_interface_->GetResolutionHeuristicIds(resolution_level));
                    InsertOrUpdate(state, heuristic_id, GetKeyValue(state, heuristic_id));  // L56
                }
            }

            // m_w1_ may be changed, so we need to update the open queues
            for (std::size_t heuristic_id = 0; heuristic_id < num_heuristics; ++heuristic_id) { RebuildOpenQueue(heuristic_id); }

            // empty all close sets
            for (auto &item: m_states_hash_map_) {
                for (std::size_t res_level = 0; res_level < num_resolution_levels; ++res_level) {                           // L57
                    item.second->RemoveFromClosed(res_level, m_planning_interface_->GetResolutionHeuristicIds(res_level));  // L58
                }
            }

            // improve path
            std::chrono::nanoseconds elapsed_time;
            goal_index = ImprovePath(start_time, elapsed_time);  // L59
            m_search_time_ += elapsed_time;
            start_time = std::chrono::system_clock::now();
            if (goal_index < 0 || m_search_time_ > m_setting_->time_limit) { break; }  // fail to find a solution or time out
            RecoverPath(goal_index);                                                   // L60
            ERL_INFO(
                "Solved with (w1, w2) = (%f, %f) | expansions = %s | time = %f sec | cost = %f",
                m_w1_,
                m_w2_,
                common::EigenToNumPyFmtString(m_expand_itr_.transpose()).c_str(),
                m_search_time_.count() / 1e9,
                m_output_->cost);

            if (m_w1_ == m_setting_->w1_final && m_w2_ == m_setting_->w2_final) { break; }  // L61-62
            m_w1_ = std::max(m_w1_ * m_setting_->w1_decay_factor, m_setting_->w1_final);    // L63
            m_w2_ = std::max(m_w2_ * m_setting_->w2_decay_factor, m_setting_->w2_final);    // L63
            m_plan_itr_++;
        }
        return m_output_;
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
                    m_open_queues_[heuristic_id].pop();
                    int goal_index = m_planning_interface_->ReachGoal(state->env_state);
                    if (goal_index >= 0) { return goal_index; }
                    Expand(state, heuristic_id);
                    m_expand_itr_[long(heuristic_id)]++;
                    m_total_expand_itr_++;
                } else {
                    std::shared_ptr<State> state = m_open_queues_[0].top()->state;
                    m_open_queues_[0].pop();
                    int goal_index = m_planning_interface_->ReachGoal(state->env_state);
                    if (goal_index >= 0) { return goal_index; }
                    Expand(state, 0);
                    m_expand_itr_[0]++;
                    m_total_expand_itr_++;
                }
            }
        }
        return -1;
    }

    void
    AMRAStar::Expand(const std::shared_ptr<State> &parent, std::size_t heuristic_id) {

        std::size_t resolution_level = m_planning_interface_->GetResolutionAssignment(heuristic_id);  // L4
        if (heuristic_id == 0) {
            ERL_DEBUG_ASSERT(!parent->InClosed(0), "parent is already in anchor-level closed set.");
            parent->SetClosed(0, m_total_expand_itr_);  // anchor level, consistent heuristic
        } else {                                        // L5
            ERL_DEBUG_ASSERT(!parent->InClosed(resolution_level), "parent is already in closed set of resolution level %d.", int(resolution_level));
            ERL_DEBUG_ASSERT(parent->InOpened(heuristic_id, resolution_level), "parent is not in opened set of heuristic %d.", int(heuristic_id));
            // L6 to L8
            parent->SetClosed(resolution_level, m_total_expand_itr_);  // parent is also removed from other opened sets assigned to the same resolution level
        }
        if (m_setting_->log) { m_output_->closed_states[m_plan_itr_][resolution_level].push_back(parent->env_state->metric); }

        ERL_DEBUG_ASSERT(m_planning_interface_->ReachGoal(parent->env_state) < 0, "should not expand from a goal parent.");
        std::vector<env::Successor> successors = m_planning_interface_->GetSuccessors(parent->env_state, resolution_level);
        std::size_t num_resolution_levels = m_planning_interface_->GetNumResolutionLevels();
        std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
        for (auto &successor: successors) {  // L9
            std::shared_ptr<State> &child = GetState(successor.env_state);
            if (!child) {
                child.reset(new State(
                    m_plan_itr_,
                    successor.env_state,
                    num_resolution_levels,
                    m_planning_interface_->GetInResolutionLevelFlags(successor.env_state),
                    m_planning_interface_->GetHeuristicValues(successor.env_state)));
            }

            double tentative_g_value = parent->g_value + successor.cost;
            if (tentative_g_value < child->g_value) {               // L10
                child->g_value = tentative_g_value;                 // L11
                child->SetParent(parent, successor.action_coords);  // L12
                // in anchor-level closed set, inconsistency detected, re-open it
                if (child->InClosed(0)) {                  // L13
                    m_inconsistent_states_.insert(child);  // L14
                    if (m_setting_->log) { m_output_->inconsistent_states[m_plan_itr_].push_back(child->env_state->metric); }
                    continue;
                }
                double f0 = GetKeyValue(child, 0);
                InsertOrUpdate(child, 0, f0);                                                      // L16
                for (std::size_t h_id = 1; h_id < num_heuristics; ++h_id) {                        // L17
                    std::size_t res_level = m_planning_interface_->GetResolutionAssignment(h_id);  // L18
                    if (!child->InResolutionLevel(res_level)) { continue; }                        // L19-20
                    if (child->InClosed(res_level)) { continue; }                                  // L21
                    double f_h_id = GetKeyValue(child, h_id);
                    if (f_h_id > m_w2_ * f0) { continue; }  // L22
                    InsertOrUpdate(child, h_id, f_h_id);    // L23
                }
            }
        }
    }

    void
    AMRAStar::RecoverPath(int goal_index) {
        m_output_->w1_solve = m_w1_;
        m_output_->w2_solve = m_w2_;
        m_output_->search_time = double(m_search_time_.count()) / 1.e9;

        std::shared_ptr<State> goal_state = GetState(m_planning_interface_->GetGoalState(goal_index));
        m_output_->cost = goal_state->g_value;

        std::shared_ptr<State> node;
        if (m_planning_interface_->IsVirtualGoal(goal_state->env_state)) {  // virtual goal state is used!
            auto true_goal_env_state = m_planning_interface_->GetPath(goal_state->env_state, goal_state->action_coords)[0];
            goal_index = m_planning_interface_->IsMetricGoal(true_goal_env_state);  // get the true goal index
            node = GetState(true_goal_env_state);
        } else {
            node = goal_state;
        }
        m_output_->goal_index = goal_index;

        ERL_DEBUG(
            "Reach goal[%d/%d] (metric: %s, grid: %s) from metric start %s at expansion_itr %lu plan_itr %u.",
            goal_index,
            m_planning_interface_->GetNumGoals(),
            common::EigenToNumPyFmtString(node->env_state->metric.transpose()).c_str(),
            common::EigenToNumPyFmtString(node->env_state->grid.transpose()).c_str(),
            common::EigenToNumPyFmtString(m_planning_interface_->GetStartState()->metric.transpose()).c_str(),
            m_total_expand_itr_,
            m_plan_itr_);

        m_output_->action_coords.clear();
        while (node->parent != nullptr) {
            m_output_->action_coords.push_front(node->action_coords);
            node = node->parent;
        }
        std::vector<std::vector<std::shared_ptr<env::EnvironmentState>>> path_segments;
        path_segments.reserve(m_output_->action_coords.size());
        long num_path_states = 0;
        auto state = GetState(m_start_state_->env_state)->env_state;
        for (auto &action_id: m_output_->action_coords) {
            auto path_segment = m_planning_interface_->GetPath(state, action_id);
            if (path_segment.empty()) { continue; }
            path_segments.push_back(path_segment);
            num_path_states += long(path_segment.size());
            state = path_segment.back();
        }
        m_output_->path.resize(state->metric.size(), num_path_states + 1);
        m_output_->path.col(0) = m_planning_interface_->GetStartState()->metric;
        long index = 1;
        for (auto &path_segment: path_segments) {
            auto num_states = long(path_segment.size());
            for (long i = 0; i < num_states; ++i) { m_output_->path.col(index++) = path_segment[i]->metric; }
        }
    }
}  // namespace erl::search_planning::amra_star
