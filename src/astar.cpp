#include "erl_search_planning/astar.hpp"

namespace erl::search_planning::astar {

    AStar::AStar(std::shared_ptr<PlanningInterface> planning_interface, std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)),
          m_planning_interface_(std::move(planning_interface)),
          m_output_(std::make_shared<Output>()) {

        if (!m_setting_) { m_setting_ = std::make_shared<Setting>(); }
        ERL_ASSERTM(m_setting_->eps > 0., "eps must be positive.");
        // initialize start node
        auto start_env_state = m_planning_interface_->GetStartState();
        auto &start = GetState(start_env_state);
        start = std::make_shared<State>(std::move(start_env_state), 0., m_planning_interface_->GetHeuristic(start_env_state), m_iterations_);
        m_current_ = start;
        m_start_state_ = start;
    }

    std::shared_ptr<Output>
    AStar::Plan() {
        if (static_cast<int>(m_reached_goal_indices_.size()) == m_planning_interface_->GetNumGoals()) { return m_output_; }  // all reachable goals are reached
        // check if the start state is a goal
        if (const int goal_index = m_planning_interface_->IsMetricGoal(m_start_state_->env_state); goal_index >= 0) {
            if (!m_reached_goal_indices_.contains(goal_index)) {  // such result is not returned before
                RecoverPath(m_start_state_, goal_index);
                m_reached_goal_indices_.insert(goal_index);
                return m_output_;
            }
        }

        if (m_astar_states_.size() > 1) {         // Plan() is already called, resume the search
            if (m_output_->goal_index >= 0) {     // previous call of Plan() reached a goal
                if (m_priority_queue_.empty()) {  // Open is empty, the free space is fully explored.
                    // get all remaining goals and their costs
                    std::vector<std::tuple<int, double, std::shared_ptr<State>>> goal_costs_indices;
                    for (int i = 0, num_goals = m_planning_interface_->GetNumGoals(); i < num_goals; ++i) {
                        if (m_reached_goal_indices_.contains(i)) { continue; }
                        if (const auto goal_state = GetState(m_planning_interface_->GetGoalState(i)); goal_state != nullptr) {
                            goal_costs_indices.emplace_back(i, goal_state->g_value, goal_state);
                        }
                    }
                    if (goal_costs_indices.empty()) { return m_output_; }  // all reachable goals are reached
                    // order remaining goals by cost in ascending order
                    std::sort(goal_costs_indices.begin(), goal_costs_indices.end(), [](const auto &a, const auto &b) {
                        return std::get<1>(a) < std::get<1>(b);
                    });
                    // get the goal with the smallest cost
                    const auto [goal_index, cost, goal_state] = goal_costs_indices.front();
                    RecoverPath(goal_state, goal_index);
                    m_reached_goal_indices_.insert(goal_index);
                    return m_output_;
                }
                m_current_ = m_priority_queue_.top()->state;
                m_priority_queue_.pop();
            }
        }

        while (m_setting_->max_num_iterations < 0 || static_cast<long>(m_iterations_) < m_setting_->max_num_iterations) {
            // check if done
            if (m_planning_interface_->ReachGoal(m_current_->env_state)) {
                RecoverPath(m_current_, -1);
                LogStates();
                // if a goal was reached in privious call of Plan(), the goal is in CLOSED set and we should not get it again.
                ERL_DEBUG_ASSERT(m_reached_goal_indices_.insert(m_output_->goal_index).second, "Goal {} is reached multiple times.", m_output_->goal_index);
                return m_output_;
            }

            // increase the number of iterations
            m_iterations_++;

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

    void
    AStar::Expand() {
        // get successors
        auto successors = m_planning_interface_->GetSuccessors(m_current_->env_state);

        // process successors
        for (auto &successor: successors) {
            // get child node
            auto &child = GetState(successor.env_state);
            if (!child) { child = std::make_shared<State>(successor.env_state, m_planning_interface_->GetHeuristic(successor.env_state)); }  // new node

            double tentative_g_value = m_current_->g_value + successor.cost;
            double old_g_value = child->g_value;
            if (tentative_g_value < child->g_value) {

                child->parent = m_current_;
                child->action_coords = successor.action_coords;
                child->g_value = tentative_g_value;
                double f_value = tentative_g_value + m_setting_->eps * child->h_value;

                if (child->IsOpened()) {  // already in open set
                    (*child->heap_key)->f_value = f_value;
                    m_priority_queue_.increase(child->heap_key);
                } else if (child->IsClosed()) {  // already in closed set
                    // unexpected in A* with consistent heuristic because closed node has optimal g value
                    // this happens if m_planning_interface_ does not provide consistent heuristic function
                    if (m_setting_->reopen_inconsistent) {
                        child->heap_key = m_priority_queue_.push(std::make_shared<PriorityQueueItem>(f_value, child));
                        child->iteration_closed = 0;
                    } else {
                        ERL_WARN_ONCE(
                            "Inconsistent heuristic function is detected! (m_reopen_inconsistent_ is false). g_value changes from {} to {}\n",
                            old_g_value,
                            tentative_g_value);
                        if (m_setting_->log) { m_output_->inconsistent_list[m_iterations_].push_back(child->env_state->metric); }
                    }
                } else {
                    // new node
                    child->heap_key = m_priority_queue_.push(std::make_shared<PriorityQueueItem>(f_value, child));
                    child->iteration_opened = m_iterations_;
                }
            }
        }
    }

    void
    AStar::RecoverPath(const std::shared_ptr<State> &goal_state, const int goal_index) {
        m_output_->cost = goal_state->g_value;  // terminal cost is included in g_value if goal_state is virtual goal
        m_output_->goal_index = goal_index;
        std::shared_ptr<State> node = goal_state;
        if (m_planning_interface_->IsVirtualGoal(node->env_state)) {
            node = node->parent;
            m_output_->goal_index = m_planning_interface_->IsMetricGoal(node->env_state);
            ERL_DEBUG_ASSERT(m_output_->goal_index >= 0, "goal index is invalid.");
        } else {
            ERL_DEBUG_ASSERT(goal_index >= 0, "goal index is invalid when goal_state is not virtual goal.");
            m_output_->cost += m_planning_interface_->GetTerminalCost(goal_index);
        }

        if (node == nullptr) {}
        ERL_DEBUG_ASSERT(node != nullptr, "Goal state is not found.");

        ERL_DEBUG(
            "Reach goal[{}/{}] (metric: {}, grid: {}) from metric start {} at iteration {}.",
            m_output_->goal_index,
            m_planning_interface_->GetNumGoals(),
            common::EigenToNumPyFmtString(node->env_state->metric.transpose()),
            common::EigenToNumPyFmtString(node->env_state->grid.transpose()),
            common::EigenToNumPyFmtString(m_planning_interface_->GetStartState()->metric.transpose()),
            m_iterations_);

        m_output_->action_coords.clear();
        while (node->parent != nullptr) {
            m_output_->action_coords.push_front(node->action_coords);
            node = node->parent;
        }
        std::vector<std::vector<std::shared_ptr<env::EnvironmentState>>> path_segments;
        long num_path_states = 0;
        auto state = GetState(m_start_state_->env_state)->env_state;
        for (auto &action_coords: m_output_->action_coords) {
            std::vector<std::shared_ptr<env::EnvironmentState>> path_segment = m_planning_interface_->GetPath(state, action_coords);
            if (path_segment.empty()) { continue; }
            path_segments.push_back(path_segment);
            num_path_states += static_cast<long>(path_segment.size());
            state = path_segment.back();
        }
        m_output_->path.resize(state->metric.size(), num_path_states + 1);
        m_output_->path.col(0) = m_planning_interface_->GetStartState()->metric;
        long index = 1;
        for (auto &path_segment: path_segments) {
            const auto num_states = static_cast<long>(path_segment.size());
            for (long i = 0; i < num_states; ++i) { m_output_->path.col(index++) = path_segment[i]->metric; }
        }
    }

    void
    AStar::LogStates() const {
        if (!m_setting_->log) { return; }
        for (auto &[state_hashing, astar_state]: m_astar_states_) {
            if (astar_state->IsOpened()) {
                m_output_->opened_list[astar_state->iteration_opened].push_back(astar_state->env_state->metric);
            } else if (astar_state->IsClosed()) {
                m_output_->closed_list[astar_state->iteration_closed] = astar_state->env_state->metric;
            }
        }

        // for (auto &astar_state: m_astar_states_) {
        //     if (astar_state == nullptr) { continue; }
        //     if (astar_state->IsOpened()) {
        //         m_output_->opened_list[astar_state->iteration_opened].push_back(astar_state->env_state->metric);
        //     } else if (astar_state->IsClosed()) {
        //         m_output_->closed_list[astar_state->iteration_closed] = astar_state->env_state->metric;
        //     }
        // }
    }
}  // namespace erl::search_planning::astar
