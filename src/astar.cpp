#include "erl_search_planning/astar.hpp"

namespace erl::search_planning::astar {

    AStar::AStar(std::shared_ptr<PlanningInterface> planning_interface, std::shared_ptr<Setting> setting)
        : m_setting_(std::move(setting)),
          m_planning_interface_(std::move(planning_interface)),
          m_output_(std::make_shared<Output>()) {

        if (!m_setting_) { m_setting_ = std::make_shared<Setting>(); }

        // initialize start node
        auto start_env_state = m_planning_interface_->GetStartState();
        double h = m_planning_interface_->GetHeuristic(start_env_state);
        auto &start = GetState(start_env_state);
        start.reset(new State(std::move(start_env_state), 0., h, m_iterations_));
        m_current_ = start;
        m_start_state_ = start;
    }

    std::shared_ptr<Output>
    AStar::Plan() {
        if (m_planned_) { return m_output_; }

        while (m_setting_->max_num_iterations < 0 || long(m_iterations_) < m_setting_->max_num_iterations) {
            // check if done
            int reached_goal_index = m_planning_interface_->ReachGoal(m_current_->env_state);
            if (reached_goal_index >= 0) {
                RecoverPath(reached_goal_index);
                LogStates();
                m_planned_ = true;
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
        m_planned_ = true;
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
            if (!child) {  // new node
                child.reset(new State(successor.env_state, m_planning_interface_->GetHeuristic(successor.env_state)));
            }

            double tentative_g_value = m_current_->g_value + successor.cost;
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
                        ERL_WARN_ONCE("Inconsistent heuristic function is detected! (m_reopen_inconsistent_ is false).\n");
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
    AStar::RecoverPath(int goal_index) {
        std::shared_ptr<State> goal_state = GetState(m_planning_interface_->GetGoalState(goal_index));
        // If there is multiple goals with nonzero terminal costs, the planning interface will handle it to make g_value include the terminal cost.
        m_output_->cost = goal_state->g_value;

        std::shared_ptr<State> node;
        if (m_planning_interface_->IsVirtualGoal(goal_state->env_state)) {  // virtual goal state is used!
            auto true_goal_env_state = m_planning_interface_->GetPath(goal_state->env_state, goal_state->action_coords)[0];
            goal_index = m_planning_interface_->IsMetricGoal(true_goal_env_state);
            node = GetState(true_goal_env_state);
        } else {
            node = goal_state;
        }
        m_output_->goal_index = goal_index;

        ERL_DEBUG(
            "Reach goal[%d/%d] (metric: %s, grid: %s) from metric start %s at iteration %lu.",
            goal_index,
            m_planning_interface_->GetNumGoals(),
            common::EigenToNumPyFmtString(node->env_state->metric.transpose()).c_str(),
            common::EigenToNumPyFmtString(node->env_state->grid.transpose()).c_str(),
            common::EigenToNumPyFmtString(m_planning_interface_->GetStartState()->metric.transpose()).c_str(),
            m_iterations_);

        m_output_->action_coords.clear();
        while (node->parent != nullptr) {
            m_output_->action_coords.push_front(node->action_coords);
            node = node->parent;
        }
        std::vector<std::vector<std::shared_ptr<env::EnvironmentState>>> path_segments;
        long num_path_states = 0;
        auto state = GetState(m_start_state_->env_state)->env_state;
        for (auto &action_id: m_output_->action_coords) {
            auto path_segment = m_planning_interface_->GetPath(state, action_id);
            if (path_segment.empty()) { continue; }
            path_segments.push_back(path_segment);
            num_path_states += long(path_segment.size());
            state = path_segment.back();
        }
        m_output_->path.resize(state->metric.size(), num_path_states + 2);
        m_output_->path.col(0) = m_planning_interface_->GetStartState()->metric;
        long index = 1;
        for (auto &path_segment: path_segments) {
            auto num_states = long(path_segment.size());
            for (long i = 0; i < num_states; ++i) { m_output_->path.col(index++) = path_segment[i]->metric; }
        }
        m_output_->path.col(index) = m_planning_interface_->GetGoalState(goal_index)->metric;
    }

    void
    AStar::LogStates() const {
        if (!m_setting_->log) { return; }
        for (auto &[state_hashing, astar_state]: m_states_hash_map_) {
            if (astar_state->IsOpened()) {
                m_output_->opened_list[astar_state->iteration_opened].push_back(astar_state->env_state->metric);
            } else if (astar_state->IsClosed()) {
                m_output_->closed_list[astar_state->iteration_closed] = astar_state->env_state->metric;
            }
        }
    }

}  // namespace erl::search_planning::astar
