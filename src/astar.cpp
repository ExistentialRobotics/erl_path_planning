#include "erl_search_planning/astar.hpp"
#include <Eigen/src/Core/Matrix.h>

#include <memory>

namespace erl::search_planning::astar {

    AStar::AStar(
        const std::shared_ptr<PlanningInterface> &planning_interface,
        double eps,
        long max_num_iterations,
        bool reopen_inconsistent,
        bool log)
        : m_planning_interface_(planning_interface),
          m_states_hash_map_(true, planning_interface->GetGridSpaceSize()),
          m_eps_(eps),
          m_reopen_inconsistent_(reopen_inconsistent),
          m_log_(log),
          m_max_num_iterations_(max_num_iterations),
          m_output_(std::make_shared<Output>()) {

        // initialize start node
        auto env_state = std::make_shared<env::EnvironmentState>(m_planning_interface_->GetMetricStart(), m_planning_interface_->GetGridStart());
        constexpr double kG = 0;
        double h = m_planning_interface_->GetHeuristic(env_state);
        auto &start = GetState(env_state);
        start.reset(new State(std::move(env_state), kG, h, m_iterations_));
        m_current_ = start;
        m_start_state_ = start;
    }

    std::shared_ptr<Output>
    AStar::Plan() {
        if (m_planned_) { return m_output_; }
        m_planning_interface_->Reset();       // reset the environment
        m_planning_interface_->PlaceRobot();  // place the robot at the start state

        while ((m_max_num_iterations_ < 0 || long(m_iterations_) < m_max_num_iterations_) && !m_priority_queue_.empty()) {
            // check if done
            auto reached_goal_index = m_planning_interface_->IsMetricGoal(m_current_->env_state);  // ignore reached goals
            if (reached_goal_index >= 0) {
                ERL_DEBUG(
                    "Reach goal[%d/%d] (metric: %s, grid: %s) from metric start %s at iteration %lu.",
                    reached_goal_index,
                    m_planning_interface_->GetNumGoals(),
                    common::EigenToNumPyFmtString(m_current_->env_state->metric.transpose()).c_str(),
                    common::EigenToNumPyFmtString(m_current_->env_state->grid.transpose()).c_str(),
                    common::EigenToNumPyFmtString(m_planning_interface_->GetMetricStart().transpose()).c_str(),
                    m_iterations_);

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
                child->parent_action_id = (int) successor.action_id;
                child->g_value = tentative_g_value;
                double f_value = tentative_g_value + m_eps_ * child->h_value;

                if (child->IsOpened()) {  // already in open set
                    (*child->heap_key)->f_value = f_value;
                    m_priority_queue_.increase(child->heap_key);
                } else if (child->IsClosed()) {  // already in closed set
                    // unexpected in A* with consistent heuristic because closed node has optimal g value
                    // this happens if m_planning_interface_ does not provide consistent heuristic function
                    if (m_reopen_inconsistent_) {
                        child->heap_key = m_priority_queue_.push(std::make_shared<PriorityQueueItem>(f_value, child));
                        child->iteration_closed = 0;
                    } else {
                        ERL_WARN_ONCE("Inconsistent heuristic function is detected! (m_reopen_inconsistent_ is false).\n");
                        if (m_log_) { m_output_->inconsistent_list[m_iterations_].push_back(child->env_state->metric); }  // record this abnormal env_state
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
        m_output_->goal_index = goal_index;
        std::shared_ptr<State> goal_state = GetState(m_planning_interface_->GetGoalState(goal_index));
        m_output_->cost = goal_state->g_value + m_planning_interface_->GetTerminalCost(goal_index);
        m_output_->action_ids.clear();
        auto node = goal_state;
        while (node->parent != nullptr) {
            m_output_->action_ids.push_front(node->parent_action_id);
            node = node->parent;
        }
        std::vector<std::vector<std::shared_ptr<env::EnvironmentState>>> path_segments;
        long num_path_states = 0;
        auto state = GetState(m_start_state_->env_state)->env_state;
        for (auto &action_id: m_output_->action_ids) {
            auto path_segment = m_planning_interface_->GetPath(state, action_id);
            if (path_segment.empty()) { continue; }
            path_segments.push_back(path_segment);
            num_path_states += long(path_segment.size());
            state = path_segment.back();
        }
        m_output_->path.resize(state->metric.size(), num_path_states + 2);
        m_output_->path.col(0) = m_planning_interface_->GetMetricStart();
        long index = 1;
        for (auto &path_segment: path_segments) {
            auto num_states = long(path_segment.size());
            for (long i = 0; i < num_states; ++i) { m_output_->path.col(index++) = path_segment[i]->metric; }
        }
        m_output_->path.col(index) = m_planning_interface_->GetMetricGoal(goal_index);

#ifndef NDEBUG
        m_planning_interface_->GetEnvironment()->ShowPaths({{goal_index, m_output_->path}});  // DEBUG
#endif
    }

    void
    AStar::LogStates() const {
        if (!m_log_) { return; }

        if (m_states_hash_map_.UseVector()) {
            auto begin = m_states_hash_map_.VectorBegin();
            auto end = m_states_hash_map_.VectorEnd();
            for (auto it = begin; it != end; ++it) {
                auto &kAstarState = *it;
                if (kAstarState == nullptr) { continue; }
                if (kAstarState->IsOpened()) {
                    m_output_->opened_list[kAstarState->iteration_opened].push_back(kAstarState->env_state->metric);
                } else if (kAstarState->IsClosed()) {
                    m_output_->closed_list[kAstarState->iteration_closed] = kAstarState->env_state->metric;
                }
            }
        } else {
            auto begin = m_states_hash_map_.MapBegin();
            auto end = m_states_hash_map_.MapEnd();
            for (auto it = begin; it != end; ++it) {
                const auto &kAstarState = it->second;
                if (kAstarState->IsOpened()) {
                    m_output_->opened_list[kAstarState->iteration_opened].push_back(kAstarState->env_state->metric);
                } else if (kAstarState->IsClosed()) {
                    m_output_->closed_list[kAstarState->iteration_closed] = kAstarState->env_state->metric;
                }
            }
        }

        // for (auto &[state_hashing, astar_state]: m_states_hash_map_) {
        //     if (astar_state->IsOpened()) {
        //         m_output_->opened_list[astar_state->iteration_opened].push_back(astar_state->env_state->metric);
        //     } else if (astar_state->IsClosed()) {
        //         m_output_->closed_list[astar_state->iteration_closed] = astar_state->env_state->metric;
        //     }
        // }
    }

}  // namespace erl::search_planning::astar
