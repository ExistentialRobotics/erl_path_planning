#include "erl_search_planning/amra_star.hpp"

namespace erl::search_planning::amra_star {
    void
    AMRAStar::Replan() {}

    void
    AMRAStar::ReinitState(erl::search_planning::amra_star::State &state) {}

    void
    AMRAStar::ImprovePath() {}

    void
    AMRAStar::Expand(const std::shared_ptr<State> &parent, std::size_t heuristic_id) {
        uint8_t resolution_level = m_planning_interface_->GetResolutionAssignment(heuristic_id);
        if (heuristic_id == 0) {  // anchor level, consistent heuristic
            ERL_DEBUG_ASSERT(!parent->InClosed(0), "parent is already in anchor-level closed set.");
            parent->SetClosed(0, m_expand_itr_);
        } else {
            ERL_DEBUG_ASSERT(!parent->InClosed(resolution_level), "parent is already in closed set of resolution level %d.", int(resolution_level));
            ERL_DEBUG_ASSERT(parent->InOpened(heuristic_id, resolution_level), "parent is not in opened set of heuristic %d.", int(heuristic_id));
            parent->SetClosed(resolution_level, m_expand_itr_);  // parent is also removed from other opened sets assigned to the same resolution level
        }

        ERL_DEBUG_ASSERT(!m_planning_interface_->IsGoal(parent->env_state), "should not expand from a goal parent.");
        std::vector<PlanningInterfaceMultiResolutions::Successor> successors = m_planning_interface_->GetSuccessors(parent->env_state, resolution_level);
        std::size_t num_resolution_levels = m_planning_interface_->GetNumResolutionLevels();
        std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
        for (auto &successor: successors) {
            std::shared_ptr<State> &child = GetState(successor.env_state);
            if (!child) {
                std::vector<double> heuristic_values = m_planning_interface_->GetHeuristicValues(successor.env_state);
                child.reset(new State(m_plan_itr_, successor, num_resolution_levels, std::move(heuristic_values), m_expand_itr_));
            }

            double tentative_g_value = parent->g_value + successor.cost;
            if (tentative_g_value < child->g_value) {
                child->g_value = tentative_g_value;
                child->parent = parent;
                child->parent_action_id = successor.action_id;
                if (child->InClosed(0)) {
                    // child is in anchor-level closed set, inconsistency detected, need to re-open it
                    m_inconsistent_states_.insert(child);
                    continue;
                }
                InsertOrUpdate(child, 0);
                for (std::size_t h_id = 1; h_id < num_heuristics; ++h_id) {
                    uint8_t res_level = m_planning_interface_->GetResolutionAssignment(h_id);
                }
            }
        }
    }
}  // namespace erl::search_planning::amra_star
