#include "erl_search_planning/amra_star.hpp"

namespace erl::search_planning::amra_star {
    void
    AMRAStar::Replan() {}

    void
    AMRAStar::ReinitState(erl::search_planning::amra_star::State &state) {}

    void
    AMRAStar::ImprovePath() {}

    void
    AMRAStar::Expand(const std::shared_ptr<State> &state, std::size_t heuristic_id) {
        if (heuristic_id == 0) {  // anchor level, consistent heuristic
            ERL_DEBUG_ASSERT(!state->InClosed(0), "state is already in anchor-level closed set.\n");
            state->SetClosed(0, m_expand_itr_);
        } else {
            uint8_t resolution_level = m_heuristics_[heuristic_id].resolution_level;
            ERL_DEBUG_ASSERT(!state->InClosed(resolution_level), "state is already in closed set of resolution level %d.\n", int(resolution_level));
            ERL_DEBUG_ASSERT(state->InOpened(heuristic_id), "state is not in opened set of heuristic %d.\n", int(heuristic_id));
            state->SetClosed(resolution_level, m_expand_itr_);  // state is also removed from other opened sets assigned to the same resolution level
        }

//        ERL_DEBUG_ASSERT(!m_env_->IsGoal(state), "should not expand from a goal state.\n");


    }
}  // namespace erl::search_planning::amra_star
