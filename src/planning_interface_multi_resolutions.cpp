#include "erl_search_planning/planning_interface_multi_resolutions.hpp"

namespace erl::search_planning {
    PlanningInterfaceMultiResolutions::PlanningInterfaceMultiResolutions(
        std::vector<std::shared_ptr<env::EnvironmentBase>> environments,
        std::vector<std::pair<std::shared_ptr<HeuristicBase>, std::size_t>> heuristics,
        Eigen::VectorXd metric_start_coords,
        const std::vector<Eigen::VectorXd> &metric_goals_coords,
        std::vector<Eigen::VectorXd> metric_goals_tolerances)
        : m_envs_(std::move(environments)),
          m_heuristics_(std::move(heuristics)),
          m_init_start_(std::move(metric_start_coords)),
          m_goals_tolerances_(std::move(metric_goals_tolerances)),
          m_terminal_cost_set_(false) {

        std::size_t num_resolution_levels = m_envs_.size();
        ERL_ASSERTM(num_resolution_levels > 0, "at least one environment must be provided.");
        // check if all environments are valid
        m_env_anchor_ = std::dynamic_pointer_cast<EnvironmentAnchor>(m_envs_[0]);
        ERL_ASSERTM(m_env_anchor_ != nullptr, "the anchor-level environment must be derived from EnvironmentAnchor.");
        for (std::size_t i = 1; i < num_resolution_levels; ++i) { ERL_ASSERTM(m_envs_[i] != nullptr, "environment %d is nullptr.", int(i)); }

        SetStart();
        SetGoals(metric_goals_coords);

        // check goals and goals tolerances
        int num_goals = GetNumGoals();
        ERL_ASSERTM(num_goals > 0, "at least one goal must be provided.");
        if (num_goals > 1) {
            if (m_goals_tolerances_.size() == 1) {
                ERL_INFO("only one goal tolerance is provided, copying it to all goals.");
                m_goals_tolerances_.resize(num_goals, m_goals_tolerances_[0]);
            } else {
                ERL_FATAL("number of goal tolerances must be 1 or equal to the number of goals.");
            }
            ERL_WARN(
                "multiple goals are provided, please use %s or %s if necessary.",
                ERL_AS_STRING(MultiGoalsHeuristic),
                ERL_AS_STRING(MultiGoalsWithCostHeuristic));
        }

        // check if each resolution level has at least one heuristic
        std::size_t num_heuristics = m_heuristics_.size();
        m_heuristic_ids_by_resolution_level_.clear();
        m_heuristic_ids_by_resolution_level_.resize(num_resolution_levels);
        for (std::size_t i = 0; i < num_heuristics; ++i) {
            ERL_ASSERTM(m_heuristics_[i].first != nullptr, "heuristic %d is nullptr.", int(i));
            m_heuristic_ids_by_resolution_level_[m_heuristics_[i].second].push_back(i);
        }
        for (std::size_t i = 0; i < num_resolution_levels; ++i) {
            ERL_ASSERTM(!m_heuristic_ids_by_resolution_level_[i].empty(), "resolution level %d has no heuristic.", int(i));
        }
        ERL_ASSERTM(m_heuristics_[0].second == 0, "the first heuristic must be assigned to the anchor level (resolution level 0).");
        ERL_ASSERTM(m_heuristic_ids_by_resolution_level_[0].size() == 1, "the anchor level must have exactly one heuristic.");

        // miscellaneous
        m_terminal_costs_.setZero(num_goals);
    }
}  // namespace erl::search_planning
