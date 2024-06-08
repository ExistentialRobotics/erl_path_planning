#include "erl_search_planning/planning_interface.hpp"

#include "erl_common/string_utils.hpp"

namespace erl::search_planning {

    PlanningInterface::PlanningInterface(
        std::shared_ptr<env::EnvironmentBase> env,
        Eigen::VectorXd metric_start_coords,
        const std::vector<Eigen::VectorXd> &metric_goals_coords,
        std::vector<Eigen::VectorXd> metric_goals_tolerances,
        std::vector<double> terminal_costs,
        std::shared_ptr<HeuristicBase> heuristic)
        : m_env_(std::move(env)),
          m_heuristic_(std::move(heuristic)),
          m_init_start_(std::move(metric_start_coords)),
          m_goals_tolerances_(std::move(metric_goals_tolerances)),
          m_terminal_costs_(std::move(terminal_costs)) {

        ERL_ASSERTM(m_env_ != nullptr, "env is nullptr");
        const std::size_t num_goals = metric_goals_coords.size();
        ERL_ASSERTM(num_goals > 0, "no goal is provided");
        if (metric_goals_coords.size() > 1) {
            if (m_goals_tolerances_.size() == 1) {
                ERL_INFO("only one goal tolerance is provided, copying it to all goals.");
                m_goals_tolerances_.resize(num_goals, m_goals_tolerances_[0]);
            } else if (m_goals_tolerances_.size() != num_goals) {
                ERL_FATAL("number of goal tolerances must be 1 or equal to the number of goals.");
            }
            if (m_terminal_costs_.size() == 1) {
                ERL_INFO("only one terminal cost is provided, copying it to all goals.");
                m_terminal_costs_.resize(num_goals, m_terminal_costs_[0]);
            } else if (m_terminal_costs_.size() != num_goals) {
                ERL_FATAL("number of terminal costs must be 1 or equal to the number of goals.");
            }
        }

        // to handle nonzero terminal cost for single goal or multiple goals
        m_virtual_goal_successors_.resize(num_goals);
        for (std::size_t i = 0; i < num_goals; ++i) {
            auto &successor = m_virtual_goal_successors_[i];
            successor.env_state = std::make_shared<env::EnvironmentState>();
            successor.env_state->grid.resize(2);
            successor.env_state->grid[0] = env::VirtualStateValue::kGoal;
            successor.cost = m_terminal_costs_[i];
            successor.action_coords = {(-static_cast<int>(i) - 1)};
        }

        if (m_heuristic_ == nullptr) {
            ERL_INFO("use {} as default heuristic for single goal.", type_name<EuclideanDistanceHeuristic<Eigen::Dynamic>>().c_str());
            using DefaultHeuristic = EuclideanDistanceHeuristic<Eigen::Dynamic>;
            if (num_goals == 1) {
                m_heuristic_ = std::make_shared<DefaultHeuristic>(metric_goals_coords[0], m_goals_tolerances_[0], m_terminal_costs_[0]);
            } else {
                std::vector<std::shared_ptr<HeuristicBase>> heuristics;
                heuristics.reserve(num_goals);
                for (std::size_t i = 0; i < num_goals; ++i) {
                    heuristics.emplace_back(std::make_shared<DefaultHeuristic>(metric_goals_coords[i], m_goals_tolerances_[i], m_terminal_costs_[i]));
                }
                m_heuristic_ = std::make_shared<MultiGoalsHeuristic>(std::move(heuristics));
            }
        }

        SetStart();
        SetGoals(metric_goals_coords);
    }

    int
    PlanningInterface::IsMetricGoal(const std::shared_ptr<env::EnvironmentState> &env_state) const {
        if (IsVirtualGoal(env_state)) { return -1; }
        const int num_goals = GetNumGoals();
        const long dim = env_state->metric.size();
        for (int i = 0; i < num_goals; ++i) {
            if (std::isinf(m_terminal_costs_[i])) { continue; }
            bool goal_reached = true;
            for (long j = 0; j < dim; ++j) {
                if (std::abs(env_state->metric[j] - m_goals_[i]->metric[j]) > m_goals_tolerances_[i][j]) {
                    goal_reached = false;
                    break;
                }
            }
            if (goal_reached) { return i; }
        }
        return -1;
    }
}  // namespace erl::search_planning
