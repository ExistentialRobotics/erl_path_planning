#include "erl_search_planning/planning_interface.hpp"

namespace erl::search_planning {
    PlanningInterface::PlanningInterface(
        std::shared_ptr<env::EnvironmentBase> env,
        Eigen::VectorXd metric_start_coords,
        const std::vector<Eigen::VectorXd> &metric_goals_coords,
        std::vector<Eigen::VectorXd> metric_goals_tolerances)
        : m_env_(std::move(env)),
          m_init_start_(std::move(metric_start_coords)),
          m_goals_tolerances_(std::move(metric_goals_tolerances)),
          m_terminal_cost_set_(false) {

        ERL_ASSERTM(m_env_ != nullptr, "environment must be provided");

        auto num_goals = int(metric_goals_coords.size());
        ERL_ASSERTM(num_goals > 0, "at least one goal must be provided");

        if (num_goals > 1) {
            if (m_goals_tolerances_.size() == 1) {
                ERL_INFO("only one goal tolerance is provided, copying it to all goals.");
                m_goals_tolerances_.resize(num_goals, m_goals_tolerances_[0]);
            } else {
                ERL_FATAL("number of goal tolerances must be 1 or equal to the number of goals.");
            }
        }

        SetStart();
        SetGoals(metric_goals_coords);

        if (num_goals == 1) {
            ERL_INFO("use EuclideanDistanceHeuristic as default heuristic for single goal");
            m_heuristic_ = std::make_shared<EuclideanDistanceHeuristic>(metric_goals_coords[0], m_goals_tolerances_[0]);
        } else {
            ERL_INFO("use MultiGoalsHeuristic<EuclideanDistanceHeuristic> as default heuristic for multiple goals");
            m_heuristic_ = std::make_shared<MultiGoalsHeuristic<EuclideanDistanceHeuristic>>(metric_goals_coords, m_goals_tolerances_);
        }
        m_terminal_costs_.setZero(num_goals);

    }

    PlanningInterface::PlanningInterface(
        std::shared_ptr<env::EnvironmentBase> env,
        Eigen::VectorXd metric_start_coords,
        const std::vector<Eigen::VectorXd> &metric_goals_coords,
        std::vector<Eigen::VectorXd> metric_goals_tolerance,
        Eigen::VectorXd terminal_costs)
        : m_env_(std::move(env)),
          m_init_start_(std::move(metric_start_coords)),
          m_goals_tolerances_(std::move(metric_goals_tolerance)),
          m_terminal_costs_(std::move(terminal_costs)),
          m_terminal_cost_set_(true) {
        ERL_ASSERTM(m_env_ != nullptr, "environment must be provided.");

        auto num_goals = int(metric_goals_coords.size());
        ERL_ASSERTM(num_goals > 0, "at least one goal must be provided.");

        for (auto &kCost: m_terminal_costs_) { ERL_ASSERTM(!std::isnan(kCost), "one of the terminal costs is nan!"); }
        ERL_ASSERTM(num_goals == m_terminal_costs_.size(), "number of terminal costs must be equal to the number of goals.");

        if (num_goals > 1) {
            if (m_goals_tolerances_.size() == 1) {
                ERL_INFO("only one goal tolerance is provided, copying it to all goals.");
                m_goals_tolerances_.resize(num_goals, m_goals_tolerances_[0]);
            } else {
                ERL_FATAL("number of goal tolerances must be 1 or equal to the number of goals.");
            }
        }

        SetStart();
        SetGoals(metric_goals_coords);

        if (num_goals == 1) {
            ERL_INFO("use %s as default heuristic for single goal.", type_name<EuclideanDistanceHeuristic>().c_str());
            m_heuristic_ = std::make_shared<EuclideanDistanceHeuristic>(metric_goals_coords[0], m_goals_tolerances_[0]);
        } else {
            using HeuristicType = MultiGoalsWithCostHeuristic<EuclideanDistanceHeuristic>;
            ERL_INFO("use %s as default heuristic for multiple goals with terminal costs.", type_name<HeuristicType>().c_str());

            auto distance_cost_func = m_env_->GetDistanceCostFunc();
            Eigen::MatrixXd goal_cost_matrix = Eigen::MatrixXd::Zero(num_goals, num_goals);
            m_virtual_goal_successors_.resize(num_goals);
            for (int i = 0; i < num_goals; ++i) {
                goal_cost_matrix(i, i) = m_terminal_costs_[i];
                for (int j = i + 1; j < num_goals; ++j) {
                    double distance = (*distance_cost_func)(*m_goals_[i], *m_goals_[j]);
                    goal_cost_matrix(i, j) = distance;
                    goal_cost_matrix(j, i) = distance;
                }

                auto &successor = m_virtual_goal_successors_[i];
                successor.env_state->grid.resize(2);
                successor.env_state->grid[0] = env::VirtualStateValue::kGoal;
                successor.cost = m_terminal_costs_[i];
                successor.action_id = -i - 1;
            }
            m_heuristic_ = std::make_shared<HeuristicType>(metric_goals_coords, m_goals_tolerances_, goal_cost_matrix);
        }
    }

    int
    PlanningInterface::IsMetricGoal(const std::shared_ptr<env::EnvironmentState> &state) const {
        int num_goals = GetNumGoals();
        long dim = state->metric.size();
        for (int i = 0; i < num_goals; ++i) {

            if (m_terminal_cost_set_ && (m_terminal_costs_[i] >= std::numeric_limits<double>::max())) { continue; }

            bool goal_reached = true;
            for (long j = 0; j < dim; ++j) {
                double err = std::abs(state->metric[j] - m_goals_[i]->metric[j]);
                if (err > m_goals_tolerances_[i][j]) {
                    goal_reached = false;
                    break;
                }
            }

            if (goal_reached) { return i; }
        }

        return -1;
    }
}  // namespace erl::search_planning
