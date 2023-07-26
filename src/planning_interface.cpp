#include "erl_search_planning/planning_interface.hpp"

namespace erl::search_planning {
    PlanningInterface::PlanningInterface(
        std::shared_ptr<env::EnvironmentBase> env,
        Eigen::VectorXd metric_start_coords,
        std::vector<Eigen::VectorXd> metric_goals_coords,
        std::vector<Eigen::VectorXd> metric_goals_tolerances)
        : m_env_(std::move(env)),
          m_init_start_(std::move(metric_start_coords)),
          m_goals_(std::move(metric_goals_coords)),
          m_goals_tolerances_(std::move(metric_goals_tolerances)),
          m_goals_reached_(Eigen::VectorXb::Constant(GetNumGoals(), false)),
          m_terminal_cost_set_(false) {

        ERL_ASSERTM(m_env_ != nullptr, "environment must be provided");

        int num_goals = GetNumGoals();
        ERL_ASSERTM(num_goals > 0, "at least one goal must be provided");
        if (num_goals > 1) {
            if (m_goals_tolerances_.size() == 1) {
                ERL_INFO("only one goal tolerance is provided, copying it to all goals.");
                m_goals_tolerances_.resize(num_goals, m_goals_tolerances_[0]);
            } else {
                ERL_FATAL("number of goal tolerances must be 1 or equal to the number of goals.");
            }
        }
        if (num_goals == 1) {
            ERL_INFO("use EuclideanDistanceHeuristic as default heuristic for single goal");
            m_heuristic_ = std::make_shared<EuclideanDistanceHeuristic>(m_goals_[0], m_goals_tolerances_[0]);
        } else {
            ERL_INFO("use MultiGoalsHeuristic<EuclideanDistanceHeuristic> as default heuristic for multiple goals");
            m_heuristic_ = std::make_shared<MultiGoalsHeuristic<EuclideanDistanceHeuristic>>(m_goals_, m_goals_tolerances_);
        }
        m_terminal_costs_.setZero(num_goals);

        SetStart();
    }

    PlanningInterface::PlanningInterface(
        std::shared_ptr<env::EnvironmentBase> env,
        Eigen::VectorXd metric_start_coords,
        std::vector<Eigen::VectorXd> metric_goals_coords,
        std::vector<Eigen::VectorXd> metric_goals_tolerance,
        Eigen::VectorXd terminal_costs)
        : m_env_(std::move(env)),
          m_init_start_(std::move(metric_start_coords)),
          m_goals_(std::move(metric_goals_coords)),
          m_goals_tolerances_(std::move(metric_goals_tolerance)),
          m_terminal_costs_(std::move(terminal_costs)),
          m_goals_reached_(Eigen::VectorXb::Constant(GetNumGoals(), false)),
          m_terminal_cost_set_(true) {
        ERL_ASSERTM(m_env_ != nullptr, "environment must be provided.");

        int num_goals = GetNumGoals();
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
        if (num_goals == 1) {
            ERL_INFO("use %s as default heuristic for single goal.", type_name<EuclideanDistanceHeuristic>().c_str());
            m_heuristic_ = std::make_shared<EuclideanDistanceHeuristic>(m_goals_[0], m_goals_tolerances_[0]);
        } else {
            using HeuristicType = MultiGoalsWithCostHeuristic<EuclideanDistanceHeuristic>;
            ERL_INFO("use %s as default heuristic for multiple goals with terminal costs.", type_name<HeuristicType>().c_str());
            Eigen::MatrixXd goal_distance_cost_matrix = Eigen::MatrixXd::Zero(num_goals, num_goals);
            auto distance_cost_func = m_env_->GetDistanceCostFunc();
            env::EnvironmentState state1, state2;
            for (int i = 0; i < num_goals; ++i) {
                state1.metric = m_goals_[i];
                state1.grid = m_env_->MetricToGrid(state1.metric);
                goal_distance_cost_matrix(i, i) = m_terminal_costs_[i];
                for (int j = i + 1; j < num_goals; ++j) {
                    state2.metric = m_goals_[j];
                    state2.grid = m_env_->MetricToGrid(state2.metric);
                    double distance = (*distance_cost_func)(state1, state2);
                    goal_distance_cost_matrix(i, j) = distance;
                    goal_distance_cost_matrix(j, i) = distance;
                }
            }
            m_heuristic_ = std::make_shared<HeuristicType>(m_goals_, m_goals_tolerances_, goal_distance_cost_matrix);
        }

        SetStart();
    }

    int
    PlanningInterface::IsGoal(const std::shared_ptr<env::EnvironmentState> &state, bool ignore_reached) {
        int num_goals = GetNumGoals();
        long dim = state->metric.size();
        for (int i = 0; i < num_goals; ++i) {
            if (ignore_reached && m_goals_reached_[i]) { continue; }

            if (m_terminal_cost_set_ && (m_terminal_costs_[i] >= std::numeric_limits<double>::max())) { continue; }

            bool goal_reached = true;
            for (long j = 0; j < dim; ++j) {
                double err = std::abs(state->metric[j] - m_goals_[i][j]);
                if (err > m_goals_tolerances_[i][j]) {
                    goal_reached = false;
                    break;
                }
            }

            if (goal_reached) {
                m_goals_reached_[i] = true;
                return i;
            }
        }

        return -1;
    }
}  // namespace erl::search_planning
