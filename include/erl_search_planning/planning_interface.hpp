#pragma once

#include "erl_common/eigen.hpp"
#include "erl_env/environment_base.hpp"
#include "heuristic.hpp"

#include <memory>
#include <utility>
#include <typeinfo>

namespace erl::search_planning {

    /**
     * PlanningInterface provides the minimum functionality for a typical planning algorithm that needs obtaining successors, computing heuristics, checking
     * a goal state and hashing a state as an index.
     */
    class PlanningInterface {                                    // virtual inheritance to avoid diamond problem
    protected:
        std::shared_ptr<env::EnvironmentBase> m_env_ = nullptr;  // used to store the environment in GetSuccessors and ComputeHeuristic
        std::shared_ptr<HeuristicBase> m_heuristic_ = nullptr;   // allow custom heuristic function to be used as callback in ComputeHeuristic
        Eigen::VectorXd m_init_start_;                           // used to store the initial start position
        Eigen::VectorXi m_grid_start_;                           // used to store the start position in grid space
        Eigen::VectorXd m_metric_start_;                         // used to store the start position in metric space
        std::vector<Eigen::VectorXd> m_goals_;                   // used to store the goals in ComputeHeuristic and IsGoal
        std::vector<Eigen::VectorXd> m_goals_tolerances_;        // used to store the goals tolerance in ComputeHeuristic and IsGoal
        Eigen::VectorXd m_terminal_costs_;                       // used to store the terminal costs in ComputeHeuristic
        Eigen::VectorXb m_goals_reached_;                        // used to store the goals reached in IsGoal
        double m_time_resolution_ = 0.01;                        // used to compute the path in GetPath, default to 10 ms
        bool m_terminal_cost_set_ = false;

    public:
        PlanningInterface(
            std::shared_ptr<env::EnvironmentBase> env,
            Eigen::VectorXd metric_start_coords,
            std::vector<Eigen::VectorXd> metric_goals_coords,
            std::vector<Eigen::VectorXd> metric_goals_tolerances)
            : m_env_(std::move(env)),
              m_init_start_(std::move(metric_start_coords)),
              m_goals_(std::move(metric_goals_coords)),
              m_goals_tolerances_(std::move(metric_goals_tolerances)),
              m_terminal_costs_(Eigen::VectorXd::Zero(GetNumGoals())),
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

        PlanningInterface(
            std::shared_ptr<env::EnvironmentBase> env,
            Eigen::VectorXd metric_start_coords,
            Eigen::VectorXd metric_goal_coords,
            Eigen::VectorXd metric_goal_tolerance)
            : PlanningInterface(
                  std::move(env),
                  std::move(metric_start_coords),
                  std::vector({std::move(metric_goal_coords)}),
                  std::vector({std::move(metric_goal_tolerance)})) {}

        PlanningInterface(
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
                ERL_INFO("use %s as default heuristic for multiple goals", type_name<HeuristicType>().c_str());
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

        [[nodiscard]] inline std::shared_ptr<env::EnvironmentBase>
        GetEnvironment() const {
            return m_env_;
        }

        inline void
        SetHeuristic(std::shared_ptr<HeuristicBase> heuristic) {
            m_heuristic_ = std::move(heuristic);
        }

        [[nodiscard]] inline double
        GetHeuristic(const std::shared_ptr<env::EnvironmentState> &state) const {
            return (*m_heuristic_)(*state);
        }

        [[nodiscard]] inline std::vector<env::Successor>
        GetSuccessors(const std::shared_ptr<const env::EnvironmentState> &state) const {
            return m_env_->GetSuccessors(state);
        }

        [[nodiscard]] inline Eigen::VectorXd
        GetInitStart() const {
            return m_init_start_;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GetMetricStart() const {
            return m_metric_start_;
        }

        [[nodiscard]] inline Eigen::VectorXi
        GetGridStart() const {
            return m_grid_start_;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GetGoal(long index) const {
            return m_goals_[index];
        }

        [[nodiscard]] inline Eigen::VectorXd
        GetGoalTolerance(long index) const {
            return m_goals_tolerances_[index];
        }

        [[nodiscard]] inline int
        GetNumGoals() const {
            return int(m_goals_.size());
        }

        [[nodiscard]] inline double
        GetTerminalCost(int goal_index) const {
            return m_terminal_costs_[goal_index];
        }

        [[nodiscard]] std::size_t
        GetGridSpaceSize() const {
            return m_env_->GetStateSpaceSize();
        }

        [[nodiscard]] inline int  // return the index of the goal that is reached, -1 if none is reached
        IsGoal(const std::shared_ptr<env::EnvironmentState> &state, bool ignore_reached = false) {
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

        inline void
        PlaceRobot() {
            m_env_->PlaceRobot(m_metric_start_);
        }

        [[nodiscard]] inline std::size_t
        StateHashing(const Eigen::Ref<const Eigen::VectorXi> &grid_state) const {
            return m_env_->GridStateHashing(grid_state);
        }

        [[nodiscard]] inline std::vector<std::shared_ptr<env::EnvironmentState>>
        GetPath(const std::shared_ptr<const env::EnvironmentState> &state, std::size_t action_index) const {
            return m_env_->ForwardAction(state, action_index);
        }

        inline void
        Reset() {
            m_goals_reached_.setConstant(false);
            if (m_env_ != nullptr) { m_env_->Reset(); }
        }

    private:
        inline void
        SetStart() {
            m_grid_start_ = m_env_->MetricToGrid(m_init_start_);
            m_metric_start_ = m_env_->GridToMetric(m_grid_start_);
        }
    };

}  // namespace erl::search_planning
