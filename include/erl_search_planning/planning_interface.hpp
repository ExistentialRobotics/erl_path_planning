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
        bool m_terminal_cost_set_ = false;

    public:
        PlanningInterface(
            std::shared_ptr<env::EnvironmentBase> env,
            Eigen::VectorXd metric_start_coords,
            std::vector<Eigen::VectorXd> metric_goals_coords,
            std::vector<Eigen::VectorXd> metric_goals_tolerances);

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
            Eigen::VectorXd terminal_costs);

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
        GetSuccessors(const std::shared_ptr<env::EnvironmentState> &state) const {
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

        [[nodiscard]] inline std::shared_ptr<env::EnvironmentState>
        GetStartState() const {
            return std::make_shared<env::EnvironmentState>(m_metric_start_, m_grid_start_);
        }

        [[nodiscard]] inline Eigen::VectorXd
        GetMetricGoal(long index) const {
            return m_goals_[index];
        }

        [[nodiscard]] inline Eigen::VectorXi
        GetGridGoal(long index) const {
            return m_env_->MetricToGrid(m_goals_[index]);
        }

        [[nodiscard]] inline std::shared_ptr<env::EnvironmentState>
        GetGoalState(long index) const {
            return std::make_shared<env::EnvironmentState>(m_goals_[index], m_env_->MetricToGrid(m_goals_[index]));
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

        [[nodiscard]] int  // return the index of the goal that is reached, -1 if none is reached
        IsGoal(const std::shared_ptr<env::EnvironmentState> &state, bool ignore_reached = false);

        inline void
        PlaceRobot() {
            m_env_->PlaceRobot(m_metric_start_);
        }

        [[nodiscard]] inline std::size_t
        StateHashing(const std::shared_ptr<env::EnvironmentState> &state) const {
            return m_env_->StateHashing(state);
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
