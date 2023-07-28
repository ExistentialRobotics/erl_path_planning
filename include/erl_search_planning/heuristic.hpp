#pragma once

#include "erl_common/assert.hpp"
#include "erl_common/eigen.hpp"
#include "erl_env/environment_state.hpp"

namespace erl::search_planning {

    class HeuristicBase {
    protected:
        Eigen::VectorXd m_goal_;
        Eigen::VectorXd m_goal_tolerance_;

    public:
        HeuristicBase() = default;

        HeuristicBase(Eigen::VectorXd goal, Eigen::VectorXd goal_tolerance)
            : m_goal_(std::move(goal)),
              m_goal_tolerance_(std::move(goal_tolerance)) {
            ERL_ASSERTM(this->m_goal_.size() > 0, "goal dimension is zero.");
            ERL_ASSERTM(this->m_goal_tolerance_.size() == this->m_goal_.size(), "goal tolerance dimension is not equal to goal dimension.");
        }

        virtual ~HeuristicBase() = default;

        virtual double
        operator()(const env::EnvironmentState &state) const = 0;
    };

    struct EuclideanDistanceHeuristic : public HeuristicBase {

        EuclideanDistanceHeuristic(Eigen::VectorXd goal, Eigen::VectorXd goal_tolerance)
            : HeuristicBase(std::move(goal), std::move(goal_tolerance)) {}

        double
        operator()(const env::EnvironmentState &state) const override {
            ERL_ASSERTM(state.metric.size() == m_goal_.size(), "state dimension is not equal to goal dimension.");
            long n = state.metric.size();
            double distance = 0.0;
            for (long i = 0; i < n; ++i) {
                double diff = std::abs(state.metric[i] - m_goal_[i]);
                diff = std::max(diff - m_goal_tolerance_[i], 0.0);
                distance += diff * diff;
            }
            distance = std::sqrt(distance);
            return distance;
        }
    };

    struct ManhattanDistanceHeuristic : public HeuristicBase {

        ManhattanDistanceHeuristic(Eigen::VectorXd goal, Eigen::VectorXd goal_tolerance)
            : HeuristicBase(std::move(goal), std::move(goal_tolerance)) {}

        double
        operator()(const env::EnvironmentState &state) const override {
            ERL_ASSERTM(state.metric.size() == m_goal_.size(), "state dimension is not equal to goal dimension.");
            long n = state.metric.size();
            double distance = 0.0;
            for (long i = 0; i < n; ++i) {
                double diff = std::abs(state.metric[i] - m_goal_[i]);
                diff = std::max(diff - m_goal_tolerance_[i], 0.0);
                distance += diff;
            }
            return distance;
        }
    };

    template<typename Heuristic>
    struct MultiGoalsHeuristic : public HeuristicBase {
        static_assert(std::is_base_of_v<HeuristicBase, Heuristic>, "Heuristic must be derived from HeuristicBase.");
        std::vector<std::shared_ptr<HeuristicBase>> goal_heuristics;

        explicit MultiGoalsHeuristic(std::vector<std::shared_ptr<HeuristicBase>> goal_heuristics_in)
            : goal_heuristics(std::move(goal_heuristics_in)) {
            ERL_ASSERTM(!goal_heuristics.empty(), "goal_heuristics_in is empty.");
            std::size_t num_goals = goal_heuristics.size();
            for (std::size_t i = 0; i < num_goals; ++i) { ERL_ASSERTM(goal_heuristics[i] != nullptr, "goal_heuristics_in[%d] is nullptr.", int(i)); }
        }

        explicit MultiGoalsHeuristic(const Eigen::Ref<const Eigen::MatrixXd> &goals, const Eigen::Ref<const Eigen::VectorXd> &goal_tolerance) {
            ERL_ASSERTM(goals.size() > 0, "goals is empty.");
            long num_goals = goals.cols();
            goal_heuristics.reserve(num_goals);
            for (long i = 0; i < num_goals; ++i) { goal_heuristics.emplace_back(std::make_shared<Heuristic>(goals.col(i), goal_tolerance)); }
        }

        MultiGoalsHeuristic(const std::vector<Eigen::VectorXd> &goals, const std::vector<Eigen::VectorXd> &goal_tolerances) {
            ERL_ASSERTM(!goals.empty(), "goals is empty.");
            ERL_ASSERTM(goals.size() == goal_tolerances.size(), "goals and goal tolerances have different sizes.");
            std::size_t num_goals = goals.size();
            for (std::size_t i = 0; i < num_goals; ++i) { goal_heuristics.emplace_back(std::make_shared<Heuristic>(goals[i], goal_tolerances[i])); }
        }

        double
        operator()(const env::EnvironmentState &state) const override {
            double min_h = std::numeric_limits<double>::max();
            for (auto &heuristic: goal_heuristics) {
                double h = (*heuristic)(state);
                if (h < min_h) { min_h = h; }
            }
            return min_h;
        }
    };

    template<typename Heuristic>
    struct MultiGoalsWithCostHeuristic : public HeuristicBase {
        static_assert(std::is_base_of_v<HeuristicBase, Heuristic>, "Heuristic must be derived from HeuristicBase.");
        std::vector<std::shared_ptr<Heuristic>> goal_heuristics;
        Eigen::VectorXd min_cost_to_virtual_goal;

        /**
         *
         * @param goals
         * @param goal_tolerances
         * @param goal_cost_matrix the cost of moving from goal i to goal j is goal_cost_matrix(i, j), where i and j are indices of goals.
         * The terminal cost of goal i is goal_cost_matrix(i, i).
         */
        explicit MultiGoalsWithCostHeuristic(
            const std::vector<Eigen::VectorXd> &goals,
            const std::vector<Eigen::VectorXd> &goal_tolerances,
            const Eigen::Ref<const Eigen::MatrixXd> &goal_cost_matrix) {

            ERL_ASSERTM(!goals.empty(), "goals is empty.");
            ERL_ASSERTM(goals.size() == goal_tolerances.size(), "goals and goal tolerances have different sizes.");
            auto num_goals = long(goals.size());
            ERL_ASSERTM(goal_cost_matrix.rows() == num_goals, "goal distance matrix should have %ld rows.", num_goals);
            ERL_ASSERTM(goal_cost_matrix.cols() == num_goals, "goal distance matrix should have %ld columns.", num_goals);
            ERL_ASSERTM((goal_cost_matrix.array() >= 0.0).all(), "goal distance matrix should be non-negative.");
            ERL_ASSERTM(goal_cost_matrix.transpose() == goal_cost_matrix, "goal distance matrix should be symmetric.");

            Eigen::MatrixXd cost_to_virtual_goal = goal_cost_matrix;
            Eigen::VectorXd terminal_cost = goal_cost_matrix.diagonal();
            for (long i = 0; i < num_goals; ++i) {
                cost_to_virtual_goal(i, i) = 0.0;
                for (long j = 0; j < num_goals; ++j) { cost_to_virtual_goal(j, i) += terminal_cost[j]; }  // per column
                goal_heuristics.emplace_back(std::make_shared<Heuristic>(goals[i], goal_tolerances[i]));
            }
            min_cost_to_virtual_goal = cost_to_virtual_goal.colwise().minCoeff();  // continuous memory access by per column
        }

        double
        operator()(const env::EnvironmentState &state) const override {
            if (state.grid[0] == env::VirtualStateValue::kGoal) {
                return 0.0; }  // virtual goal
            double min_h = std::numeric_limits<double>::max();
            auto num_goals = int(goal_heuristics.size());
            for (int i = 0; i < num_goals; ++i) {
                double h = (*goal_heuristics[i])(state) + min_cost_to_virtual_goal[i];
                if (h < min_h) { min_h = h; }
            }
            return min_h;
        }
    };
}  // namespace erl::search_planning
