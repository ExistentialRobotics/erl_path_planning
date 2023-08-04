#pragma once

#include "erl_common/assert.hpp"
#include "erl_common/eigen.hpp"
#include "erl_common/csv.hpp"
#include "erl_env/environment_state.hpp"

namespace erl::search_planning {

    class HeuristicBase {
    protected:
        Eigen::VectorXd m_goal_;
        Eigen::VectorXd m_goal_tolerance_;
        double m_terminal_cost_ = 0.0;

    public:
        HeuristicBase() = default;

        HeuristicBase(Eigen::VectorXd goal, Eigen::VectorXd goal_tolerance, double terminal_cost = 0.0)
            : m_goal_(std::move(goal)),
              m_goal_tolerance_(std::move(goal_tolerance)),
              m_terminal_cost_(terminal_cost) {
            ERL_ASSERTM(this->m_goal_.size() > 0, "goal dimension is zero.");
            ERL_ASSERTM(this->m_goal_tolerance_.size() == this->m_goal_.size(), "goal tolerance dimension is not equal to goal dimension.");
        }

        virtual ~HeuristicBase() = default;

        virtual double
        operator()(const env::EnvironmentState &state) const = 0;
    };

    struct EuclideanDistanceHeuristic : public HeuristicBase {

        EuclideanDistanceHeuristic(Eigen::VectorXd goal, Eigen::VectorXd goal_tolerance, double terminal_cost = 0.0)
            : HeuristicBase(std::move(goal), std::move(goal_tolerance), terminal_cost) {}

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
            distance = std::sqrt(distance) + m_terminal_cost_;
            return distance;
        }
    };

    struct ManhattanDistanceHeuristic : public HeuristicBase {

        ManhattanDistanceHeuristic(Eigen::VectorXd goal, Eigen::VectorXd goal_tolerance, double terminal_cost = 0.0)
            : HeuristicBase(std::move(goal), std::move(goal_tolerance), terminal_cost) {}

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
            distance += m_terminal_cost_;
            return distance;
        }
    };

    struct DictionaryHeuristic : public HeuristicBase {
        std::unordered_map<long, double> m_heuristic_dictionary_;
        std::function<long(const env::EnvironmentState &)> m_state_hashing_func_;
        bool m_assert_on_missing_ = true;

        DictionaryHeuristic(const std::string &csv_path, std::function<long(const env::EnvironmentState &)> state_hashing_func, bool assert_on_missing = true)
            : m_state_hashing_func_(std::move(state_hashing_func)),
              m_assert_on_missing_(assert_on_missing) {
            ERL_ASSERTM(!csv_path.empty(), "csv_path is empty.");
            ERL_ASSERTM(std::filesystem::exists(csv_path), "%s does not exist.", csv_path.c_str());
            ERL_ASSERTM(m_state_hashing_func_ != nullptr, "state_hashing is nullptr.");
            // Read csv file of two columns. The first column is the state hashing, and the second one is the heuristic value.
            std::vector<std::vector<std::string>> lines = common::LoadCsvFile(csv_path.c_str());
            std::size_t num_lines = lines.size();
            ERL_ASSERTM(num_lines > 0, "No heuristic values are provided.");
            for (std::size_t i = 0; i < num_lines; ++i) {
                auto &line = lines[i];
                ERL_ASSERTM(line.size() == 2, "Each line should be <state hashing>, <heuristic value>.");
                long state_hashing;
                double heuristic_value;
                try {
                    state_hashing = std::stol(line[0]);
                } catch (const std::invalid_argument &e) {
                    ERL_FATAL("%s. Failed to convert %s to a long number.", e.what(), line[0].c_str());
                } catch (const std::out_of_range &e) { ERL_FATAL("%s. The number, %s, is out of the range of signed long.", e.what(), line[0].c_str()); }
                try {
                    heuristic_value = std::stod(line[1]);
                } catch (const std::invalid_argument &e) {
                    ERL_FATAL("%s. Failed to convert %s to a double number.", e.what(), line[1].c_str());
                } catch (const std::out_of_range &e) { ERL_FATAL("%s. The number, %s, is out of the range of double.", e.what(), line[1].c_str()); }
                ERL_ASSERTM(m_heuristic_dictionary_.insert({state_hashing, heuristic_value}).second, "Duplicated state hashing %ld.", state_hashing);
            }
        }

        double
        operator()(const env::EnvironmentState &state) const override {
            long state_hashing = m_state_hashing_func_(state);
            auto it = m_heuristic_dictionary_.find(state_hashing);
            if (it == m_heuristic_dictionary_.end()) {
                if (m_assert_on_missing_) {
                    ERL_FATAL(
                        "The state (metric: %s, grid: %s, hashing: %ld) is not found in the dictionary.",
                        common::EigenToNumPyFmtString(state.metric).c_str(),
                        common::EigenToNumPyFmtString(state.grid).c_str(),
                        state_hashing);
                }
                return std::numeric_limits<double>::max();
            } else {
                return it->second;
            }
        }
    };

    struct MultiGoalsHeuristic : public HeuristicBase {
        std::vector<std::shared_ptr<HeuristicBase>> goal_heuristics;

        explicit MultiGoalsHeuristic(std::vector<std::shared_ptr<HeuristicBase>> goal_heuristics_in)
            : goal_heuristics(std::move(goal_heuristics_in)) {
            ERL_ASSERTM(!goal_heuristics.empty(), "goal_heuristics_in is empty.");
            std::size_t num_goals = goal_heuristics.size();
            for (std::size_t i = 0; i < num_goals; ++i) { ERL_ASSERTM(goal_heuristics[i] != nullptr, "goal_heuristics_in[%d] is nullptr.", int(i)); }
        }

        double
        operator()(const env::EnvironmentState &state) const override {
            if (state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0; }  // virtual goal
            double min_h = std::numeric_limits<double>::max();
            for (auto &heuristic: goal_heuristics) {
                double h = (*heuristic)(state);
                if (h < min_h) { min_h = h; }
            }
            return min_h;
        }
    };
}  // namespace erl::search_planning
