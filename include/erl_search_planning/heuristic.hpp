#pragma once

#include "erl_common/angle_utils.hpp"
#include "erl_common/csv.hpp"
#include "erl_common/eigen.hpp"
#include "erl_common/logging.hpp"
#include "erl_env/environment_state.hpp"
#include "erl_env/finite_state_automaton.hpp"

namespace erl::search_planning {

    template<typename Dtype, int Dim>
    class HeuristicBase {
        static_assert(Dim > 0 || Dim == Eigen::Dynamic, "Dim must be positive or Eigen::Dynamic.");

    public:
        using EnvState = env::EnvironmentState<Dtype, Dim>;
        using MetricState = typename EnvState::MetricState;
        using GridState = typename EnvState::GridState;

    protected:
        MetricState m_goal_;
        MetricState m_goal_tolerance_;
        Dtype m_terminal_cost_ = 0.0f;

    public:
        HeuristicBase() = default;

        HeuristicBase(MetricState goal, MetricState goal_tolerance, const Dtype terminal_cost)
            : m_goal_(std::move(goal)),
              m_goal_tolerance_(std::move(goal_tolerance)),
              m_terminal_cost_(terminal_cost) {}

        virtual ~HeuristicBase() = default;

        virtual Dtype
        operator()(const EnvState &state) const = 0;
    };

    template<typename Dtype, int Dim>
    struct EuclideanDistanceHeuristic : public HeuristicBase<Dtype, Dim> {

        using Super = HeuristicBase<Dtype, Dim>;
        using EnvState = typename Super::EnvState;
        using MetricState = typename Super::MetricState;

        EuclideanDistanceHeuristic(
            MetricState goal,
            MetricState goal_tolerance,
            const Dtype terminal_cost)
            : Super(std::move(goal), std::move(goal_tolerance), terminal_cost) {}

        Dtype
        operator()(const EnvState &state) const override {
            if (state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0f; }  // virtual goal
            ERL_DEBUG_ASSERT(
                state.metric.size() == this->m_goal_.size(),
                "state dimension is not equal to goal dimension.");
            const long n = Dim == Eigen::Dynamic ? state.metric.size() : static_cast<long>(Dim);
            Dtype distance = 0.0f;
            for (long i = 0; i < n; ++i) {
                Dtype diff = std::abs(state.metric[i] - this->m_goal_[i]);
                diff = std::max(diff - this->m_goal_tolerance_[i], static_cast<Dtype>(0.0));
                distance += diff * diff;
            }
            distance = std::sqrt(distance) + this->m_terminal_cost_;
            return distance;
        }
    };

    template<typename Dtype>
    struct Se2Heuristic : public HeuristicBase<Dtype, 3> {
        using Super = HeuristicBase<Dtype, 3>;
        using EnvState = typename Super::EnvState;
        using MetricState = typename Super::MetricState;

        Dtype w_theta = 0;  // weight for the heading difference

        explicit Se2Heuristic(
            MetricState goal,
            MetricState goal_tolerance,
            const Dtype terminal_cost,
            Dtype w_theta_in = 0.0)
            : Super(std::move(goal), std::move(goal_tolerance), terminal_cost),
              w_theta(w_theta_in) {}

        Dtype
        operator()(const EnvState &state) const override {
            if (state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0f; }  // virtual goal
            ERL_DEBUG_ASSERT(
                state.metric.size() == this->m_goal_.size(),
                "state dimension is not equal to goal dimension.");
            Dtype diff_x = std::abs(state.metric[0] - this->m_goal_[0]);
            Dtype diff_y = std::abs(state.metric[1] - this->m_goal_[1]);
            Dtype diff_theta = std::abs(state.metric[2] - this->m_goal_[2]);
            diff_theta = common::WrapAngleTwoPi(diff_theta);
            diff_theta = std::min(diff_theta, static_cast<Dtype>(2 * M_PI - diff_theta));
            diff_x = std::max(diff_x - this->m_goal_tolerance_[0], static_cast<Dtype>(0.0));
            diff_y = std::max(diff_y - this->m_goal_tolerance_[1], static_cast<Dtype>(0.0));
            diff_theta = std::max(diff_theta - this->m_goal_tolerance_[2], static_cast<Dtype>(0.0));
            Dtype distance =
                std::sqrt(diff_x * diff_x + diff_y * diff_y + w_theta * diff_theta * diff_theta);
            distance += this->m_terminal_cost_;
            return distance;
        }
    };

    template<typename Dtype, int Dim>
    struct ManhattanDistanceHeuristic : public HeuristicBase<Dtype, Dim> {

        using Super = HeuristicBase<Dtype, Dim>;
        using EnvState = typename Super::EnvState;
        using MetricState = typename Super::MetricState;

        ManhattanDistanceHeuristic(
            MetricState goal,
            MetricState goal_tolerance,
            const Dtype terminal_cost)
            : Super(std::move(goal), std::move(goal_tolerance), terminal_cost) {}

        Dtype
        operator()(const EnvState &state) const override {
            if (state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0f; }  // virtual goal
            ERL_DEBUG_ASSERT(
                state.metric.size() == this->m_goal_.size(),
                "state dimension is not equal to goal dimension.");
            const long n = Dim == Eigen::Dynamic ? state.metric.size() : Dim;
            Dtype distance = 0.0f;
            for (long i = 0; i < n; ++i) {
                Dtype diff = std::abs(state.metric[i] - this->m_goal_[i]);
                diff = std::max(diff - this->m_goal_tolerance_[i], static_cast<Dtype>(0.0));
                distance += diff;
            }
            distance += this->m_terminal_cost_;
            return distance;
        }
    };

    template<typename Dtype, int Dim>
    struct DictionaryHeuristic : public HeuristicBase<Dtype, Dim> {

        using Super = HeuristicBase<Dtype, Dim>;
        using EnvState = typename Super::EnvState;
        using MetricState = typename Super::MetricState;

        std::unordered_map<long, Dtype> heuristic_dictionary;
        std::function<long(const EnvState &)> state_hashing_func;
        bool assert_on_missing = true;

        DictionaryHeuristic(
            const std::string &csv_path,
            std::function<long(const EnvState &)> state_hashing_func_in,
            const bool assert_on_missing_in = true)
            : state_hashing_func(std::move(state_hashing_func_in)),
              assert_on_missing(assert_on_missing_in) {
            ERL_ASSERTM(!csv_path.empty(), "csv_path is empty.");
            ERL_ASSERTM(std::filesystem::exists(csv_path), "%s does not exist.", csv_path.c_str());
            ERL_ASSERTM(state_hashing_func != nullptr, "state_hashing is nullptr.");
            // Read csv file of two columns. The first column is the state hashing, and the second
            // one is the heuristic value.
            std::vector<std::vector<std::string>> lines = common::LoadCsvFile(csv_path.c_str());
            std::size_t num_lines = lines.size();
            ERL_ASSERTM(num_lines > 0, "No heuristic values are provided.");
            for (std::size_t i = 0; i < num_lines; ++i) {
                auto &line = lines[i];
                ERL_ASSERTM(
                    line.size() == 2,
                    "Each line should be <state hashing>, <heuristic value>.");
                long state_hashing;
                Dtype heuristic_value;
                try {
                    state_hashing = std::stol(line[0]);
                } catch (const std::invalid_argument &e) {
                    ERL_FATAL(
                        "%s. Failed to convert %s to a long number.",
                        e.what(),
                        line[0].c_str());
                } catch (const std::out_of_range &e) {
                    ERL_FATAL(
                        "%s. The number, %s, is out of the range of signed long.",
                        e.what(),
                        line[0].c_str());
                }
                try {
                    heuristic_value = static_cast<Dtype>(std::stod(line[1]));
                } catch (const std::invalid_argument &e) {
                    ERL_FATAL(
                        "%s. Failed to convert %s to a floating number.",
                        e.what(),
                        line[1].c_str());
                } catch (const std::out_of_range &e) {
                    ERL_FATAL(
                        "%s. The number, %s, is out of the range of double.",
                        e.what(),
                        line[1].c_str());
                }
                ERL_ASSERTM(
                    heuristic_dictionary.insert({state_hashing, heuristic_value}).second,
                    "Duplicated state hashing {}.",
                    state_hashing);
            }
        }

        Dtype
        operator()(const EnvState &state) const override {
            if (state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0f; }  // virtual goal
            long state_hashing = state_hashing_func(state);
            if (const auto it = heuristic_dictionary.find(state_hashing);
                it == heuristic_dictionary.end()) {
                if (assert_on_missing) {
                    ERL_FATAL(
                        "The state (metric: {}, grid: {}, hashing: {}) is not found in the "
                        "dictionary.",
                        common::EigenToNumPyFmtString(state.metric).c_str(),
                        common::EigenToNumPyFmtString(state.grid).c_str(),
                        state_hashing);
                }
                return std::numeric_limits<Dtype>::infinity();
            } else {
                return it->second;
            }
        }
    };

    template<typename Dtype, int Dim>
    struct MultiGoalsHeuristic : public HeuristicBase<Dtype, Dim> {

        using Super = HeuristicBase<Dtype, Dim>;
        using EnvState = typename Super::EnvState;

        std::vector<std::shared_ptr<Super>> goal_heuristics;

        explicit MultiGoalsHeuristic(std::vector<std::shared_ptr<Super>> goal_heuristics_in)
            : goal_heuristics(std::move(goal_heuristics_in)) {  // may be empty
            for (std::size_t i = 0; i < goal_heuristics.size(); ++i) {
                ERL_ASSERTM(
                    goal_heuristics[i] != nullptr,
                    "goal_heuristics_in[%d] is nullptr.",
                    static_cast<int>(i));
            }
        }

        Dtype
        operator()(const EnvState &state) const override {
            if (state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0f; }  // virtual goal
            Dtype min_h = std::numeric_limits<Dtype>::infinity();
            for (auto &heuristic: goal_heuristics) {
                if (const Dtype h = (*heuristic)(state); h < min_h) { min_h = h; }
            }
            return min_h;
        }
    };
}  // namespace erl::search_planning
