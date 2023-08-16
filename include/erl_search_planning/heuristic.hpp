#pragma once

#include <boost/heap/d_ary_heap.hpp>
#include "erl_common/assert.hpp"
#include "erl_common/eigen.hpp"
#include "erl_common/csv.hpp"
#include "erl_common/grid_map.hpp"
#include "erl_env/environment_state.hpp"
#include "erl_env/finite_state_automaton.hpp"

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

    struct LinearTemporalLogicHeuristic2D : public HeuristicBase {

        std::shared_ptr<erl::env::FiniteStateAutomaton> fsa;
        std::unordered_map<uint32_t, std::vector<Eigen::Vector2d>> label_to_metric_states;
        Eigen::MatrixXd label_distance;

        LinearTemporalLogicHeuristic2D(
            std::shared_ptr<erl::env::FiniteStateAutomaton> fsa_in,
            const Eigen::Ref<const Eigen::MatrixXi> &label_map,
            const std::shared_ptr<erl::common::GridMapInfo2D> &grid_map_info)
            : fsa(std::move(fsa_in)) {

            ERL_ASSERTM(fsa != nullptr, "fsa is nullptr.");
            ERL_ASSERTM(grid_map_info != nullptr, "grid_map_info is nullptr.");
            ERL_ASSERTM(
                label_map.rows() == grid_map_info->Rows(),
                "label_map #rows is not equal to grid_map_info #rows: %ld vs %d.",
                label_map.rows(),
                grid_map_info->Rows());
            ERL_ASSERTM(
                label_map.cols() == grid_map_info->Cols(),
                "label_map #cols is not equal to grid_map_info #cols: %ld vs %d.",
                label_map.cols(),
                grid_map_info->Cols());

            auto fsa_setting = fsa->GetSetting();
            auto num_labels = fsa->GetAlphabetSize();
            auto num_fsa_states = fsa_setting->num_states;

            // compute label_to_grid_states
            label_to_metric_states.reserve(num_labels);
            int n_rows = grid_map_info->Rows();
            int n_cols = grid_map_info->Cols();
            for (int i = 0; i < n_rows; ++i) {
                for (int j = 0; j < n_cols; ++j) {
                    double &&x = grid_map_info->GridToMeterForValue(i, 0);
                    double &&y = grid_map_info->GridToMeterForValue(j, 1);
                    label_to_metric_states[label_map(i, j)].emplace_back(x, y);
                }
            }

            // compute label distances
            struct Node {
                double cost = std::numeric_limits<double>::infinity();
                uint32_t label = 0;
                uint32_t fsa_state = 0;
                std::shared_ptr<Node> parent = nullptr;

                Node(double cost_in, uint32_t label_in, uint32_t fsa_state_in, std::shared_ptr<Node> parent_in)
                    : cost(cost_in),
                      label(label_in),
                      fsa_state(fsa_state_in),
                      parent(std::move(parent_in)) {}
            };

            struct CompareNode {
                [[nodiscard]] bool
                operator()(const std::shared_ptr<Node> &n1, const std::shared_ptr<Node> &n2) const {
                    return n1->cost > n2->cost;
                }
            };

            using QueueItem = std::shared_ptr<Node>;
            using Mutable = boost::heap::mutable_<true>;
            using PriorityQueue = boost::heap::d_ary_heap<QueueItem, Mutable, boost::heap::arity<8>, boost::heap::compare<CompareNode>>;
            using HeapKey = PriorityQueue::handle_type;

            Eigen::MatrixXd cost_l2l =
                Eigen::MatrixXd::Constant(num_labels, num_labels, std::numeric_limits<double>::infinity());   // cost from one label to another
            label_distance.setConstant(num_labels, num_fsa_states, std::numeric_limits<double>::infinity());  // g values
            Eigen::MatrixXb closed = Eigen::MatrixXb::Constant(num_labels, num_fsa_states, false);            // closed set
            Eigen::MatrixXb opened = Eigen::MatrixXb::Constant(num_labels, num_fsa_states, false);            // open set
            Eigen::MatrixX<HeapKey> heap_keys(num_labels, num_fsa_states);
            PriorityQueue queue;
            /// initialize the g-values of all accepting states to 0 and add them to OPEN
            for (auto accepting_state: fsa_setting->accepting_states) {
                for (uint32_t label = 0; label < num_labels; ++label) {
                    label_distance(label, accepting_state) = 0;
                    heap_keys(label, accepting_state) = queue.push(std::make_shared<Node>(0, label, accepting_state, nullptr));
                    opened(label, accepting_state) = true;
                }
            }
            /// Dijkstra's algorithm
            auto compute_label_distance = [&](uint32_t label1, uint32_t label2) -> double {
                if (label1 == label2) { return 0.; }
                auto label1_states_it = label_to_metric_states.find(label1);
                if (label1_states_it == label_to_metric_states.end()) { return std::numeric_limits<double>::infinity(); }
                auto label2_states_it = label_to_metric_states.find(label2);
                if (label2_states_it == label_to_metric_states.end()) { return std::numeric_limits<double>::infinity(); }

                /// label1 and label2 both exist.
                if (label1 == 0 || label2 == 0) { return 0.; }  // label 0 means all atomic propositions are evaluated false. Distance to label 0 is always 0.

                double min_d = std::numeric_limits<double>::infinity();
                for (auto &state1: label1_states_it->second) {
                    for (auto &state2: label2_states_it->second) {
                        double dx = state1[0] - state2[0];
                        double dy = state1[1] - state2[1];
                        double d = dx * dx + dy * dy;
                        if (d < min_d) { min_d = d; }
                    }
                }
                return std::sqrt(min_d);
            };
            while (!queue.empty()) {
                /// get node with min g value, move it from opened to closed
                auto node = queue.top();
                queue.pop();
                opened(node->label, node->fsa_state) = false;
                closed(node->label, node->fsa_state) = true;
                /// find predecessors: ? -> node.fsa_state via what label
                for (uint32_t pred_state = 0; pred_state < num_fsa_states; ++pred_state) {
                    if (pred_state == node->fsa_state) { continue; }  // skip self-loop
                    if (fsa->GetNextState(pred_state, node->label) != node->fsa_state) { continue; }
                    for (uint32_t pred_label = 0; pred_label < num_labels; ++pred_label) {
                        if (closed(pred_label, pred_state)) { continue; }  // skip closed
                        // compute transition cost if needed
                        if (std::isinf(cost_l2l(pred_label, node->label))) {
                            cost_l2l(pred_label, node->label) = compute_label_distance(pred_label, node->label);
                            cost_l2l(node->label, pred_label) = cost_l2l(pred_label, node->label);
                        }
                        // update g value
                        double tentative_g = cost_l2l(pred_label, node->label) + label_distance(node->label, node->fsa_state);
                        if (tentative_g < label_distance(pred_label, pred_state)) {
                            label_distance(pred_label, pred_state) = tentative_g;
                            if (opened(pred_label, pred_state)) {
                                // node->cost = tentative_g;
                                (*heap_keys(pred_label, pred_state))->cost = tentative_g;
                                (*heap_keys(pred_label, pred_state))->parent = node;
                                queue.increase(heap_keys(pred_label, pred_state));
                            } else {
                                heap_keys(pred_label, pred_state) = queue.push(std::make_shared<Node>(tentative_g, pred_label, pred_state, node));
                                opened(pred_label, pred_state) = true;
                            }
                        }
                    }
                }
            }
        }

        double
        operator()(const env::EnvironmentState &state) const override {
            if (state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0; }  // virtual goal
            if (label_distance.size() == 0) { return 0.; }
            double h = std::numeric_limits<double>::infinity();
            auto q = uint32_t(state.grid[2]);
            if (fsa->IsSinkState(q)) { return h; }       // sink state, never reach the goal
            if (fsa->IsAcceptingState(q)) { return 0; }  // accepting state, goal
            // for each successor of q
            auto num_states = fsa->GetSetting()->num_states;
            for (uint32_t nq = 0; nq < num_states; ++nq) {
                if (q == nq) { continue; }
                auto labels = fsa->GetTransitionLabels(q, nq);  // labels that can go from q to nq
                for (uint32_t &label: labels) {
                    double c = std::numeric_limits<double>::infinity();
                    auto itr = label_to_metric_states.find(label);
                    if (itr == label_to_metric_states.end()) { continue; }
                    for (const Eigen::Vector2d &metric_state: itr->second) {
                        double dx = metric_state[0] - state.metric[0];
                        double dy = metric_state[1] - state.metric[1];
                        double d = dx * dx + dy * dy;
                        if (d < c) { c = d; }
                    }
                    c = std::sqrt(c);

                    double tentative_h = c + label_distance(label, nq);
                    if (tentative_h < h) { h = tentative_h; }
                }
            }
            return h;
        }
    };

    struct DictionaryHeuristic : public HeuristicBase {
        std::unordered_map<long, double> heuristic_dictionary;
        std::function<long(const env::EnvironmentState &)> state_hashing_func;
        bool assert_on_missing = true;

        DictionaryHeuristic(
            const std::string &csv_path,
            std::function<long(const env::EnvironmentState &)> state_hashing_func_in,
            bool assert_on_missing_in = true)
            : state_hashing_func(std::move(state_hashing_func_in)),
              assert_on_missing(assert_on_missing_in) {
            ERL_ASSERTM(!csv_path.empty(), "csv_path is empty.");
            ERL_ASSERTM(std::filesystem::exists(csv_path), "%s does not exist.", csv_path.c_str());
            ERL_ASSERTM(state_hashing_func != nullptr, "state_hashing is nullptr.");
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
                ERL_ASSERTM(heuristic_dictionary.insert({state_hashing, heuristic_value}).second, "Duplicated state hashing %ld.", state_hashing);
            }
        }

        double
        operator()(const env::EnvironmentState &state) const override {
            long state_hashing = state_hashing_func(state);
            auto it = heuristic_dictionary.find(state_hashing);
            if (it == heuristic_dictionary.end()) {
                if (assert_on_missing) {
                    ERL_FATAL(
                        "The state (metric: %s, grid: %s, hashing: %ld) is not found in the dictionary.",
                        common::EigenToNumPyFmtString(state.metric).c_str(),
                        common::EigenToNumPyFmtString(state.grid).c_str(),
                        state_hashing);
                }
                return std::numeric_limits<double>::infinity();
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
            double min_h = std::numeric_limits<double>::infinity();
            for (auto &heuristic: goal_heuristics) {
                double h = (*heuristic)(state);
                if (h < min_h) { min_h = h; }
            }
            return min_h;
        }
    };
}  // namespace erl::search_planning
