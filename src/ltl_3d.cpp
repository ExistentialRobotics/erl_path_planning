#include "erl_search_planning/ltl_3d.hpp"

namespace erl::search_planning {

    LinearTemporalLogicHeuristic3D::LinearTemporalLogicHeuristic3D(
        std::shared_ptr<erl::env::FiniteStateAutomaton> fsa_in,
        std::unordered_map<int, Eigen::MatrixX<uint64_t>> label_maps_in,
        const std::shared_ptr<erl::common::GridMapInfo3D> &grid_map_info)
        : fsa(std::move(fsa_in)) {

        ERL_ASSERTM(fsa != nullptr, "fsa is nullptr.");
        ERL_ASSERTM(grid_map_info != nullptr, "grid_map_info is nullptr.");
        long label_map_rows = label_maps_in[0].rows();
        long label_map_cols = label_maps_in[0].cols();
        int grid_map_rows = grid_map_info->Shape(0);
        int grid_map_cols = grid_map_info->Shape(1);
        auto num_floors = int(label_maps_in.size());
        ERL_ASSERTM(label_map_rows == grid_map_rows, "label_maps #rows is not equal to grid_map_info #rows: %ld vs %d.", label_map_rows, grid_map_rows);
        ERL_ASSERTM(label_map_cols == grid_map_cols, "label_maps #cols is not equal to grid_map_info #cols: %ld vs %d.", label_map_cols, grid_map_cols);

        auto fsa_setting = fsa->GetSetting();
        auto num_labels = fsa->GetAlphabetSize();
        auto num_fsa_states = fsa_setting->num_states;

        // compute label_to_grid_states
        label_to_metric_states.reserve(num_labels);
        for (int i = 0; i < grid_map_rows; ++i) {
            for (int j = 0; j < grid_map_cols; ++j) {
                for (int k = 0; k < num_floors; ++k) {
                    double &&x = grid_map_info->GridToMeterForValue(i, 0);
                    double &&y = grid_map_info->GridToMeterForValue(j, 1);
                    double &&z = grid_map_info->GridToMeterForValue(k, 2);
                    label_to_metric_states[label_maps_in[k](i, j)].emplace_back(x, y, z);
                }
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
            [[nodiscard]] inline bool
            operator()(const std::shared_ptr<Node> &n1, const std::shared_ptr<Node> &n2) const {
                return n1->cost > n2->cost;
            }
        };

        using QueueItem = std::shared_ptr<Node>;
        using Mutable = boost::heap::mutable_<true>;
        using PriorityQueue = boost::heap::d_ary_heap<QueueItem, Mutable, boost::heap::arity<8>, boost::heap::compare<CompareNode>>;
        using HeapKey = PriorityQueue::handle_type;

        // cost from one label to another
        Eigen::MatrixXd cost_l2l = Eigen::MatrixXd::Constant(num_labels, num_labels, std::numeric_limits<double>::infinity());  // transition cost
        label_distance.setConstant(num_labels, num_fsa_states, std::numeric_limits<double>::infinity());                        // g values
        Eigen::MatrixXb closed = Eigen::MatrixXb::Constant(num_labels, num_fsa_states, false);                                  // closed set
        Eigen::MatrixXb opened = Eigen::MatrixXb::Constant(num_labels, num_fsa_states, false);                                  // open set
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
                    double dz = state1[2] - state2[2];
                    double d = dx * dx + dy * dy + dz * dz;
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
                    if (tentative_g >= label_distance(pred_label, pred_state)) { continue; }
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

    double
    LinearTemporalLogicHeuristic3D::operator()(const env::EnvironmentState &state) const {
        if (state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0; }  // virtual goal
        if (label_distance.size() == 0) { return 0.; }
        double h = std::numeric_limits<double>::infinity();
        auto q = uint32_t(state.grid[3]);            // (x, y, z, q)
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
                for (const Eigen::Vector3d &metric_state: itr->second) {
                    double dx = metric_state[0] - state.metric[0];
                    double dy = metric_state[1] - state.metric[1];
                    double dz = metric_state[2] - state.metric[2];
                    double d = dx * dx + dy * dy + dz * dz;
                    if (d < c) { c = d; }
                }
                c = std::sqrt(c);

                double tentative_h = c + label_distance(label, nq);
                if (tentative_h < h) { h = tentative_h; }
            }
        }
        return h;
    }

}  // namespace erl::search_planning
