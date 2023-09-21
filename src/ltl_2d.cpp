#include "erl_search_planning/ltl_2d.hpp"

namespace erl::search_planning {

    LinearTemporalLogicHeuristic2D::LinearTemporalLogicHeuristic2D(
        std::shared_ptr<erl::env::FiniteStateAutomaton> fsa_in,
        const Eigen::Ref<const Eigen::MatrixX<uint32_t>> &label_map,
        const std::shared_ptr<erl::common::GridMapInfo2D> &grid_map_info)
        : MultiGoalsHeuristic({}),
          fsa(std::move(fsa_in)) {

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
        std::unordered_map<uint32_t, std::vector<std::array<double, 2>>> label_to_metric_states(num_labels);
        int n_rows = grid_map_info->Rows();
        int n_cols = grid_map_info->Cols();
        for (int i = 0; i < n_rows; ++i) {
            for (int j = 0; j < n_cols; ++j) {
                double &&x = grid_map_info->GridToMeterForValue(i, 0);
                double &&y = grid_map_info->GridToMeterForValue(j, 1);
                label_to_metric_states[label_map(i, j)].emplace_back(std::array<double, 2>{x, y});
            }
        }
        // construct kdtree for each label
        label_to_kdtree.resize(num_labels);
        for (uint32_t label = 0; label < num_labels; ++label) {
            auto &metric_states = label_to_metric_states[label];
            auto num_states = long(metric_states.size());
            if (!num_states) { continue; }
            Eigen::Map<Eigen::Matrix2Xd> data_map(metric_states[0].data(), 2, num_states);
            label_to_kdtree[label] = std::make_shared<KdTree>(data_map);
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
            auto label1_kdtree = label_to_kdtree[label1];
            if (label1_kdtree == nullptr) { return std::numeric_limits<double>::infinity(); }
            auto label2_kdtree = label_to_kdtree[label2];
            if (label2_kdtree == nullptr) { return std::numeric_limits<double>::infinity(); }

            /// label1 and label2 both exist.
            if (label1 == 0 || label2 == 0) { return 0.; }  // label 0 means all atomic propositions are evaluated false. Distance to label 0 is always 0.

            if (label1_kdtree->GetDataMatrix().cols() > label2_kdtree->GetDataMatrix().cols()) {
                std::swap(label1, label2);
                std::swap(label1_kdtree, label2_kdtree);
            }
            double min_d = std::numeric_limits<double>::infinity();
            auto &label1_states = label1_kdtree->GetDataMatrix();
            long num_label1_states = label1_states.cols();
            for (long i = 0; i < num_label1_states; ++i) {
                Eigen::Vector2d &&state1 = label1_states.col(i);
                int index = -1;
                double min_d2 = std::numeric_limits<double>::infinity();
                label2_kdtree->Knn(1, state1, index, min_d2);
                if (min_d2 < min_d) { min_d = min_d2; }
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
    LinearTemporalLogicHeuristic2D::operator()(const env::EnvironmentState &env_state) const {
        if (env_state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0; }  // virtual goal
        if (label_distance.size() == 0) { return 0.; }
        double h = std::numeric_limits<double>::infinity();
        auto q = uint32_t(env_state.grid[2]);
        if (fsa->IsSinkState(q)) { return h; }       // sink state, never reach the goal
        if (fsa->IsAcceptingState(q)) { return 0; }  // accepting state, goal
        // for each successor of q
        auto num_states = fsa->GetSetting()->num_states;
        for (uint32_t nq = 0; nq < num_states; ++nq) {
            if (q == nq) { continue; }
            std::vector<uint32_t> labels = fsa->GetTransitionLabels(q, nq);  // labels that can go from q to nq
            for (uint32_t &label: labels) {
                auto label_kdtree = label_to_kdtree[label];
                if (label_kdtree == nullptr) { continue; }
                int index = -1;
                double c = std::numeric_limits<double>::infinity();
                label_kdtree->Knn(1, env_state.metric.head<2>(), index, c);
                c = std::sqrt(c);
                double tentative_h = c + label_distance(label, nq);
                if (tentative_h < h) { h = tentative_h; }
            }
        }
        return h;
    }

}  // namespace erl::search_planning
