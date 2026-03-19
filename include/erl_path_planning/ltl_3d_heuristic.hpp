#pragma once

#include "heuristic.hpp"

#include "erl_common/block_timer.hpp"
#include "erl_geometry/kdtree_eigen_adaptor.hpp"

#include <absl/container/flat_hash_map.h>
#include <boost/heap/d_ary_heap.hpp>

#include <memory>

namespace erl::path_planning {

    template<typename Dtype>
    struct LinearTemporalLogicHeuristic3D : public MultiGoalsHeuristic<Dtype, 4> {

        using Super = MultiGoalsHeuristic<Dtype, 4>;
        using EnvState = typename Super::EnvState;
        using GridMapInfo = common::GridMapInfo3D<Dtype>;
        using KdTree = geometry::KdTreeEigenAdaptor<Dtype, 3>;

        std::shared_ptr<env::FiniteStateAutomaton> fsa;
        std::vector<std::shared_ptr<KdTree>> label_to_kdtree;
        Eigen::MatrixX<Dtype> label_distance;

        /**
         * @brief Construct a new Linear Temporal Logic Heuristic 3D object
         * @param fsa_in
         * @param label_maps_in
         * @param grid_map_info cell size of x, y and z axis
         */
        LinearTemporalLogicHeuristic3D(
            std::shared_ptr<env::FiniteStateAutomaton> fsa_in,
            const absl::flat_hash_map<int, Eigen::MatrixX<uint32_t>> &label_maps_in,
            const std::shared_ptr<GridMapInfo> &grid_map_info)
            : Super({}),
              fsa(std::move(fsa_in)) {

            const ERL_BLOCK_TIMER_MSG("LTL Heuristic 3D Construction");

            ERL_ASSERTM(fsa != nullptr, "fsa is nullptr.");
            ERL_ASSERTM(grid_map_info != nullptr, "grid_map_info is nullptr.");

            long label_map_rows = label_maps_in.at(0).rows();
            long label_map_cols = label_maps_in.at(0).cols();
            int grid_map_rows = grid_map_info->Shape(0);
            int grid_map_cols = grid_map_info->Shape(1);
            auto num_floors = static_cast<int>(label_maps_in.size());
            ERL_ASSERTM(
                label_map_rows == grid_map_rows,
                "label_maps #rows is not equal to grid_map_info #rows: {} vs {}.",
                label_map_rows,
                grid_map_rows);
            ERL_ASSERTM(
                label_map_cols == grid_map_cols,
                "label_maps #cols is not equal to grid_map_info #cols: {} vs {}.",
                label_map_cols,
                grid_map_cols);

            auto fsa_setting = fsa->GetSetting();
            auto n_labels = fsa->GetAlphabetSize();
            auto n_fsa_states = fsa_setting->num_states;

            // compute label_to_grid_states
            using SpaceCoords = Eigen::Vector<Dtype, 3>;
            std::vector<std::vector<SpaceCoords>> label_to_metric_states(n_labels);
            for (int i = 0; i < grid_map_rows; ++i) {
                for (int j = 0; j < grid_map_cols; ++j) {
                    for (int k = 0; k < num_floors; ++k) {
                        Dtype &&x = grid_map_info->GridToMeterAtDim(i, 0);
                        Dtype &&y = grid_map_info->GridToMeterAtDim(j, 1);
                        Dtype &&z = grid_map_info->GridToMeterAtDim(k, 2);
                        label_to_metric_states[label_maps_in.at(k)(i, j)].emplace_back(
                            SpaceCoords{x, y, z});
                    }
                }
            }
            // construct kdtree
            label_to_kdtree.resize(n_labels);
            for (uint32_t label = 0; label < n_labels; ++label) {
                auto &metric_states = label_to_metric_states[label];
                auto num_states = static_cast<long>(metric_states.size());
                if (!num_states) { continue; }
                label_to_kdtree[label] =
                    std::make_shared<KdTree>(metric_states[0].data(), num_states);
            }

            // compute label distances
            struct Node {
                Dtype cost = std::numeric_limits<Dtype>::infinity();
                uint32_t label = 0;
                uint32_t fsa_state = 0;
                std::shared_ptr<Node> parent = nullptr;

                Node(
                    const Dtype cost_in,
                    const uint32_t label_in,
                    const uint32_t fsa_state_in,
                    std::shared_ptr<Node> parent_in)
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
            using PriorityQueue = boost::heap::d_ary_heap<
                QueueItem,
                Mutable,
                boost::heap::arity<8>,
                boost::heap::compare<CompareNode>>;
            using HeapKey = typename PriorityQueue::handle_type;

            // cost from one label to another
            Eigen::MatrixX<Dtype> cost_l2l = Eigen::MatrixX<Dtype>::Constant(  // transition cost
                n_labels,
                n_labels,
                std::numeric_limits<Dtype>::infinity());
            label_distance.setConstant(  // g values
                n_labels,
                n_fsa_states,
                std::numeric_limits<Dtype>::infinity());
            // closed set
            Eigen::MatrixXb closed = Eigen::MatrixXb::Constant(n_labels, n_fsa_states, false);
            // open set
            Eigen::MatrixXb opened = Eigen::MatrixXb::Constant(n_labels, n_fsa_states, false);
            Eigen::MatrixX<HeapKey> heap_keys(n_labels, n_fsa_states);
            PriorityQueue queue;
            /// initialize the g-values of all accepting states to 0 and add them to OPEN
            for (auto accepting_state: fsa_setting->accepting_states) {
                for (uint32_t label = 0; label < n_labels; ++label) {
                    label_distance(label, accepting_state) = 0;
                    heap_keys(label, accepting_state) =
                        queue.push(std::make_shared<Node>(0, label, accepting_state, nullptr));
                    opened(label, accepting_state) = true;
                }
            }
            /// Dijkstra's algorithm
            auto compute_label_distance = [&](uint32_t label1, uint32_t label2) -> Dtype {
                if (label1 == label2) { return 0.0f; }
                auto label1_kdtree = label_to_kdtree[label1];
                if (label1_kdtree == nullptr) { return std::numeric_limits<Dtype>::infinity(); }
                auto label2_kdtree = label_to_kdtree[label2];
                if (label2_kdtree == nullptr) { return std::numeric_limits<Dtype>::infinity(); }

                // label1 and label2 both exist.
                // label 0 means all atomic propositions are evaluated false. Distance to label 0
                // is always 0.
                if (label1 == 0 || label2 == 0) { return 0.0f; }

                if (label1_kdtree->kdtree_get_point_count() >
                    label2_kdtree->kdtree_get_point_count()) {
                    std::swap(label1, label2);
                    std::swap(label1_kdtree, label2_kdtree);
                }
                Dtype min_d = std::numeric_limits<Dtype>::infinity();
                const auto num_label1_states =
                    static_cast<long>(label1_kdtree->kdtree_get_point_count());
                for (long i = 0; i < num_label1_states; ++i) {
                    SpaceCoords &&state1 = label1_kdtree->GetPoint(i);
                    long index = -1;
                    Dtype min_d2 = std::numeric_limits<Dtype>::infinity();
                    label2_kdtree->Nearest(state1, index, min_d2);
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
                /// find predecessors: ? -> node->fsa_state via node->label
                for (uint32_t pred_state = 0; pred_state < n_fsa_states; ++pred_state) {
                    if (pred_state == node->fsa_state) { continue; }  // skip self-loop
                    if (fsa->GetNextState(pred_state, node->label) != node->fsa_state) { continue; }
                    for (uint32_t pred_label = 0; pred_label < n_labels; ++pred_label) {
                        if (closed(pred_label, pred_state)) { continue; }  // skip closed
                        // compute transition cost if needed
                        if (std::isinf(cost_l2l(pred_label, node->label))) {
                            cost_l2l(pred_label, node->label) =
                                compute_label_distance(pred_label, node->label);
                            cost_l2l(node->label, pred_label) = cost_l2l(pred_label, node->label);
                        }
                        // update g value
                        Dtype tentative_g = cost_l2l(pred_label, node->label) +
                                            label_distance(node->label, node->fsa_state);
                        if (tentative_g >= label_distance(pred_label, pred_state)) { continue; }
                        label_distance(pred_label, pred_state) = tentative_g;
                        if (opened(pred_label, pred_state)) {
                            // node->cost = tentative_g;
                            (*heap_keys(pred_label, pred_state))->cost = tentative_g;
                            (*heap_keys(pred_label, pred_state))->parent = node;
                            queue.increase(heap_keys(pred_label, pred_state));
                        } else {
                            heap_keys(pred_label, pred_state) = queue.push(
                                std::make_shared<Node>(
                                    tentative_g,
                                    pred_label,
                                    pred_state,
                                    /*parent*/ node));
                            opened(pred_label, pred_state) = true;
                        }
                    }
                }
            }
        }

        /**
         * @brief Compute the heuristic value of the given state
         * @param env_state metric state (x, y, z, q) and grid state (i, j, k, q)
         * @return
         */
        [[nodiscard]] Dtype
        operator()(const EnvState &env_state) const override {
            if (env_state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0f; }

            if (label_distance.size() == 0) { return 0.0f; }

            Dtype h = std::numeric_limits<Dtype>::infinity();
            const auto q = static_cast<uint32_t>(env_state.grid[3]);  // (x, y, z, q)
            if (fsa->IsSinkState(q)) { return h; }          // sink state, never reach the goal
            if (fsa->IsAcceptingState(q)) { return 0.0f; }  // accepting state, goal

            // for each successor of q
            const auto num_states = fsa->GetSetting()->num_states;
            for (uint32_t nq = 0; nq < num_states; ++nq) {
                if (q == nq) { continue; }
                std::vector<uint32_t> labels =
                    fsa->GetTransitionLabels(q, nq);  // labels that can go from q to nq
                for (const uint32_t &label: labels) {
                    auto label_kdtree = label_to_kdtree[label];
                    if (label_kdtree == nullptr) { continue; }
                    long index = -1;
                    Dtype c = std::numeric_limits<Dtype>::infinity();
                    label_kdtree->Nearest(env_state.metric.template head<3>(), index, c);
                    c = std::sqrt(c);
                    if (const Dtype tentative_h = c + label_distance(label, nq); tentative_h < h) {
                        h = tentative_h;
                    }
                }
            }
            return h;
        }
    };

    extern template class LinearTemporalLogicHeuristic3D<float>;
    extern template class LinearTemporalLogicHeuristic3D<double>;

}  // namespace erl::path_planning
