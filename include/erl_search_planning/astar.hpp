#pragma once

#include <boost/heap/d_ary_heap.hpp>
#include <limits>
#include <map>
#include <utility>

#include "planning_interface.hpp"
#include "erl_common/hash_map.hpp"

namespace erl::search_planning::astar {

    struct State;

    struct PriorityQueueItem {
        double f_value = std::numeric_limits<double>::max();
        std::shared_ptr<State> state = nullptr;

        PriorityQueueItem() = default;

        PriorityQueueItem(double f, std::shared_ptr<State> s)
            : f_value(f),
              state(std::move(s)) {}
    };

    template<typename T>
    struct Greater {
        bool
        operator()(const std::shared_ptr<T>& s1, const std::shared_ptr<T>& s2) const {
            if (std::abs(s1->f_value - s2->f_value) < 1.e-6) {
                // f value is too close, compare g value
                return s1->state->g_value > s2->state->g_value;
            }
            return s1->f_value > s2->f_value;
        }
    };

    using HashMap = std::unordered_map<std::size_t, std::shared_ptr<State>>;

    // min-heap because we use Greater as a comparer instead
    using PriorityQueue = boost::heap::
        d_ary_heap<std::shared_ptr<PriorityQueueItem>, boost::heap::mutable_<true>, boost::heap::arity<8>, boost::heap::compare<Greater<PriorityQueueItem>>>;

    struct State {
        std::shared_ptr<env::EnvironmentState> env_state = nullptr;
        std::shared_ptr<State> parent = nullptr;
        int parent_action_id = -1;
        PriorityQueue::handle_type heap_key{};

        double g_value = std::numeric_limits<double>::max();
        double h_value = std::numeric_limits<double>::max();

        std::size_t iteration_opened = 0;
        std::size_t iteration_closed = 0;

        // TODO: DEBUG
        // std::vector<std::size_t> parent_history;
        // std::vector<double> g_value_history;

        State(std::shared_ptr<env::EnvironmentState> env_state_in, double g, double h, std::size_t iter_opened)
            : env_state(std::move(env_state_in)),
              g_value(g),
              h_value(h),
              iteration_opened(iter_opened) {
            ERL_DEBUG_ASSERT(env_state != nullptr, "env_state is nullptr.\n");
        }

        State(std::shared_ptr<env::EnvironmentState> env_state_in, double h)
            : env_state(std::move(env_state_in)),
              h_value(h) {
            ERL_DEBUG_ASSERT(env_state != nullptr, "env_state is nullptr.\n");
        }

        [[nodiscard]] inline bool
        IsOpened() const {
            return iteration_opened > iteration_closed;
        }

        [[nodiscard]] inline bool
        IsClosed() const {
            return iteration_opened < iteration_closed;
        }
    };

    struct Output {
        int goal_index = -1;
        Eigen::MatrixXd path = {};
        std::list<std::size_t> action_ids{};
        double cost = std::numeric_limits<double>::max();

        // logging
        std::map<std::size_t, std::list<Eigen::VectorXd>> opened_list = {};
        std::map<std::size_t, Eigen::VectorXd> closed_list = {};
        std::map<std::size_t, std::list<Eigen::VectorXd>> inconsistent_list = {};
    };

    class AStar {

        // Eigen::VectorXi m_grid_start_state_;
        std::shared_ptr<State> m_start_state_;
        std::shared_ptr<State> m_current_;
        std::shared_ptr<PlanningInterface> m_planning_interface_;
        PriorityQueue m_priority_queue_;
        common::HashMap<std::size_t, std::shared_ptr<State>> m_states_hash_map_;
        // HashMap m_states_hash_map_;  // slower than common::HashMap
        double m_eps_ = 1.;
        std::size_t m_iterations_ = 0;
        bool m_reopen_inconsistent_ = false;
        bool m_log_ = false;
        bool m_planned_ = false;
        long m_max_num_iterations_ = -1;  // -1 means no limit
        std::shared_ptr<Output> m_output_;

    public:
        explicit AStar(
            const std::shared_ptr<PlanningInterface>& planning_interface,
            double eps = 1.,
            long max_num_iterations = -1,    // -1 means no limit
            bool reopen_inconsistent = false,
            bool log = false);

        [[nodiscard]] std::shared_ptr<Output>
        Plan();

    private:
        inline std::shared_ptr<State>&
        GetState(const std::shared_ptr<env::EnvironmentState>& env_state) {
            return m_states_hash_map_[m_planning_interface_->StateHashing(env_state)];
        }

        void
        Expand();

        void
        RecoverPath(int goal_index);

        void
        LogStates() const;
    };
}  // namespace erl::search_planning::astar
