#pragma once

#include <boost/heap/d_ary_heap.hpp>
#include <limits>
#include <map>
#include <utility>
#include <memory>

#include "planning_interface.hpp"
#include "erl_common/hash_map.hpp"
#include "erl_common/yaml.hpp"

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

    using HashMap = std::unordered_map<long, std::shared_ptr<State>>;

    // min-heap because we use Greater as a comparer instead
    // clang-format off
    using PriorityQueue = boost::heap::d_ary_heap<
        std::shared_ptr<PriorityQueueItem>,
        boost::heap::mutable_<true>,
        boost::heap::arity<8>,
        boost::heap::compare<Greater<PriorityQueueItem>>>;

    // clang-format on

    struct State {
        std::shared_ptr<env::EnvironmentState> env_state = nullptr;
        std::shared_ptr<State> parent = nullptr;
        std::vector<int> action_coords = {};
        PriorityQueue::handle_type heap_key = {};

        double g_value = std::numeric_limits<double>::max();
        double h_value = std::numeric_limits<double>::max();

        std::size_t iteration_opened = 0;
        std::size_t iteration_closed = 0;

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
        std::list<std::vector<int>> action_coords = {};
        double cost = std::numeric_limits<double>::max();

        // logging
        std::map<std::size_t, std::list<Eigen::VectorXi>> opened_list = {};
        std::map<std::size_t, Eigen::VectorXi> closed_list = {};
        std::map<std::size_t, std::list<Eigen::VectorXi>> inconsistent_list = {};
    };

    class AStar {

    public:
        struct Setting : public common::Yamlable<Setting> {
            double eps = 1.;
            long max_num_iterations = -1;
            bool log = false;
            bool reopen_inconsistent = false;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<State> m_start_state_ = nullptr;
        std::shared_ptr<State> m_current_ = nullptr;
        std::shared_ptr<PlanningInterface> m_planning_interface_ = nullptr;
        PriorityQueue m_priority_queue_;
        HashMap m_states_hash_map_;  // faster than std::unordered_map
        std::size_t m_iterations_ = 0;
        bool m_planned_ = false;
        std::shared_ptr<Output> m_output_ = nullptr;

    public:
        explicit AStar(const std::shared_ptr<PlanningInterface>& planning_interface, std::shared_ptr<Setting> setting = nullptr);

        [[nodiscard]] std::shared_ptr<Output>
        Plan();

    private:
        inline std::shared_ptr<State>&
        GetState(const std::shared_ptr<env::EnvironmentState>& env_state) {
            long hashing = m_planning_interface_->StateHashing(env_state);
            return m_states_hash_map_[hashing];
        }

        void
        Expand();

        void
        RecoverPath(int goal_index);

        void
        LogStates() const;
    };
}  // namespace erl::search_planning::astar

namespace YAML {
    template<>
    struct convert<erl::search_planning::astar::AStar::Setting> {
        static Node
        encode(const erl::search_planning::astar::AStar::Setting& rhs) {
            Node node;
            node["eps"] = rhs.eps;
            node["max_num_iterations"] = rhs.max_num_iterations;
            node["log"] = rhs.log;
            node["reopen_inconsistent"] = rhs.reopen_inconsistent;
            return node;
        }

        static bool
        decode(const Node& node, erl::search_planning::astar::AStar::Setting& rhs) {
            rhs.eps = node["eps"].as<double>();
            rhs.max_num_iterations = node["max_num_iterations"].as<long>();
            rhs.log = node["log"].as<bool>();
            rhs.reopen_inconsistent = node["reopen_inconsistent"].as<bool>();
            return true;
        }
    };

    inline Emitter&
    operator<<(Emitter& out, const erl::search_planning::astar::AStar::Setting& rhs) {
        out << YAML::BeginMap;
        out << YAML::Key << "eps" << YAML::Value << rhs.eps;
        out << YAML::Key << "max_num_iterations" << YAML::Value << rhs.max_num_iterations;
        out << YAML::Key << "log" << YAML::Value << rhs.log;
        out << YAML::Key << "reopen_inconsistent" << YAML::Value << rhs.reopen_inconsistent;
        out << YAML::EndMap;
        return out;
    }
}  // namespace YAML
