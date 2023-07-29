#pragma once

// Implementation of AMRA*: Anytime Multi-Resolution Multi-HeuristicBase A*

#include <limits>
#include <cstdint>
#include <memory>
#include <boost/heap/d_ary_heap.hpp>
#include "erl_common/yaml.hpp"
#include "erl_common/eigen.hpp"
#include "heuristic.hpp"
#include "planning_interface_multi_resolutions.hpp"

using namespace std::chrono_literals;

namespace erl::search_planning::amra_star {

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

    using PriorityQueue = boost::heap::
        d_ary_heap<std::shared_ptr<PriorityQueueItem>, boost::heap::mutable_<true>, boost::heap::arity<8>, boost::heap::compare<Greater<PriorityQueueItem>>>;

    struct State {
        uint32_t plan_itr = 0;  // iteration of the plan that generated/updated this state
        std::shared_ptr<env::EnvironmentState> env_state;
        std::vector<uint64_t> iteration_opened;
        std::vector<PriorityQueue::handle_type> open_queue_keys;
        std::vector<uint64_t> iteration_closed;
        double g_value = std::numeric_limits<double>::max();
        std::vector<double> h_values;
        const std::vector<uint8_t> in_resolution_levels;  // resolution levels where the state exists
        std::shared_ptr<State> parent = nullptr;          // parent state
        std::vector<int> action_coords = {};              // action coords = (env_action_coords, env_res_level) that generates this state from its parent

        // uint8_t action_resolution_level = -1;             // resolution level which the action belongs to

        State(
            uint32_t plan_itr_in,
            std::shared_ptr<env::EnvironmentState> env_state_in,
            uint8_t num_resolution_levels,
            std::vector<uint8_t> in_resolution_levels,
            std::vector<double> h_vals)
            : plan_itr(plan_itr_in),
              env_state(std::move(env_state_in)),
              iteration_opened(h_vals.size(), 0),
              open_queue_keys(h_vals.size()),
              iteration_closed(num_resolution_levels + 1, 0),
              h_values(std::move(h_vals)),
              in_resolution_levels(std::move(in_resolution_levels)) {}

        [[nodiscard]] bool
        InResolutionLevel(uint8_t resolution_level) const {
            return std::find(in_resolution_levels.begin(), in_resolution_levels.end(), resolution_level) != in_resolution_levels.end();
        }

        [[nodiscard]] inline bool
        InOpened(uint8_t open_set_id, uint8_t close_set_id) const {
            // 1. if state is just moved into OPEN_i, then iteration_opened[i] > 0 and for other heuristics assigned to the same resolution,
            // iteration_opened[j] == 0 and any iteration_closed[j] == 0, i.e. iteration_opened[i] > any iteration_closed[j] of the same resolution.
            // 2. if state is moved into OPEN_i because it has not been in CLOSE_res(i), then iteration_opened[i] > iteration_closed[res(i)]
            return iteration_opened[open_set_id] > iteration_closed[close_set_id];
        }

        [[nodiscard]] inline bool
        InClosed(uint8_t closed_set_id) const {
            // as long as the state is moved into CLOSE_res(i), then iteration_closed[res(i)] > 0, where close_set_id = res(i).
            return iteration_closed[closed_set_id] > 0;
        }

        inline void
        SetOpened(uint8_t open_set_id, uint64_t opened_itr) {
            iteration_opened[open_set_id] = opened_itr;
        }

        inline void
        SetClosed(uint8_t close_set_id, uint64_t closed_itr) {
            iteration_closed[close_set_id] = closed_itr;
        }

        inline void
        RemoveFromClosed(uint8_t close_set_id) {
            iteration_closed[close_set_id] = 0;
        }

        inline void
        RemoveFromAllClosed() {
            std::fill(iteration_closed.begin(), iteration_closed.end(), 0);
        }

        inline void
        SetParent(std::shared_ptr<State> parent_in, std::vector<int> action_coords_in) {  // }, uint8_t action_resolution_level_in) {
            parent = std::move(parent_in);
            action_coords = std::move(action_coords_in);
            // action_resolution_level = action_resolution_level_in;
        }

        inline void
        Reset() {
            iteration_opened.resize(iteration_opened.size(), 0);
            iteration_closed.resize(iteration_closed.size(), 0);
            g_value = std::numeric_limits<double>::max();
            parent = nullptr;
            action_coords.clear();
        }
    };

    struct Output {
        int goal_index = -1;
        Eigen::MatrixXd path = {};
        std::list<std::vector<int>> action_coords = {};
        double cost = std::numeric_limits<double>::max();

        // statistics
        double w1_solve = -1.0;
        double w2_solve = -1.0;
        double search_time = 0.;

        // logging
        std::map<uint32_t, std::map<uint8_t, std::list<Eigen::VectorXi>>> opened_states;  // plan_itr -> heuristic_id -> list of states
        std::map<uint32_t, std::map<uint8_t, std::list<Eigen::VectorXi>>> closed_states;  // plan_itr -> action_resolution_level -> list of states
        std::map<uint32_t, std::list<Eigen::VectorXi>> inconsistent_states;               // plan_itr -> list of states
    };

    class AMRAStar {

    public:
        struct Setting : public common::Yamlable<Setting> {
            std::chrono::nanoseconds time_limit = std::chrono::duration_cast<std::chrono::nanoseconds>(1s);
            double w1_init = 0;
            double w2_init = 0;
            double w1_final = 1;
            double w2_final = 1;
            double w1_decay_factor = 0.5;
            double w2_decay_factor = 0.5;
            bool log = false;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<PlanningInterfaceMultiResolutions> m_planning_interface_ = nullptr;
        uint32_t m_plan_itr_ = 0;
        uint64_t m_total_expand_itr_ = 0;
        Eigen::VectorX<uint64_t> m_expand_itr_;

        double m_w1_ = 0;
        double m_w2_ = 0;

        std::chrono::nanoseconds m_search_time_ = 0ns;

        std::vector<PriorityQueue> m_open_queues_ = {};
        std::unordered_map<std::size_t, std::shared_ptr<State>> m_states_hash_map_ = {};
        // TODO: original AMRA* uses a list, we use a set here, need to check if it is correct
        std::unordered_set<std::shared_ptr<State>> m_inconsistent_states_ = {};
        std::shared_ptr<State> m_start_state_ = nullptr;
        std::vector<std::shared_ptr<State>> m_goal_states_ = {};
        std::shared_ptr<Output> m_output_ = nullptr;

    public:
        /**
         * @brief Re-plan the path:
         * 1. check if any goal is reached
         * 2. reset:
         *      a. weight parameters: w1, w2
         *      b. reset counters of expands
         * 3. increase plan_itr
         * 4. reset:
         *      goal state
         *      start state
         *      queues of open states for each heuristic
         *      list of inconsistent states to be {start_state}
         * 5. while time limit is not reached:
         *      a. move states from inconsistent to open with the heuristic of the anchor level
         *          i. if the state is already in open, update its f-value
         *          ii. if the state is not in open, insert it into open
         *          iii. here we need to check if the state is in open or not. Original AMRA* uses a vector-based min-heap so that we can check if the state has
         *              a valid index. However, we use a d_ary_heap which is list-based. We need to store the open_itr and close_itr for each heuristic, and
         *              check if the state is in open or not by comparing the open_itr and close_itr.
         *          iv. clear the list of inconsistent states
         *      b. for each state in the open set of the anchor level:
         *              for each non-anchor-level heuristic:
         *                  if the state's resolution level is higher than the heuristic's resolution level, i.e. the heuristic's is finer than the state's:
         *                      i. update the state's f-value of the heuristic's resolution level
         *                      ii. set the state's closed flag of the heuristic's resolution level to false
         *                      iii. insert the state into the open list for the heuristic
         *      c. reordering each resolution level's open list
         *      d. clear the closed set of each resolution level
         *      e. improve the solution path, if failed or reach the time limit, break
         *      f. update time statistics
         *      g. extract the improved solution path
         *      h. if expansion happens, update the solution path of each resolution level (optional, for visualization)
         *      i. if w1 == 1 and w2 == 1, done
         *      j. update w1 and w2
         */
        std::shared_ptr<Output>
        Plan();

        /**
         * while open set of the anchor level is not empty and the minimum f-value in this open set is not infinite:
         *     i. if time limit is reached, break
         *     ii. for each non-anchor-level heuristic:
         *             1.) if the open set of the anchor level is empty, break
         *             2.) f_check = w2 * open_anchor->min_f
         *             2.) if the current open set cannot improve the solution, i.e. goal->g <= f_check, done
         *             3.) if the open set of the heuristic is not empty and the minimum f-value in this open set is smaller than f_check:
         *                 a.) if the state of minimum f-value is a goal, done
         *                 b.) expand the state of minimum f-value with the heuristic
         *                 c.) increase the counter of expands for the heuristic
         *             4.) else, expand the state of minimum f-value in the open set of the anchor level with the anchor heuristic
         */
        int
        ImprovePath(const std::chrono::system_clock::time_point& start_time, std::chrono::nanoseconds& elapsed_time);

        void
        Expand(const std::shared_ptr<State>& parent, std::size_t heuristic_id);

    private:
        inline std::shared_ptr<State>&
        GetState(const std::shared_ptr<env::EnvironmentState>& env_state) {
            return m_states_hash_map_[m_planning_interface_->StateHashing(env_state)];
        }

        /**
         * @brief Reinitialize the state:
         * 1. update plan_itr
         * 2. reset g-value
         * 3. update resolution level
         * 4. reset back-pointer to nullptr
         * 5. reset h-value and f-value for each heuristic
         * 6. reset closed flags
         * @param state
         *
         */
        void
        ReinitState(std::shared_ptr<State>& state) {
            if (state->plan_itr == m_plan_itr_) { return; }
            state->Reset();
            state->plan_itr = m_plan_itr_;
            // recompute h-value
            std::size_t num_heuristics = m_planning_interface_->GetNumHeuristics();
            for (std::size_t heuristic_id = 0; heuristic_id < num_heuristics; ++heuristic_id) {
                state->h_values[heuristic_id] = (*m_planning_interface_->GetHeuristic(heuristic_id))(*state->env_state);
            }
        }

        [[nodiscard]] inline double
        GetKeyValue(const std::shared_ptr<State>& state, std::size_t heuristic_id) const {
            return state->g_value + m_w1_ * state->h_values[heuristic_id];
        }

        inline void
        InsertOrUpdate(const std::shared_ptr<State>& state, std::size_t heuristic_id, double f_value) {
            uint8_t resolution_level = m_planning_interface_->GetResolutionAssignment(heuristic_id);
            if (state->InOpened(heuristic_id, resolution_level)) {
                // state is already in open, update its f-value
                (*state->open_queue_keys[heuristic_id])->f_value = f_value;
                m_open_queues_[heuristic_id].increase(state->open_queue_keys[heuristic_id]);
            } else {
                // state is not in open, insert it into open
                state->open_queue_keys[heuristic_id] = m_open_queues_[heuristic_id].push(std::make_shared<PriorityQueueItem>(f_value, state));
                state->SetOpened(heuristic_id, m_total_expand_itr_);
            }
        }

        inline void
        RebuildOpenQueue(std::size_t heuristic_id) {
            auto& open_queue = m_open_queues_[heuristic_id];
            PriorityQueue new_open_queue;
            for (auto& queue_item: open_queue) {
                queue_item->f_value = GetKeyValue(queue_item->state, heuristic_id);
                queue_item->state->open_queue_keys[heuristic_id] = new_open_queue.push(queue_item);
            }
            open_queue.swap(new_open_queue);
        }

        void
        RecoverPath(int goal_index);
    };
}  // namespace erl::search_planning::amra_star
