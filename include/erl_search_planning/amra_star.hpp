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
                return s1->g_value > s2->g_value;
            }
            return s1->f_value > s2->f_value;
        }
    };

    using PriorityQueue = boost::heap::
        d_ary_heap<std::shared_ptr<PriorityQueueItem>, boost::heap::mutable_<true>, boost::heap::arity<8>, boost::heap::compare<Greater<PriorityQueueItem>>>;

    struct State {
        std::shared_ptr<env::EnvironmentState> env_state;
        uint32_t plan_itr = 0;  // id of the plan that generated/updated this state
        Eigen::VectorX<uint64_t> iteration_opened;
        Eigen::VectorX<uint64_t> iteration_closed;
        uint64_t iteration_closed_max = 0;
        bool closed_in_anchor = false;
        double g_value = std::numeric_limits<double>::max();
        double h_value = std::numeric_limits<double>::max();
        uint8_t resolution_level = 0;
        std::shared_ptr<State> parent = nullptr;
        int parent_action_id = -1;

        State(
            std::shared_ptr<env::EnvironmentState> env_state_in,
            uint32_t plan_itr_in,
            uint8_t resolution_level_in,
            uint8_t num_resolution_levels,
            double g,
            double h,
            uint64_t iter_opened)
            : env_state(std::move(env_state_in)),
              plan_itr(plan_itr_in),
              iteration_opened(Eigen::VectorX<uint64_t>::Zero(num_resolution_levels + 1)),
              iteration_closed(Eigen::VectorX<uint64_t>::Zero(num_resolution_levels + 1)),
              g_value(g),
              h_value(h),
              resolution_level(resolution_level_in) {
            ERL_ASSERTM(
                resolution_level <= num_resolution_levels,
                "resolution level %d is out of range [0, %d].",
                int(resolution_level),
                int(num_resolution_levels));
            iteration_opened[resolution_level] = iter_opened;
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
            // as long as the state is moved into CLOSE_res(i), then iteration_closed[res(i)] > 0.
            // close_set_id = res(i).
            return iteration_closed[closed_set_id] > 0;
        }

        [[nodiscard]] inline bool
        InAnyClosed() const {
            return iteration_closed_max > 0;
        }

        inline void
        SetOpened(uint8_t open_set_id, uint64_t opened_itr) {
            iteration_opened[open_set_id] = opened_itr;
        }

        inline void
        SetClosed(uint8_t close_set_id, uint64_t closed_itr) {
            iteration_closed[close_set_id] = closed_itr;
        }
    };

    struct Output {
        std::map<uint8_t, std::map<uint8_t, Eigen::MatrixXd>> paths;                      // goal_id -> resolution_level -> path
        std::map<uint8_t, std::map<uint8_t, Eigen::VectorXi>> actions;                    // goal_id -> resolution_level -> actions
        std::map<uint8_t, std::map<uint8_t, double>> costs;                               // goal_id -> resolution_level -> cost
        std::map<uint64_t, std::map<uint8_t, std::list<Eigen::VectorXi>>> opened_states;  // plan_itr -> heuristic_id -> list of states
        std::map<uint64_t, std::map<uint8_t, std::list<Eigen::VectorXi>>> closed_states;  // plan_itr -> resolution_level -> list of states
        std::map<uint64_t, std::list<Eigen::VectorXi>> inconsistent_states;               // plan_itr -> list of states
    };

    class AMRAStar {

    public:
        struct Setting : public common::Yamlable<Setting> {
            double time_limit = 0;
            double w1_init = 0;
            double w2_init = 0;
            bool record = false;
            bool paths_of_all_resolutions = false;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

        uint32_t m_plan_itr_ = 0;
        uint64_t m_expand_itr_ = 0;

        double m_w1_ = 0;
        double m_w2_ = 0;
        double m_w1_solve_ = 0;
        double m_w2_solve_ = 0;
        static constexpr double sk_W1Final_ = 1;
        static constexpr double sk_W2Final_ = 1;

        double m_search_time_ = 0;

        struct HeuristicInfo {
            std::shared_ptr<HeuristicBase> heuristic = nullptr;
            PriorityQueue open_queue;
            uint8_t resolution_level = -1;
        };

        std::vector<HeuristicInfo> m_heuristics_;
        std::vector<std::vector<uint8_t>> m_resolution_level_to_heuristic_ids_;

    public:
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
        ReinitState(State& state);

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
        void
        Replan();

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
        void
        ImprovePath();

        void
        Expand(const std::shared_ptr<State>& state, std::size_t heuristic_id);
    };
}  // namespace erl::search_planning::amra_star
