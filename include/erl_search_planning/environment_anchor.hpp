#pragma once

#include "erl_env/environment_base.hpp"

namespace erl::search_planning {

    /**
     * @brief EnvironmentAnchor is used as the finest resolution level of a multi-resolution search. Its state space is
     * the union of all the state spaces of the other resolution levels. And its action space is the union of all the
     * action spaces of the other resolution levels. This class is used mainly for hashing the state space and
     * generating all possible neighboring states of a given state with the finest resolution, i.e. all possible
     * actions.
     */
    class EnvironmentAnchor : public env::EnvironmentBase {
    protected:
        std::vector<std::shared_ptr<env::EnvironmentBase>> m_envs_ = {};
        std::size_t m_max_action_space_size_ = 0;

    public:
        // struct Successor : env::Successor {
        //     uint8_t action_resolution_level = 0;
        //
        //     Successor() = default;
        //
        //     Successor(std::shared_ptr<env::EnvironmentState> state, double cost, std::vector<int> action_coords, uint8_t resolution_level)
        //         : env::Successor(std::move(state), cost, std::move(action_coords)),
        //           action_resolution_level(resolution_level) {}
        //
        //     Successor(Eigen::VectorXd env_metric_state, Eigen::VectorXi env_grid_state, double cost, std::vector<int> action_coords, uint8_t
        //     resolution_level)
        //         : env::Successor(std::move(env_metric_state), std::move(env_grid_state), cost, std::move(action_coords)),
        //           action_resolution_level(resolution_level) {}
        //
        //     Successor(env::Successor successor, uint8_t resolution_level)
        //         : env::Successor(std::move(successor)),
        //           action_resolution_level(resolution_level) {}
        // };

        explicit EnvironmentAnchor(std::vector<std::shared_ptr<env::EnvironmentBase>> environments)
            : env::EnvironmentBase(nullptr),  // just use the interface of EnvironmentBase, no need to use the distance cost function
              m_envs_(std::move(environments)) {
            for (auto &env: m_envs_) {
                ERL_ASSERTM(env != nullptr, "env is nullptr");
                if (env->GetActionSpaceSize() > m_max_action_space_size_) { m_max_action_space_size_ = env->GetActionSpaceSize(); }
            }
        }

        [[nodiscard]] inline std::size_t
        GetStateSpaceSize() const override {
            ERL_WARN("Default implementation of GetStateSpaceSize() is used. The returned size is an upper bound.");
            std::size_t state_space_size = 0;
            for (auto &env: m_envs_) { state_space_size += env->GetStateSpaceSize(); }
            return state_space_size;
        }

        [[nodiscard]] inline std::size_t
        GetActionSpaceSize() const override {
            ERL_WARN("Default implementation of GetActionSpaceSize() is used. The returned size is an upper bound.");
            std::size_t action_space_size = 0;
            for (auto &env: m_envs_) { action_space_size += env->GetActionSpaceSize(); }
            return action_space_size;
        }

        /**
         * @brief Get the trajectory starting from the given state and following the given action.
         * @param state
         * @param action_coords (env_action_coords, resolution_level), where resolution_level = 1, 2, ..., n
         * @return
         */
        [[nodiscard]] inline std::vector<std::shared_ptr<env::EnvironmentState>>
        ForwardAction(const std::shared_ptr<const env::EnvironmentState> &state, const std::vector<int> &action_coords) const override {
            return m_envs_[action_coords.back() - 1]->ForwardAction(state, {action_coords.begin(), action_coords.end() - 1});
        }

        [[nodiscard]] std::vector<env::Successor>
        GetSuccessors(const std::shared_ptr<env::EnvironmentState> &state) const override {
            std::vector<env::Successor> successors;
            for (auto it = m_envs_.begin(); it < m_envs_.end(); ++it) {
                std::vector<env::Successor> env_successors = (*it)->GetSuccessors(state);
                if (env_successors.empty()) { continue; }
                auto resolution_level = int(std::distance(m_envs_.begin(), it)) + 1;
                for (auto &successor: env_successors) {
                    successor.action_coords.push_back(resolution_level);
                    successors.push_back(successor);
                }
            }
            return successors;
        }

        [[nodiscard]] std::vector<env::Successor>
        GetSuccessors(const std::shared_ptr<env::EnvironmentState> &state, uint8_t resolution_level) const {
            if (resolution_level == 0) { return GetSuccessors(state); }
            std::vector<env::Successor> successors = m_envs_[resolution_level - 1]->GetSuccessors(state);
            for (auto &successor: successors) { successor.action_coords.push_back(resolution_level); }
            return successors;
        }

        [[nodiscard]] bool
        InStateSpace(const std::shared_ptr<env::EnvironmentState> &state) const override {
            return std::any_of(m_envs_.begin(), m_envs_.end(), [&state](const auto &env) { return env->InStateSpace(state); });
        }

        [[nodiscard]] bool
        IsReachable(const std::vector<std::shared_ptr<env::EnvironmentState>> &trajectory) const override {
            return std::any_of(m_envs_.begin(), m_envs_.end(), [&trajectory](const auto &env) { return env->IsReachable(trajectory); });
        }
    };

}  // namespace erl::search_planning
