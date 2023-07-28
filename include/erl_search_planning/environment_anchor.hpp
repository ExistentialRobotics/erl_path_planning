#pragma once

#include "erl_env/environment_base.hpp"

namespace erl::search_planning {

    /**
     * @brief EnvironmentAnchor is used as the finest resolution level of a multi-resolution search. Its state space is the union of all the state spaces of
     * the other resolution levels. And its action space is the union of all the action spaces of the other resolution levels. This class is used mainly for
     * hashing the state space and generating all possible neighboring states of a given state with the finest resolution, i.e. all possible actions.
     */
    class EnvironmentAnchor : public env::EnvironmentBase {

        std::vector<std::shared_ptr<env::EnvironmentBase>> m_envs_ = {};

    public:
        EnvironmentAnchor(std::vector<std::shared_ptr<env::EnvironmentBase>> environments)
            : env::EnvironmentBase(nullptr),  // just use the interface of EnvironmentBase, no need to use the distance cost function
              m_envs_(std::move(environments)) {}
    };

}  // namespace erl::search_planning
