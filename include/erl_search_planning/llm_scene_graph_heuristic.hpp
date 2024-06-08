#pragma once

#include "heuristic.hpp"

#include "erl_common/yaml.hpp"
#include "erl_env/atomic_proposition.hpp"
#include "erl_env/environment_ltl_scene_graph.hpp"

#include <memory>

namespace erl::search_planning {

    class LlmSceneGraphHeuristic : public HeuristicBase {

    public:
        struct Setting : public common::Yamlable<Setting> {
            struct LlmWaypoint {
                env::AtomicProposition::Type type;
                int uuid1;
                int uuid2;
            };

            std::map<int, std::map<uint32_t, std::vector<LlmWaypoint>>> llm_paths;
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<env::EnvironmentLTLSceneGraph> m_env_ = nullptr;
        std::unordered_map<int, std::vector<double>> m_heuristic_cache_ = {};

    public:
        LlmSceneGraphHeuristic(std::shared_ptr<Setting> setting, std::shared_ptr<env::EnvironmentLTLSceneGraph> env);

        [[nodiscard]] double
        operator()(const env::EnvironmentState &env_state) const override;
    };

}  // namespace erl::search_planning

// ReSharper disable CppInconsistentNaming
namespace YAML {
    template<>
    struct convert<erl::search_planning::LlmSceneGraphHeuristic::Setting::LlmWaypoint> {
        static Node
        encode(const erl::search_planning::LlmSceneGraphHeuristic::Setting::LlmWaypoint &rhs) {
            Node node;
            switch (rhs.type) {
                case erl::env::AtomicProposition::Type::kEnterRoom: {
                    node = fmt::format("move({}, {})", rhs.uuid1, rhs.uuid2);
                    break;
                }
                case erl::env::AtomicProposition::Type::kReachObject: {
                    node = fmt::format("reach({}, {})", rhs.uuid1, rhs.uuid2);
                    break;
                }
                default:
                    throw std::runtime_error("Unknown LLMWaypoint type.");
            }
            return node;
        }

        static bool
        decode(const Node &node, erl::search_planning::LlmSceneGraphHeuristic::Setting::LlmWaypoint &rhs) {
            if (!node.IsScalar()) { return false; }
            if (auto str = node.as<std::string>(); str.find("move(") != std::string::npos) {
                rhs.type = erl::env::AtomicProposition::Type::kEnterRoom;
                std::stringstream ss(str);
                std::string tmp;
                std::getline(ss, tmp, '(');
                std::getline(ss, tmp, ',');
                rhs.uuid1 = std::stoi(tmp);
                std::getline(ss, tmp, ')');
                rhs.uuid2 = std::stoi(tmp);
            } else if (str.find("reach(") != std::string::npos) {
                rhs.type = erl::env::AtomicProposition::Type::kReachObject;
                std::stringstream ss(str);
                std::string tmp;
                std::getline(ss, tmp, '(');
                std::getline(ss, tmp, ',');
                rhs.uuid1 = std::stoi(tmp);
                std::getline(ss, tmp, ')');
                rhs.uuid2 = std::stoi(tmp);
            } else {
                throw std::runtime_error("Unknown LLMWaypoint string: " + str);
            }
            return true;
        }
    };

    template<>
    struct convert<erl::search_planning::LlmSceneGraphHeuristic::Setting> {
        static Node
        encode(const erl::search_planning::LlmSceneGraphHeuristic::Setting &rhs) {
            Node node;
            node = rhs.llm_paths;
            return node;
        }

        static bool
        decode(const Node &node, erl::search_planning::LlmSceneGraphHeuristic::Setting &rhs) {
            using namespace erl::search_planning;
            using LLMPath = std::vector<LlmSceneGraphHeuristic::Setting::LlmWaypoint>;
            if (!node.IsMap()) { return false; }
            rhs.llm_paths = node.as<std::map<int, std::map<uint32_t, LLMPath>>>();
            return true;
        }
    };
}  // namespace YAML

// ReSharper restore CppInconsistentNaming
