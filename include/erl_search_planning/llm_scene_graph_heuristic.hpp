#pragma once

#include "heuristic.hpp"

#include "erl_common/yaml.hpp"
#include "erl_env/environment_ltl_scene_graph.hpp"

#include <memory>

namespace erl::search_planning {

    template<typename Dtype>
    class LlmSceneGraphHeuristic : public HeuristicBase<Dtype, 4> {

    public:
        using Env = env::EnvironmentLTLSceneGraph<Dtype>;
        using EnvState = typename Env::State;
        using MetricState = typename Env::MetricState;
        using AtomicProposition = typename Env::AtomicProposition;

        struct LlmWaypoint {
            using Type = typename AtomicProposition::Type;
            Type type;
            int uuid1;
            int uuid2;

            struct YamlConvertImpl {
                static YAML::Node
                encode(const LlmWaypoint &waypoint) {
                    YAML::Node node;
                    ERL_YAML_SAVE_ENUM_ATTR(
                        node,
                        waypoint,
                        type,
                        {"kNA", Type::kNA},
                        {"kEnterRoom", Type::kEnterRoom},
                        {"kReachObject", Type::kReachObject});
                    ERL_YAML_SAVE_ATTR(node, waypoint, uuid1);
                    ERL_YAML_SAVE_ATTR(node, waypoint, uuid2);
                    return node;
                }

                static bool
                decode(const YAML::Node &node, LlmWaypoint &waypoint) {
                    if (!node.IsMap()) { return false; }
                    ERL_YAML_LOAD_ENUM_ATTR(
                        node,
                        waypoint,
                        type,
                        {"kNA", Type::kNA},
                        {"kEnterRoom", Type::kEnterRoom},
                        {"kReachObject", Type::kReachObject});
                    ERL_YAML_LOAD_ATTR(node, waypoint, uuid1);
                    ERL_YAML_LOAD_ATTR(node, waypoint, uuid2);
                    return true;
                }
            };
        };

        struct Setting : public common::Yamlable<Setting> {
            std::map<int, std::map<uint32_t, std::vector<LlmWaypoint>>> llm_paths;

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting) {
                    YAML::Node node;
                    ERL_YAML_SAVE_ATTR(node, setting, llm_paths);
                    return node;
                }

                static bool
                decode(const YAML::Node &node, Setting &setting) {
                    if (!node.IsMap()) { return false; }
                    ERL_YAML_LOAD_ATTR(node, setting, llm_paths);
                    return true;
                }
            };
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<Env> m_env_ = nullptr;
        std::unordered_map<int, std::vector<Dtype>> m_heuristic_cache_ = {};

    public:
        LlmSceneGraphHeuristic(std::shared_ptr<Setting> setting, std::shared_ptr<Env> env)
            : m_setting_(std::move(setting)),
              m_env_(std::move(env)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
            ERL_ASSERTM(m_env_ != nullptr, "env is nullptr.");

            auto fsa = m_env_->GetFiniteStateAutomaton();
            auto scene_graph = m_env_->GetSceneGraph();

            uint32_t num_fsa_states = fsa->GetSetting()->num_states;
            constexpr Dtype inf = std::numeric_limits<Dtype>::infinity();

            for (auto &room_uuid: m_env_->GetSceneGraph()->room_uuids) {
                auto &heuristic_cache_for_room = m_heuristic_cache_[room_uuid];
                heuristic_cache_for_room.resize(num_fsa_states, inf);
                for (uint32_t fsa_state = 0; fsa_state < num_fsa_states; ++fsa_state) {
                    if (fsa->IsAcceptingState(fsa_state)) {
                        heuristic_cache_for_room[fsa_state] = 0.0f;
                        continue;
                    }
                    if (fsa->IsSinkState(fsa_state)) { continue; }

                    auto &llm_path = m_setting_->llm_paths.at(room_uuid).at(fsa_state);
                    // LLM suggests only one step, which will be used when operator() is called.
                    if (llm_path.size() <= 1) {
                        heuristic_cache_for_room[fsa_state] = 0.0f;
                        continue;
                    }

                    Dtype h = 0;
                    const std::size_t n = llm_path.size();
                    for (std::size_t i = 1; i < n; ++i) {
                        auto &waypoint = llm_path[i];
                        using namespace env::scene_graph;
                        switch (waypoint.type) {
                            case AtomicProposition::Type::kEnterRoom: {
                                auto room1 = scene_graph->template GetNode<Room>(waypoint.uuid1);
                                auto room2 = scene_graph->template GetNode<Room>(waypoint.uuid2);
                                h += (room1->location - room2->location).norm();
                                break;
                            }
                            case AtomicProposition::Type::kReachObject: {
                                auto room = scene_graph->template GetNode<Room>(waypoint.uuid1);
                                auto object = scene_graph->template GetNode<Object>(waypoint.uuid2);
                                h += (room->location - object->location).norm();
                                break;
                            }
                            default:
                                throw std::runtime_error("Unknown LlmWaypoint type.");
                        }
                    }
                    heuristic_cache_for_room[fsa_state] = h;
                }
            }
        }

        [[nodiscard]] Dtype
        operator()(const EnvState &env_state) const override {
            // use the precomputed cost maps and path maps to get a more accurate heuristic, but
            // paths generated by LLM sometimes omit some rooms, causing instructions like
            // move(uuid1, uuid2) while room of uuid1 and room of uuid2 are not adjacent. This makes
            // it impossible to use the precomputed cost maps and path maps. Unless we use a backup
            // process to fix such instructions. But this makes the LLM heuristics pointless.

            if (env_state.grid[0] == env::VirtualStateValue::kGoal) { return 0.0f; }

            const auto &scene_graph = m_env_->GetSceneGraph();
            const auto &fsa = m_env_->GetFiniteStateAutomaton();
            const auto &room_maps = m_env_->GetRoomMaps();
            int x = env_state.grid[0];
            int y = env_state.grid[1];
            int z = env_state.grid[2];
            int q = env_state.grid[3];
            if (fsa->IsAcceptingState(q)) { return 0.0f; }  // accepting state

            int room_id = room_maps.at(z).template at<int>(x, y);
            int room_uuid = scene_graph->id_to_room.at(room_id)->uuid;
            auto &llm_path = m_setting_->llm_paths.at(room_uuid).at(q);

            Dtype h = m_heuristic_cache_.at(room_uuid)[q];
            int next_uuid = llm_path[0].uuid2;
            using namespace env::scene_graph;
            switch (llm_path[0].type) {
                case AtomicProposition::Type::kEnterRoom: {
                    const auto &room = scene_graph->template GetNode<Room>(next_uuid);
                    Dtype dx = room->location[0] - env_state.metric[0];
                    Dtype dy = room->location[1] - env_state.metric[1];
                    Dtype dz = room->location[2] - env_state.metric[2];
                    h += std::sqrt(dx * dx + dy * dy + dz * dz);
                    break;
                }
                case AtomicProposition::Type::kReachObject: {
                    const auto &object = scene_graph->template GetNode<Object>(next_uuid);
                    Dtype dx = object->location[0] - env_state.metric[0];
                    Dtype dy = object->location[1] - env_state.metric[1];
                    Dtype dz = object->location[2] - env_state.metric[2];
                    h += std::sqrt(dx * dx + dy * dy + dz * dz);
                    break;
                }
                default:
                    throw std::runtime_error("Unknown LlmWaypoint type.");
            }

            return h;
        }
    };

    extern template class LlmSceneGraphHeuristic<float>;
    extern template class LlmSceneGraphHeuristic<double>;

}  // namespace erl::search_planning

template<>
struct YAML::convert<erl::search_planning::LlmSceneGraphHeuristic<float>::LlmWaypoint>
    : public erl::search_planning::LlmSceneGraphHeuristic<float>::LlmWaypoint::YamlConvertImpl {};

template<>
struct YAML::convert<erl::search_planning::LlmSceneGraphHeuristic<float>::Setting>
    : public erl::search_planning::LlmSceneGraphHeuristic<float>::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::search_planning::LlmSceneGraphHeuristic<double>::LlmWaypoint>
    : public erl::search_planning::LlmSceneGraphHeuristic<double>::LlmWaypoint::YamlConvertImpl {};

template<>
struct YAML::convert<erl::search_planning::LlmSceneGraphHeuristic<double>::Setting>
    : public erl::search_planning::LlmSceneGraphHeuristic<double>::Setting::YamlConvertImpl {};
