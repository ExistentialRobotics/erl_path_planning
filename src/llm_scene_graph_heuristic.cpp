#include "erl_path_planning/llm_scene_graph_heuristic.hpp"

namespace erl::path_planning {

    template class LlmSceneGraphHeuristic<float>;
    template class LlmSceneGraphHeuristic<double>;
}  // namespace erl::path_planning
