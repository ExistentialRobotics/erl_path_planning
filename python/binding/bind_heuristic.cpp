#include "erl_path_planning/pybind11_heuristic.hpp"

void
BindHeuristics(py::module &m) {
    BindHeuristicBaseImpl<float, 2>(m, "HeuristicBase2Df");
    BindHeuristicBaseImpl<double, 2>(m, "HeuristicBase2Dd");
    BindHeuristicBaseImpl<float, 3>(m, "HeuristicBase3Df");
    BindHeuristicBaseImpl<double, 3>(m, "HeuristicBase3Dd");
    BindHeuristicBaseImpl<float, 4>(m, "HeuristicBase4Df");
    BindHeuristicBaseImpl<double, 4>(m, "HeuristicBase4Dd");

    BindEuclideanDistanceHeuristicImpl<float, 2>(m, "EuclideanDistanceHeuristic2D");
    BindEuclideanDistanceHeuristicImpl<double, 2>(m, "EuclideanDistanceHeuristic2Dd");
    BindEuclideanDistanceHeuristicImpl<float, 3>(m, "EuclideanDistanceHeuristic3D");
    BindEuclideanDistanceHeuristicImpl<double, 3>(m, "EuclideanDistanceHeuristic3Dd");
    BindEuclideanDistanceHeuristicImpl<float, 4>(m, "EuclideanDistanceHeuristic4D");
    BindEuclideanDistanceHeuristicImpl<double, 4>(m, "EuclideanDistanceHeuristic4Dd");

    BindSe2HeuristicImpl<float>(m, "Se2HeuristicF");
    BindSe2HeuristicImpl<double>(m, "Se2HeuristicD");

    BindManhattanDistanceHeuristicImpl<float, 2>(m, "ManhattanDistanceHeuristic2D");
    BindManhattanDistanceHeuristicImpl<double, 2>(m, "ManhattanDistanceHeuristic2Dd");
    BindManhattanDistanceHeuristicImpl<float, 3>(m, "ManhattanDistanceHeuristic3D");
    BindManhattanDistanceHeuristicImpl<double, 3>(m, "ManhattanDistanceHeuristic3Dd");
    BindManhattanDistanceHeuristicImpl<float, 4>(m, "ManhattanDistanceHeuristic4D");
    BindManhattanDistanceHeuristicImpl<double, 4>(m, "ManhattanDistanceHeuristic4Dd");

    BindDictionaryHeuristicImpl<float, 2>(m, "DictionaryHeuristic2Df");
    BindDictionaryHeuristicImpl<double, 2>(m, "DictionaryHeuristic2Dd");
    BindDictionaryHeuristicImpl<float, 3>(m, "DictionaryHeuristic3Df");
    BindDictionaryHeuristicImpl<double, 3>(m, "DictionaryHeuristic3Dd");
    BindDictionaryHeuristicImpl<float, 4>(m, "DictionaryHeuristic4Df");
    BindDictionaryHeuristicImpl<double, 4>(m, "DictionaryHeuristic4Dd");

    BindMultiGoalsHeuristicImpl<float, 2>(m, "MultiGoalsHeuristic2Df");
    BindMultiGoalsHeuristicImpl<double, 2>(m, "MultiGoalsHeuristic2Dd");
    BindMultiGoalsHeuristicImpl<float, 3>(m, "MultiGoalsHeuristic3Df");
    BindMultiGoalsHeuristicImpl<double, 3>(m, "MultiGoalsHeuristic3Dd");
    BindMultiGoalsHeuristicImpl<float, 4>(m, "MultiGoalsHeuristic4Df");
    BindMultiGoalsHeuristicImpl<double, 4>(m, "MultiGoalsHeuristic4Dd");

    BindLTL2DHeuristicImpl<float>(m, "LinearTemporalLogicHeuristic2Df");
    BindLTL2DHeuristicImpl<double>(m, "LinearTemporalLogicHeuristic2Dd");

    BindLTL3DHeuristicImpl<float>(m, "LinearTemporalLogicHeuristic3Df");
    BindLTL3DHeuristicImpl<double>(m, "LinearTemporalLogicHeuristic3Dd");
}
