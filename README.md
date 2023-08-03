`erl_search_planning`
=====================

This CMake project provides `erl_search_planning` which is a collection of search-based planning algorithms.

# AMRA*

- `AMRAStar`
  - `PlanningInterfaceMultiResolutions`
    - `environments`: [`EnvironmentAnchor`, `Environment_1`, `Environment_2`, ...]
    - `heuristics`: [`<heuristic_0, 0>`, `<heuristic_1, r1>`, `<heuristic_2, r2>`, ...]
    - `start`
    - `goals`
    - `goals tolerance`
