AMRA*
=====

AMRA* is a anytime multi-resolution multi-heuristic A* algorithm.

# Usage

## C++

Check [test_amra_star_2d.cpp](../test/gtest/test_amra_star_2d.cpp) for an example.

## Python

Working on it.

# Visualization Examples

You can use the [Python script](../python/erl_search_planning/amra_star/visualize.py) to visualize the search result
stored in `*.solution` file.

## 2D Grid Map

### Paths over Time

| Plan Iteration | 1                                                                                 | 2                                                                                 |
|----------------|-----------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| Path           | <img src="assets/amra_astar/Boston_0_1024.map-path-plan_iter_1.png" width=400px/> | <img src="assets/amra_astar/Boston_0_1024.map-path-plan_iter_2.png" width=400px/> |
| Plan Iteration | 3                                                                                 | 4                                                                                 |
| Path           | <img src="assets/amra_astar/Boston_0_1024.map-path-plan_iter_3.png" width=400px/> | <img src="assets/amra_astar/Boston_0_1024.map-path-plan_iter_4.png" width=400px/> |
| Plan Iteration | 5                                                                                 | 6                                                                                 |
| Path           | <img src="assets/amra_astar/Boston_0_1024.map-path-plan_iter_5.png" width=400px/> | <img src="assets/amra_astar/Boston_0_1024.map-path-plan_iter_6.png" width=400px/> |

### Opened & Closed States

|              | Opened States                                                                                 | Closed States                                                                                     |
|--------------|-----------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------|
| Anchor Level | <img src="assets/amra_astar/Boston_0_1024.map-opened_states-heuristic_id_0.gif" width=400px/> | <img src="assets/amra_astar/Boston_0_1024.map-closed_states-resolution_level_0.gif" width=400px/> |
| Resolution 1 | <img src="assets/amra_astar/Boston_0_1024.map-opened_states-heuristic_id_1.gif" width=400px/> | <img src="assets/amra_astar/Boston_0_1024.map-closed_states-resolution_level_1.gif" width=400px/> |
| Resolution 2 | <img src="assets/amra_astar/Boston_0_1024.map-opened_states-heuristic_id_2.gif" width=400px/> | <img src="assets/amra_astar/Boston_0_1024.map-closed_states-resolution_level_2.gif" width=400px/> |
| Resolution 3 | <img src="assets/amra_astar/Boston_0_1024.map-opened_states-heuristic_id_3.gif" width=400px/> | <img src="assets/amra_astar/Boston_0_1024.map-closed_states-resolution_level_3.gif" width=400px/> |

### Inconsistent States

<img src="assets/amra_astar/Boston_0_1024.map-inconsistent_states.gif" width=400px/>

For more examples, please check [assets](assets) folder.

# References

- [AMRA* Paper](https://arxiv.org/pdf/2110.05328.pdf)
- [Original AMRA* Implementation](https://github.com/dhruvms/amra)
