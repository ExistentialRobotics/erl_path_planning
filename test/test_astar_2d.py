import unittest
import matplotlib.pyplot as plt
import numpy as np
from erl_search_planning.common.storage import GridMapInfo2D
from erl_search_planning.common.storage import GridMapUnsigned2D
from erl_search_planning.search_planning import AStar
from erl_search_planning.search_planning import Planning2D


class TestAStar2D(unittest.TestCase):
    def test_plan(self):
        grid_map_data = np.array(
            [
                [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            ]
        )  # x to bottom, y to right, along y first
        grid_map_info = GridMapInfo2D(np.array([15, 15]), np.array([0, 0]), np.array([15, 15]))
        grid_map = GridMapUnsigned2D(grid_map_info, grid_map_data.flatten())  # x to bottom, y to right, along y first
        start = grid_map_info.grid_to_meter_for_points(np.array([1, 1]))
        goal = grid_map_info.grid_to_meter_for_points(np.array([1, 10]))
        goal_tolerance = np.zeros(2)
        terminal_cost = np.array([0])
        planning_interface = Planning2D(
            metric_goals_coords=goal,
            metric_goals_tolerance=goal_tolerance,
            terminal_costs=terminal_cost,
            allow_diagonal=True,
            step_size=1,
            grid_map=grid_map,
        )
        result = AStar(start, planning_interface, eps=1, max_num_reached_goals=-1, max_num_iterations=-1).plan()
        print(result.path_costs[0])
        self.assertTrue(np.isclose(result.path_costs[0], 15.485281))


if __name__ == "__main__":
    unittest.main()
