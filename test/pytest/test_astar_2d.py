import os
import unittest
import matplotlib.pyplot as plt
import numpy as np
from erl_common.storage import GridMapInfo2D
from erl_common.storage import GridMapUnsigned2D
from erl_env import Environment2D
from erl_search_planning.astar import AStar
from erl_search_planning import PlanningInterface


class TestAStar2D(unittest.TestCase):
    def test_small_map(self):
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
        grid_map_info = GridMapInfo2D(map_shape=np.array([15, 15]), min=np.array([0, 0]), max=np.array([15, 15]))
        grid_map = GridMapUnsigned2D(grid_map_info, grid_map_data.flatten())  # row-major

        env_setting = Environment2D.Setting()
        env_setting.set_grid_motion_primitive(max_axis_step=1, allow_diagonal=True)
        env = Environment2D(grid_map, env_setting)

        metric_start_coords = grid_map_info.grid_to_meter_for_points(np.array([1, 1])).flatten()
        metric_goal_coords = grid_map_info.grid_to_meter_for_points(np.array([1, 10])).flatten()
        metric_goal_tolerance = np.zeros(2)
        planning_interface = PlanningInterface(
            env,
            metric_start_coords,
            metric_goal_coords,
            metric_goal_tolerance,
            0.0,
        )

        astar_setting = AStar.Setting()
        result = AStar(planning_interface, astar_setting).plan()

        # plot
        print("start:", metric_start_coords)
        print("goal:", metric_goal_coords)
        print("path:\n", result.path.T)
        print("cost:", result.cost)
        plt.imshow(
            grid_map_data,
            cmap="gray",
            extent=[
                grid_map_info.min_at(1),  # left
                grid_map_info.max_at(1),  # right
                grid_map_info.max_at(0),  # bottom
                grid_map_info.min_at(0),  # top
            ],
        )
        plt.plot(result.path[1], result.path[0], "r-", linewidth=2)
        plt.scatter(metric_start_coords[1], metric_start_coords[0], c="g", marker="o", s=100)
        plt.scatter(metric_goal_coords[1], metric_goal_coords[0], c="r", marker="o", s=100)
        plt.xlabel("y (m)")
        plt.ylabel("x (m)")
        plt.show()
        self.assertTrue(np.isclose(result.cost, 16.071067811865476))

    def test_large_map(self):
        grid_map_data = np.loadtxt(os.path.join(os.path.dirname(__file__), "data/astar_2d/circles_map_1001x1001.txt"))
        grid_map_info = GridMapInfo2D(map_shape=np.array([1001, 1001]), min=np.array([0, 0]), max=np.array([100, 100]))
        grid_map = GridMapUnsigned2D(grid_map_info, grid_map_data.flatten())  # row-major

        env_setting = Environment2D.Setting()
        env_setting.set_grid_motion_primitive(max_axis_step=1, allow_diagonal=True)
        env = Environment2D(grid_map, env_setting)

        metric_start_coords = np.array([90.0, 10.0])
        metric_goal_coords = np.array([1.0, 50.0])
        grid_start_coords = grid_map_info.meter_to_grid_for_points(metric_start_coords).flatten()
        grid_goal_coords = grid_map_info.meter_to_grid_for_points(metric_goal_coords).flatten()
        metric_start_coords = grid_map_info.grid_to_meter_for_points(grid_start_coords).flatten()
        metric_goal_coords = grid_map_info.grid_to_meter_for_points(grid_goal_coords).flatten()
        metric_goal_tolerance = np.zeros(2)
        planning_interface = PlanningInterface(
            env,
            metric_start_coords,
            metric_goal_coords,
            metric_goal_tolerance,
            0.0,
        )

        astar_setting = AStar.Setting()
        result = AStar(planning_interface, astar_setting).plan()

        # plot
        print("start:", metric_start_coords)
        print("goal:", metric_goal_coords)
        print("path:\n", result.path.T)
        print("cost:", result.cost)
        plt.imshow(
            grid_map_data,
            cmap="gray",
            extent=[
                grid_map_info.min_at(1),  # left
                grid_map_info.max_at(1),  # right
                grid_map_info.max_at(0),  # bottom
                grid_map_info.min_at(0),  # top
            ],
        )
        plt.plot(result.path[1], result.path[0], "r-", linewidth=2)
        plt.scatter(metric_start_coords[1], metric_start_coords[0], c="g", marker="o", s=100)
        plt.scatter(metric_goal_coords[1], metric_goal_coords[0], c="r", marker="o", s=100)
        plt.xlabel("y (m)")
        plt.ylabel("x (m)")
        plt.show()
        self.assertTrue(np.isclose(result.cost, 105.46307941550766))
        self.assertEqual(len(result.action_coords), 890)


if __name__ == "__main__":
    unittest.main()
