import time
import unittest

import matplotlib.pyplot as plt
import numpy as np
from erl_common.storage import GridMapInfo2D
from erl_common.storage import GridMapUnsigned2D
from erl_env import load_ddc_motion_primitives_from_yaml
from erl_geometry.house_expo import HouseExpoMap
from erl_geometry.house_expo.list_data import get_map_and_traj_files
from erl_search_planning.astar import AStar
from erl_search_planning import PlanningInterface


class TestAStarSe2(unittest.TestCase):
    def test_plan_with_house_expo_map(self):
        map_file = get_map_and_traj_files()[1451][0]
        # house_expo_map = HouseExpoMap(map_file, 0.15)
        house_expo_map = HouseExpoMap(map_file)
        vertices = house_expo_map.meter_space.surface.vertices
        print(vertices.shape)
        map_min = np.min(vertices, axis=1)
        map_max = np.max(vertices, axis=1)
        grid_map_info = GridMapInfo2D(
            min=map_min,
            max=map_max,
            resolution=np.array([0.05, 0.05]),
            padding=np.array([4, 4]),
        )
        map_image = house_expo_map.meter_space.generate_map_image(grid_map_info)
        print(map_image.flags["C_CONTIGUOUS"])
        print(map_image.flags["F_CONTIGUOUS"])
        print(map_image.min(), map_image.max(), flush=True)
        grid_map = GridMapUnsigned2D(grid_map_info, 255 - map_image[::-1].T.flatten())
        motion_primitives = load_ddc_motion_primitives_from_yaml("ddc_motion_primitives.yaml")
        triangle_vertices = np.array([[-0.3, 0.3, -0.3], [-0.2, 0.0, 0.2]]) * 0.5
        # metric_grids = grid_map_info.get_metric_coordinates_of_filled_metric_polygon(triangle_vertices)
        # metric_grids = grid_map_info.cell_to_meter_for_points(compute_pixels_of_polygon_contour(grid_map_info.meter_to_cell_for_points(triangle_vertices)))
        num_orientations = 51
        collision_check_dt = 0.01
        metric_start_coords = np.array([7.0, 7.0, 0.0])
        metric_goal_coords = np.array([10.0, 1.0, 1.57])
        metric_goal_tolerance = np.array([0.25, 0.25, 2 * np.pi / num_orientations])
        terminal_cost = np.array([0.0])
        # planning_grid_se2 = PlanningGridSe2(
        #     metric_goals_coords=metric_goal_coords,
        #     metric_goals_tolerance=metric_goal_tolerance,
        #     terminal_costs=terminal_cost,
        #     linear_velocity_min=0.0,
        #     linear_velocity_max=1.0,
        #     linear_velocity_step=0.1,
        #     euclidean_square_distance_cost_weight=1.0,
        #     angular_velocity_min=-1.0,
        #     angular_velocity_max=1.0,
        #     angular_velocity_step=0.1,
        #     angular_square_distance_cost_weight=1.0,
        #     duration_step=0.1,
        #     duration=1.0,
        #     max_step_size=1,
        #     grid_map=grid_map,
        #     num_orientations=num_orientations,
        #     inflate_scale=1.0,
        #     shape_metric_vertices=triangle_vertices,
        # )

        # step_size = (1,)
        # grid_map = (grid_map,)
        # num_orientations = (num_orientations,)
        # inflate_scale = (1.0,)
        # shape_metric_vertices = (triangle_vertices,)

        planning_grid_se2 = PlanningInterface(
            metric_goals_coords=metric_goal_coords,
            metric_goals_tolerances=metric_goal_tolerance,
            terminal_costs=terminal_cost,
        )
        t0 = time.time()
        for _ in range(1):
            result = AStar(
                metric_start_coords,
                planning_grid_se2,
                eps=10,
                max_num_reached_goals=1,
                max_num_iterations=-1,
                reopen_inconsistent=False,
                log=False,
            ).plan()
        print("time: ", (time.time() - t0) / 1, flush=True)

        plt.figure(figsize=(10, 10))
        extend = np.array([map_min[0], map_max[0], map_min[1], map_max[1]])
        plt.imshow(map_image, cmap="binary_r", extent=extend)  # y to the bottom -> y to the top
        # plt.show()

        np.set_printoptions(linewidth=10000000)
        print(result.paths[0])

        path = result.paths[0].T[::-10][::-1]  # rough path
        plt.plot(path[:, 0], path[:, 1], "r-")

        body_patch = plt.Polygon(triangle_vertices.transpose(), closed=False, color="b", fill=True)
        plt.gca().add_patch(body_patch)

        def update_body_patch(x, y, theta):
            s = np.sin(theta)
            c = np.cos(theta)
            rot = np.array([[c, -s], [s, c]])
            body_patch.set_xy((rot @ triangle_vertices + np.array([[x], [y]])).T)

        update_body_patch(metric_start_coords[0], metric_start_coords[1], metric_start_coords[2])

        path_line = plt.plot(metric_start_coords[0], metric_start_coords[1], "g-")[0]

        def update_path_line(x, y):
            path_line.set_data(x, y)

        for i in range(path.shape[0]):
            update_body_patch(path[i, 0], path[i, 1], path[i, 2])
            update_path_line(path[: i + 1, 0], path[: i + 1, 1])
            plt.pause(0.5)

        plt.show(block=False)
        plt.pause(5)


if __name__ == "__main__":
    unittest.main()
