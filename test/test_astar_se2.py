import time
import unittest

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from erl_path_planning.common.storage import GridMapInfo2D
from erl_path_planning.common.storage import GridMapUnsigned2D
from erl_path_planning.env import load_ddc_motion_primitives_from_yaml
from erl_path_planning import AStar
from erl_path_planning import PlanningSe2
from erl_path_planning.env.data.house_expo import HouseExpoMap
from erl_path_planning.env.data.house_expo.list_data import get_map_and_traj_files


class TestAStarSe2(unittest.TestCase):
    # def test_plan_with_point_collision_check(self):
    #     num_orientations = 25
    #     collision_check_dt = 0.05
    #     metric_start_coords = np.array([10.0, 90.0, 0.0])
    #     metric_goal_coords = np.array([50.0, 1.0, 0.0])
    #     metric_goal_tolerance = np.array([0.25, 0.25, 0.25])
    #     terminal_cost = np.array([0.0])
    #     map_shape = np.array([1001, 1001])
    #     map_min = np.array([0.0, 0.0])
    #     map_max = np.array([100.0, 100.0])
    #     grid_map_info = GridMapInfo2D(map_shape, map_min, map_max)
    #     data = np.loadtxt(
    #         "circles_map_1001x1001.txt", dtype=np.uint8, delimiter=" "
    #     )  # x to the right, y to the bottom, row-major (along x first)
    #     grid_map = GridMapUnsigned2D(
    #         grid_map_info, data.transpose().flatten()
    #     )  # x to the bottom, y to the right, row-major (along y first)
    #     motion_primitives = load_ddc_motion_primitives_from_yaml("ddc_motion_primitives.yaml")
    #     planning_se2 = PlanningSe2(
    #         metric_goals_coords=metric_goal_coords,
    #         metric_goals_tolerance=metric_goal_tolerance,
    #         terminal_costs=terminal_cost,
    #         collision_check_dt=collision_check_dt,
    #         motion_primitives=motion_primitives,
    #         grid_map=grid_map,
    #         num_thetas=num_orientations,
    #     )
    #     result = AStar(
    #         metric_start_coords,
    #         planning_se2,
    #         eps=2,
    #         max_num_reached_goals=1,
    #         max_num_iterations=-1,
    #         reopen_inconsistent=False,
    #         log=False,
    #     ).plan()
    #     self.assertEqual(len(result.paths[0]), 97)
    #     self.assertEqual(result.path_costs[0], 96)
    #     self.assertEqual(len(result.action_ids[0]), 96)
    #     self.assertEqual(len(result.inconsistent_list), 0)
    #
    # def test_plan_with_triangle_shape(self):
    #     num_orientations = 25
    #     collision_check_dt = 0.05
    #     metric_start_coords = np.array([10.0, 90.0, 0.0])
    #     metric_goal_coords = np.array([94.0, 7.0, 0.0])
    #     metric_goal_tolerance = np.array([0.25, 0.25, 0.53])
    #     terminal_cost = np.array([0.0])
    #     map_shape = np.array([1001, 1001])
    #     map_min = np.array([0.0, 0.0])
    #     map_max = np.array([100.0, 100.0])
    #     grid_map_info = GridMapInfo2D(map_shape, map_min, map_max)
    #     data = np.loadtxt(
    #         "circles_map_1001x1001.txt", dtype=np.uint8, delimiter=" "
    #     )  # x to the right, y to the bottom, row-major (along x first)
    #     grid_map = GridMapUnsigned2D(
    #         grid_map_info, data.transpose().flatten()
    #     )  # x to the bottom, y to the right, row-major (along y first)
    #     motion_primitives = load_ddc_motion_primitives_from_yaml("ddc_motion_primitives.yaml")
    #     triangle_vertices = np.array([[-0.3, 0.3, -0.3], [-0.2, 0.0, 0.2]]) * 3
    #     # metric_grids = grid_map_info.get_metric_coordinates_of_filled_metric_polygon(triangle_vertices)
    #     # metric_grids = grid_map_info.cell_to_meter_for_points(compute_pixels_of_polygon_contour(grid_map_info.meter_to_cell_for_points(triangle_vertices)))
    #     planning_se2 = PlanningSe2(
    #         metric_goals_coords=metric_goal_coords,
    #         metric_goals_tolerance=metric_goal_tolerance,
    #         terminal_costs=terminal_cost,
    #         collision_check_dt=collision_check_dt,
    #         motion_primitives=motion_primitives,
    #         grid_map=grid_map,
    #         num_thetas=num_orientations,
    #         inflate_scale=1.0,
    #         shape_metric_vertices=triangle_vertices,
    #     )
    #     result = AStar(
    #         metric_start_coords,
    #         planning_se2,
    #         eps=2,
    #         max_num_reached_goals=1,
    #         max_num_iterations=-1,
    #         reopen_inconsistent=False,
    #         log=False,
    #     ).plan()
    #
    #     plt.figure(figsize=(10, 10))
    #     extend = np.array([map_min[0], map_max[0], map_min[1], map_max[1]])
    #     plt.imshow(data[::-1], cmap="binary_r", extent=extend)  # y to the bottom -> y to the top
    #
    #     path = np.array(result.paths[0])  # rough path
    #     np.savetxt("path.txt", path)
    #     plt.plot(path[:, 0], path[:, 1], "r-")
    #
    #     body_patch = plt.Polygon(triangle_vertices.transpose(), closed=False, color="b", fill=True)
    #     plt.gca().add_patch(body_patch)
    #
    #     def update_body_patch(x, y, theta):
    #         s = np.sin(theta)
    #         c = np.cos(theta)
    #         rot = np.array([[c, -s], [s, c]])
    #         body_patch.set_xy((rot @ triangle_vertices + np.array([[x], [y]])).T)
    #
    #     update_body_patch(metric_start_coords[0], metric_start_coords[1], metric_start_coords[2])
    #
    #     path_line = plt.plot(metric_start_coords[0], metric_start_coords[1], "g-")[0]
    #
    #     def update_path_line(x, y):
    #         path_line.set_data(x, y)
    #
    #     state = metric_start_coords
    #     path = []
    #     for action_id in result.action_ids[0]:
    #         trajectory = planning_se2.forward_action_in_metric_space(state, action_id, 0.01).T
    #
    #         for metric_state in trajectory:
    #             update_body_patch(metric_state[0], metric_state[1], metric_state[2])
    #             path.append(metric_state)
    #             update_path_line(np.array(path)[:, 0], np.array(path)[:, 1])
    #
    #         state = trajectory[-1]
    #
    #         plt.pause(0.1)
    #     plt.show(block=False)
    #     plt.pause(5)

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
        num_orientations = 91
        collision_check_dt = 0.01
        metric_start_coords = np.array([7.0, 7.0, 0.0])
        metric_goal_coords = np.array([10.0, 1.0, 3.1])
        metric_goal_tolerance = np.array([0.25, 0.25, 0.25])
        terminal_cost = np.array([0.0])
        n = 10
        t_total = 0.0
        for _ in range(n):
            planning_se2 = PlanningSe2(
                metric_goals_coords=metric_goal_coords,
                metric_goals_tolerance=metric_goal_tolerance,
                terminal_costs=terminal_cost,
                collision_check_dt=collision_check_dt,
                motion_primitives=motion_primitives,
                grid_map=grid_map,
                num_thetas=num_orientations,
                inflate_scale=1.5,
                shape_metric_vertices=triangle_vertices,
            )
            t0 = time.time()
            result = AStar(
                metric_start_coords,
                planning_se2,
                eps=4,
                max_num_reached_goals=1,
                max_num_iterations=-1,
                reopen_inconsistent=False,
                log=False,
            ).plan()
            t_total += time.time() - t0
        print("time: ", t_total / n, flush=True)

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
            plt.pause(0.1)

        plt.show(block=False)
        plt.pause(5)


if __name__ == "__main__":
    unittest.main()
