import os.path
from dataclasses import dataclass
from io import StringIO
from typing import TextIO

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from tqdm import tqdm


@dataclass
class PlanIterationSolution:
    plan_itr: int
    w1: float
    w2: float
    goal_index: int
    cost: float
    num_waypoints: int
    path: pd.DataFrame
    num_actions: int
    action_coords: pd.DataFrame

    @staticmethod
    def load(file: TextIO):
        plan_itr = int(file.readline().strip().split(":")[1].strip())
        w1 = float(file.readline().strip().split(":")[1].strip())
        w2 = float(file.readline().strip().split(":")[1].strip())
        goal_index = int(file.readline().strip().split(":")[1].strip())
        cost = float(file.readline().strip().split(":")[1].strip())
        num_waypoints = int(file.readline().strip().split(":")[1].strip())
        assert file.readline().strip() == "path:", f"path data is missing for plan iteration {plan_itr}"
        path = pd.read_csv(
            StringIO("".join([file.readline() for _ in range(num_waypoints + 1)])),
            sep=",",
            skipinitialspace=True,
        )
        num_actions = int(file.readline().strip().split(":")[1].strip())
        assert (
            file.readline().strip() == "action_coords:"
        ), f"action_coords data is missing for plan iteration {plan_itr}"
        action_coords = pd.read_csv(
            StringIO("".join([file.readline() for _ in range(num_actions + 1)])),
            sep=",",
            header=None,
            skipinitialspace=True,
        )
        return PlanIterationSolution(
            plan_itr=plan_itr,
            w1=w1,
            w2=w2,
            goal_index=goal_index,
            cost=cost,
            num_waypoints=num_waypoints,
            path=path,
            num_actions=num_actions,
            action_coords=action_coords,
        )


class AMRAStarSolution:
    def __init__(self, sol_filepath: str):
        self.sol_filepath = sol_filepath

        with open(sol_filepath, "r") as file:
            assert file.readline().strip() == "AMRA* solution", f"{sol_filepath} does not contain AMRA* solution"

            self.num_successful_plans = int(file.readline().strip().split(":")[1].strip())
            self.latest_plan_itr = int(file.readline().strip().split(":")[1].strip())
            self.num_heuristics = int(file.readline().strip().split(":")[1].strip())
            self.num_resolution_levels = int(file.readline().strip().split(":")[1].strip())
            self.num_expansions = int(file.readline().strip().split(":")[1].strip())
            self.w1_solve = float(file.readline().strip().split(":")[1].strip())
            self.w2_solve = float(file.readline().strip().split(":")[1].strip())
            self.search_time = float(file.readline().strip().split(":")[1].strip())

            solutions = [PlanIterationSolution.load(file) for _ in range(self.num_successful_plans)]
            self.solutions = dict((solution.plan_itr, solution) for solution in solutions)

            assert file.readline().strip() == "opened_states:"
            num_lines = int(file.readline().strip())
            self.opened_states = pd.read_csv(
                StringIO("".join([file.readline() for _ in range(num_lines)])),
                sep=",",
                skipinitialspace=True,
            )

            assert file.readline().strip() == "closed_states:"
            num_lines = int(file.readline().strip())
            self.closed_states = pd.read_csv(
                StringIO("".join([file.readline() for _ in range(num_lines)])),
                sep=",",
                skipinitialspace=True,
            )

            assert file.readline().strip() == "inconsistent_states:"
            num_lines = int(file.readline().strip())
            self.inconsistent_states = pd.read_csv(
                StringIO("".join([file.readline() for _ in range(num_lines)])),
                sep=",",
                skipinitialspace=True,
            )

    def __getitem__(self, plan_itr: int) -> PlanIterationSolution:
        return self.solutions[plan_itr]

    def iterations(self):
        return self.solutions.keys()

    @property
    def latest_solution(self) -> PlanIterationSolution:
        return self[self.latest_plan_itr]


import vedo
import trimesh
import yaml
from erl_common.color_map import COLOR_MAP
import cv2
from matplotlib.colors import ListedColormap
import imageio


class VisualizationFor3DSceneGraph:
    class SOC:  # special object categories
        GROUND = 0
        STAIRS_UP = -1
        STAIRS_DOWN = -2
        WALL = -3
        CEILING = -4
        NA = -5

        @staticmethod
        def get(cat_id: int) -> str:
            return [
                "ground",
                "stairs up",
                "stairs down",
                "wall",
                "ceiling",
                "N/A",
            ][abs(cat_id)]

    def __init__(self, sol_filepath: str, sol_name: str, scene_graph_data_dir: str, mesh_filepath: str):
        self.amra_sol = AMRAStarSolution(sol_filepath)
        self.sol_name = sol_name
        self.scene_graph_data_dir = scene_graph_data_dir
        self.mesh_filepath = mesh_filepath
        self.output_dir = os.path.dirname(sol_filepath)
        self.output_image_dir = os.path.join(self.output_dir, "images")
        self.output_video_dir = os.path.join(self.output_dir, "videos")
        os.makedirs(self.output_image_dir, exist_ok=True)
        os.makedirs(self.output_video_dir, exist_ok=True)

        with open(os.path.join(self.scene_graph_data_dir, "building.yaml"), "r") as file:
            self.scene_graph_data = yaml.safe_load(file)
        self.rooms = dict()
        self.objects = dict()
        for floor_id, floor_data in self.scene_graph_data["floors"].items():
            for room_id, room_data in floor_data["rooms"].items():
                self.rooms[room_id] = room_data
                for obj_id, obj_data in room_data["objects"].items():
                    self.objects[obj_id] = obj_data
        self.objects[-5] = dict(name=VisualizationFor3DSceneGraph.SOC.get(-5))
        self.objects[-4] = dict(name=VisualizationFor3DSceneGraph.SOC.get(-4))
        self.objects[-3] = dict(name=VisualizationFor3DSceneGraph.SOC.get(-3))
        self.objects[-2] = dict(name=VisualizationFor3DSceneGraph.SOC.get(-2))
        self.objects[-1] = dict(name=VisualizationFor3DSceneGraph.SOC.get(-1))
        self.objects[0] = dict(name=VisualizationFor3DSceneGraph.SOC.get(0))

        init_pos = self.amra_sol.latest_solution.path.iloc[0].values
        self.robot_mesh: vedo.Mesh = (
            vedo.load(os.path.join(os.path.dirname(__file__), "BB-8.STL"))
            .rotate_x(90)
            .scale(0.001)
            .shift(init_pos[0], init_pos[1], init_pos[2] + 0.14)
            .c("orange")
        )
        print("robot mesh size [x_min, x_max, y_min, y_max, z_min, z_max]: ", self.robot_mesh.GetBounds())

        self.grid_map_x_res = self.scene_graph_data["floors"][0]["grid_map_resolution"][0]
        self.grid_map_y_res = self.scene_graph_data["floors"][0]["grid_map_resolution"][1]
        self.grid_map_z_res = self.floor_height

        self.grid_map_x_min = self.scene_graph_data["floors"][0]["grid_map_origin"][0]
        self.grid_map_y_min = self.scene_graph_data["floors"][0]["grid_map_origin"][1]
        self.grid_map_z_min = self.scene_graph_data["floors"][0]["ground_z"] - 0.5 * self.grid_map_z_res

        self.video_width = 2560
        self.video_height = 1440
        self.camera = dict(
            position=(-12.1457, -20.2928, 5.59588),
            focal_point=(-1.27467, -3.81539, 0.728039),
            viewup=(0.118814, 0.208410, 0.970798),
            distance=20.3317,
            clipping_range=(7.14519, 37.9173),
        )

    @property
    def floor_height(self):
        if self.scene_graph_data["num_floors"] == 1:
            return 2.0
        else:
            floor_height = 1000.0
            for i in range(1, self.scene_graph_data["num_floors"]):
                height = (
                    self.scene_graph_data["floors"][i]["ground_z"] - self.scene_graph_data["floors"][i - 1]["ground_z"]
                )
                if height < floor_height:
                    floor_height = height
            return floor_height

    def metric_to_grid_x(self, x):
        if hasattr(x, "__iter__"):
            if not isinstance(x, np.ndarray):
                x = np.array(x)
            return np.floor((x - self.grid_map_x_min) / self.grid_map_x_res).astype(np.int32)
        return int((x - self.grid_map_x_min) / self.grid_map_x_res)

    def metric_to_grid_y(self, y):
        if hasattr(y, "__iter__"):
            if not isinstance(y, np.ndarray):
                y = np.array(y)
            return np.floor((y - self.grid_map_y_min) / self.grid_map_y_res).astype(np.int32)
        return int((y - self.grid_map_y_min) / self.grid_map_y_res)

    def metric_to_grid_z(self, z):
        if hasattr(z, "__iter__"):
            if not isinstance(z, np.ndarray):
                z = np.array(z)
            return np.floor((z - self.grid_map_z_min) / self.grid_map_z_res).astype(np.int32)
        return int((z - self.grid_map_z_min) / self.grid_map_z_res)

    @property
    def involved_floors(self):
        floors = set()
        for plan_itr in self.amra_sol.iterations():
            path_z = self.metric_to_grid_z(self.amra_sol[plan_itr].path["pos[2]"])
            unique_z = np.unique(path_z)
            floors.update(unique_z)
        return sorted(list(floors))

    def visualize_mesh(self, show_robot: bool = True, width=None, height=None):
        mesh = trimesh.load(self.mesh_filepath)
        vertices = mesh.vertices[:, [0, 2, 1]]
        vertices[:, 1] *= -1
        faces = mesh.faces
        mesh = vedo.Mesh([vertices, faces]).c("gray").alpha(0.3)
        if width is None:
            width = self.video_width
        if height is None:
            height = self.video_height
        plotter = vedo.Plotter(axes=0, interactive=False, size=(width, height))
        if show_robot:
            plotter.show(mesh, self.robot_mesh, interactive=False, camera=self.camera)
        else:
            plotter.show(mesh, interactive=False, camera=self.camera)
        return plotter

    def set_robot_mesh_pose(self, x, y, z, yaw):
        cur_pos = self.robot_mesh.pos()
        self.robot_mesh.SetPosition(-cur_pos[0], -cur_pos[1], -cur_pos[2])
        self.robot_mesh.rotate_z(-self.robot_mesh.GetOrientation()[2]).rotate_z(yaw)
        self.robot_mesh.SetPosition(x, y, z + 0.14)

    def draw_room_map(self, floor_id: str, ax: plt.Axes):
        room_map = cv2.imread(
            os.path.join(self.scene_graph_data_dir, "room_maps", f"{floor_id}.png"),
            cv2.IMREAD_GRAYSCALE,
        )
        room_map = room_map.astype(np.int32)
        room_map += VisualizationFor3DSceneGraph.SOC.NA
        room_ids = np.unique(room_map).flatten()
        room_ids.sort()
        room_categories = {room_id: self.rooms[room_id]["name"] for room_id in room_ids if room_id > 0}
        img = np.zeros_like(room_map, dtype=np.uint8)
        labels = []
        for i, room_id in enumerate(room_ids):
            img[room_map == room_id] = i
            labels.append(room_categories.get(room_id, "N/A"))
        ax_img = ax.imshow(img, cmap=ListedColormap(COLOR_MAP[: len(room_ids)]), alpha=0.5)
        ax.axis("off")
        cbar = ax.figure.colorbar(ax_img, ax=ax)
        cbar.ax.yaxis.set_major_locator(plt.FixedLocator(np.arange(len(room_ids))))
        cbar.ax.set_yticklabels(labels, fontdict=dict(fontsize=5))
        ax.set_title(f"Room Map for Floor {floor_id}", fontdict=dict(fontsize=8))
        return len(room_ids)

    def draw_cat_map(self, floor_id: str, ax: plt.Axes):
        cat_map = cv2.imread(
            os.path.join(self.scene_graph_data_dir, "cat_maps", f"{floor_id}.png"),
            cv2.IMREAD_GRAYSCALE,
        )
        cat_map = cat_map.astype(np.int32)
        cat_map += VisualizationFor3DSceneGraph.SOC.NA
        cat_ids = np.unique(cat_map).flatten()
        cat_ids.sort()
        cat_categories = {cat_id: self.objects[cat_id]["name"] for cat_id in cat_ids}
        img = np.zeros_like(cat_map, dtype=np.uint8)
        label_locs = []
        labels = []
        for i, cat_id in enumerate(cat_ids):
            img[cat_map == cat_id] = i
            label = cat_categories.get(cat_id, "N/A")
            label_locs.append(i)
            labels.append(label)
            # if len(labels) == 0 or label != labels[-1]:
            #     label_locs.append(i)
            #     labels.append(label)
        ax_img = ax.imshow(img, cmap=ListedColormap(COLOR_MAP[: len(cat_ids)]), alpha=0.5)
        ax.axis("off")
        cbar = ax.figure.colorbar(ax_img, ax=ax)
        cbar.ax.yaxis.set_major_locator(plt.FixedLocator(label_locs))
        cbar.ax.set_yticklabels(labels, fontdict=dict(fontsize=5))
        ax.set_title(f"Object Map for Floor {floor_id}", fontdict=dict(fontsize=8))
        return len(cat_ids)

    def visualize_path_animation_2d(self):
        axes = dict(cat_maps=dict(), room_maps=dict())
        fig = plt.figure(figsize=(self.video_width / 300, self.video_height / 300), dpi=300)
        involved_floors = self.involved_floors
        num_involved_floors = len(involved_floors)
        plt_axes = fig.subplots(num_involved_floors, 2)
        n_used_colors = 0
        for i, floor_id in enumerate(self.involved_floors):
            axes["cat_maps"][floor_id] = plt_axes[i][0]
            axes["room_maps"][floor_id] = plt_axes[i][1]
            n_used_colors = max(self.draw_room_map(floor_id, axes["room_maps"][floor_id]), n_used_colors)
            n_used_colors = max(self.draw_cat_map(floor_id, axes["cat_maps"][floor_id]), n_used_colors)

        plt.tight_layout()
        plt.pause(0.1)

        # visualize the path
        path = self.amra_sol.latest_solution.path
        num_waypoints = len(path)
        x = self.metric_to_grid_x(path["pos[0]"])
        y = self.metric_to_grid_y(path["pos[1]"])
        z = self.metric_to_grid_z(path["pos[2]"])

        axes["room_maps"][z[0]].plot(y[0], x[0], c=COLOR_MAP[n_used_colors], marker="o", markersize=5)
        axes["cat_maps"][z[0]].plot(y[0], x[0], c=COLOR_MAP[n_used_colors], marker="o", markersize=5)
        n_used_colors += 1

        axes["room_maps"][z[-1]].plot(y[-1], x[-1], c=COLOR_MAP[n_used_colors], marker="*", markersize=5)
        axes["cat_maps"][z[-1]].plot(y[-1], x[-1], c=COLOR_MAP[n_used_colors], marker="*", markersize=5)
        n_used_colors += 1

        z_changes = np.abs(np.diff(z))  # how many times z changes
        cur_floor = z[0]
        i0 = 0
        cur_line_in_cat_map = axes["cat_maps"][cur_floor].plot(y[i0:1], x[i0:1], c=COLOR_MAP[n_used_colors], lw=1)[0]
        cur_line_in_room_map = axes["room_maps"][cur_floor].plot(y[i0:1], x[i0:1], c=COLOR_MAP[n_used_colors], lw=1)[0]
        n_used_colors += 1
        stride = 1
        video_path = os.path.join(self.output_video_dir, f"path_2d.mp4")
        with imageio.get_writer(video_path, fps=30) as writer:
            for i in range(1, num_waypoints + stride, stride):
                i = min(i, num_waypoints)
                if z_changes[i0 : i - 1].sum() > 0:
                    i0 = i - 1
                    cur_floor = z[i]
                    cur_line_in_cat_map = axes["cat_maps"][cur_floor].plot(
                        y[i0:i], x[i0:i], c=COLOR_MAP[n_used_colors], lw=1
                    )[0]
                    cur_line_in_room_map = axes["room_maps"][cur_floor].plot(
                        y[i0:i], x[i0:i], c=COLOR_MAP[n_used_colors], lw=1
                    )[0]
                    n_used_colors += 1
                else:
                    cur_line_in_cat_map.set_data(y[i0:i], x[i0:i])
                    cur_line_in_room_map.set_data(y[i0:i], x[i0:i])
                fig.canvas.flush_events()
                fig.canvas.draw()
                # get the image as a numpy array
                img = np.frombuffer(fig.canvas.renderer.buffer_rgba(), dtype=np.uint8)
                img = img.reshape(fig.canvas.get_width_height()[::-1] + (4,))
                writer.append_data(img)
        plt.close(fig)

    def visualize_path_animation_3d(self):
        plotter = self.visualize_mesh()
        path = self.amra_sol.latest_solution.path
        x = path["pos[0]"]
        y = path["pos[1]"]
        z = path["pos[2]"]
        num_waypoints = len(path)
        video_path = os.path.join(self.output_video_dir, f"path_3d.mp4")
        with imageio.get_writer(video_path, fps=30) as writer:
            for i in range(num_waypoints):
                if i == 0:
                    self.set_robot_mesh_pose(x[i], y[i], z[i], 0)
                else:
                    if z[i] != z[i - 1]:
                        seg = np.linspace(
                            np.array([x[i - 1], y[i - 1], z[i - 1]]),
                            np.array([x[i], y[i], z[i]]),
                            60,
                        )
                        cur_robot_yaw = self.robot_mesh.GetOrientation()[2]
                        for j in range(1, len(seg)):
                            self.set_robot_mesh_pose(seg[j][0], seg[j][1], seg[j][2], cur_robot_yaw)
                            plotter += vedo.Line(seg[j - 1], seg[j]).lw(4).c("red")
                            plotter.render()
                            # get the image as a numpy array
                            img = plotter.screenshot(asarray=True)
                            writer.append_data(img)
                    else:
                        self.set_robot_mesh_pose(
                            x[i], y[i], z[i], np.rad2deg(np.arctan2(y[i] - y[i - 1], x[i] - x[i - 1]))
                        )
                        plotter += vedo.Line([x[i - 1], y[i - 1], z[i - 1]], [x[i], y[i], z[i]]).lw(4).c("red")
                plotter.render()
                # get the image as a numpy array
                img = plotter.screenshot(asarray=True)
                writer.append_data(img)
        plotter.close()

    def visualize_opened_states(self, duration=100, fps=30):
        vedo.settings.immediate_rendering = False
        opened_states_by_heuristic_id = self.amra_sol.opened_states.groupby("heuristic_id")
        path = self.amra_sol.latest_solution.path
        x = np.array(path["pos[0]"])
        y = np.array(path["pos[1]"])
        z = np.array(path["pos[2]"])
        path = np.array([x, y, z]).T
        num_frames = (duration - 2) * fps
        num_expansions = self.amra_sol.num_expansions
        print(self.amra_sol.num_heuristics, " heuristics")
        for heuristic_id in range(self.amra_sol.num_heuristics):
            opened_states_by_expand_itr = opened_states_by_heuristic_id.get_group(heuristic_id).groupby("expand_itr")
            # if heuristic_id == 0:
            #     num_frames = len(opened_states_by_expand_itr) // 100 + 1
            plotter = self.visualize_mesh(show_robot=False)
            plotter += vedo.Sphere([x[0], y[0], z[0]], r=0.1).c("pink")
            # cnt = 0
            frame_cnt = 0
            # stride = 100 if heuristic_id == 0 else 1
            states = []
            video_path = os.path.join(self.output_video_dir, f"opened_states_{heuristic_id}.mp4")
            with imageio.get_writer(video_path, fps=fps) as writer:
                for expand_itr, opened_states in tqdm(opened_states_by_expand_itr, ncols=80, desc=f"{heuristic_id}"):
                    # tqdm.write(str(expand_itr))
                    opened_states = np.array(
                        [
                            opened_states["pos[0]"],
                            opened_states["pos[1]"],
                            opened_states["pos[2]"],
                        ]
                    ).T
                    states.append(opened_states)
                    if frame_cnt < int(expand_itr / num_expansions * num_frames) + 1:
                        states = np.concatenate(states, axis=0)
                        plotter += vedo.Points(states, r=4, c="blue")
                        plotter.render()
                        # get the image as a numpy array
                        img = plotter.screenshot(asarray=True)
                        while frame_cnt < int(expand_itr / num_expansions * num_frames) + 1:
                            writer.append_data(img)
                            frame_cnt += 1
                        states = []
                    # cnt += 1
                while frame_cnt < num_frames:
                    writer.append_data(img)
                    frame_cnt += 1
                plotter += vedo.Sphere([x[-1], y[-1], z[-1]], r=0.1).c("cyan")
                plotter += vedo.Lines(path[:-1], path[1:]).lw(8).c("red")
                plotter.render()
                # get the image as a numpy array
                img = plotter.screenshot(asarray=True)
                while frame_cnt < num_frames + 2 * fps:  # add 30 more frames to the end
                    writer.append_data(img)
                    frame_cnt += 1
                plotter.close()

    def visualize_closed_states(self, duration=100, fps=30):
        vedo.settings.immediate_rendering = False
        closed_states_by_heuristic_id = self.amra_sol.closed_states.groupby("action_resolution_level")
        path = self.amra_sol.latest_solution.path
        x = np.array(path["pos[0]"])
        y = np.array(path["pos[1]"])
        z = np.array(path["pos[2]"])
        path = np.array([x, y, z]).T
        num_frames = (duration - 2) * fps
        num_expansions = self.amra_sol.num_expansions
        print(self.amra_sol.num_heuristics, " heuristics")
        for heuristic_id in range(self.amra_sol.num_heuristics):
            closed_states_by_expand_itr = closed_states_by_heuristic_id.get_group(heuristic_id).groupby("expand_itr")
            # if heuristic_id == 0:
            #     num_frames = len(closed_states_by_expand_itr) // 100 + 1
            plotter = self.visualize_mesh(show_robot=False)
            plotter += vedo.Sphere([x[0], y[0], z[0]], r=0.1).c("pink")
            # cnt = 0
            frame_cnt = 0
            # stride = 100 if heuristic_id == 0 else 1
            states = []
            video_path = os.path.join(self.output_video_dir, f"closed_states_{heuristic_id}.mp4")
            with imageio.get_writer(video_path, fps=fps) as writer:
                for expand_itr, closed_states in tqdm(closed_states_by_expand_itr, ncols=80, desc=f"{heuristic_id}"):
                    # tqdm.write(str(expand_itr))
                    closed_states = np.array(
                        [
                            closed_states["pos[0]"],
                            closed_states["pos[1]"],
                            closed_states["pos[2]"],
                        ]
                    ).T
                    states.append(closed_states)
                    if frame_cnt < int(expand_itr / num_expansions * num_frames) + 1:
                        states = np.concatenate(states, axis=0)
                        plotter += vedo.Points(states, r=4, c="green")
                        plotter.render()
                        # get the image as a numpy array
                        img = plotter.screenshot(asarray=True)
                        while frame_cnt < int(expand_itr / num_expansions * num_frames) + 1:
                            writer.append_data(img)
                            frame_cnt += 1
                        states = []
                    # cnt += 1
                while frame_cnt < num_frames:
                    writer.append_data(img)
                    frame_cnt += 1
                plotter += vedo.Sphere([x[-1], y[-1], z[-1]], r=0.1).c("cyan")
                plotter += vedo.Lines(path[:-1], path[1:]).lw(8).c("red")
                plotter.render()
                # get the image as a numpy array
                img = plotter.screenshot(asarray=True)
                while frame_cnt < num_frames + 2 * fps:
                    writer.append_data(img)
                    frame_cnt += 1
                plotter.close()

    def visualize_inconsistent_states(self, duration=100, fps=30):
        vedo.settings.immediate_rendering = False
        path = self.amra_sol.latest_solution.path
        x = np.array(path["pos[0]"])
        y = np.array(path["pos[1]"])
        z = np.array(path["pos[2]"])
        path = np.array([x, y, z]).T
        inconsistent_states_by_expand_itr = self.amra_sol.inconsistent_states.groupby("expand_itr")
        num_frames = (duration - 2) * fps
        num_expansions = self.amra_sol.num_expansions
        plotter = self.visualize_mesh(show_robot=False)
        plotter += vedo.Sphere([x[0], y[0], z[0]], r=0.1).c("pink")
        # cnt = 0
        frame_cnt = 0
        # stride = 100 if heuristic_id == 0 else 1
        states = []
        video_path = os.path.join(self.output_video_dir, f"inconsistent_states.mp4")
        with imageio.get_writer(video_path, fps=30) as writer:
            for expand_itr, inconsistent_states in tqdm(
                inconsistent_states_by_expand_itr, ncols=80, desc=f"inconsistent_states"
            ):
                # tqdm.write(str(expand_itr))
                inconsistent_states = np.array(
                    [
                        inconsistent_states["pos[0]"],
                        inconsistent_states["pos[1]"],
                        inconsistent_states["pos[2]"],
                    ]
                ).T
                states.append(inconsistent_states)
                if frame_cnt < int(expand_itr / num_expansions * num_frames) + 1:
                    states = np.concatenate(states, axis=0)
                    plotter += vedo.Points(states, r=4, c="orange")
                    plotter.render()
                    # get the image as a numpy array
                    img = plotter.screenshot(asarray=True)
                    while frame_cnt < int(expand_itr / num_expansions * num_frames) + 1:
                        writer.append_data(img)
                        frame_cnt += 1
                    states = []
                # cnt += 1
            while frame_cnt < num_frames:
                writer.append_data(img)
                frame_cnt += 1
            plotter += vedo.Sphere([x[-1], y[-1], z[-1]], r=0.1).c("cyan")
            plotter += vedo.Lines(path[:-1], path[1:]).lw(8).c("red")
            plotter.render()
            # get the image as a numpy array
            img = plotter.screenshot(asarray=True)
            while frame_cnt < num_frames + 2 * fps:
                writer.append_data(img)
                frame_cnt += 1
            plotter.close()


# TODO: display number of expansions
