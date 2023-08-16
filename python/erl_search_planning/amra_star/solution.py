import glob
import os.path

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from io import StringIO
from dataclasses import dataclass


class AMRAStarSolution:
    @dataclass
    class Solution:
        plan_itr: int
        w1: float
        w2: float
        goal_index: int
        cost: float
        num_waypoints: int
        path: pd.DataFrame
        action_coords: pd.DataFrame

        @staticmethod
        def load(file):
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
            assert (
                file.readline().strip() == "action_coords:"
            ), f"action_coords data is missing for plan iteration {plan_itr}"
            action_coords = pd.read_csv(
                StringIO("".join([file.readline() for _ in range(num_waypoints - 1)])),
                sep=",",
                header=None,
                skipinitialspace=True,
            )
            return AMRAStarSolution.Solution(
                plan_itr=plan_itr,
                w1=w1,
                w2=w2,
                goal_index=goal_index,
                cost=cost,
                num_waypoints=num_waypoints,
                path=path,
                action_coords=action_coords,
            )

    def __init__(self, sol_filepath: str):
        self.sol_filepath = sol_filepath
        self.sol_name = os.path.splitext(os.path.basename(self.sol_filepath))[0]
        self.output_dir = os.path.join(os.path.dirname(self.sol_filepath))
        self.output_image_dir = os.path.join(self.output_dir, "images")
        self.output_video_dir = os.path.join(self.output_dir, "videos")
        os.makedirs(self.output_image_dir, exist_ok=True)
        os.makedirs(self.output_video_dir, exist_ok=True)
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

            solutions = [AMRAStarSolution.Solution.load(file) for _ in range(self.num_successful_plans)]
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

        img_path = sol_filepath.replace(".solution", ".png")
        self.img = plt.imread(img_path)

    def visualize_map(self) -> plt.Figure:
        fig = plt.figure(figsize=(10, 10))
        fontsize = 20
        plt.imshow(self.img, cmap="gray", origin="lower")
        plt.xlabel("y", fontdict={"fontsize": fontsize})
        plt.ylabel("x", fontdict={"fontsize": fontsize})
        plt.xticks(fontsize=fontsize - 2)
        plt.yticks(fontsize=fontsize - 2)
        plt.title(self.sol_name, fontdict={"fontsize": fontsize})
        plt.axis("equal")
        plt.tight_layout()
        return fig

    def visualize_paths(self):
        for plan_itr in self.solutions.keys():
            self.visualize_path(plan_itr, save=True)

    def visualize_path(self, plan_itr: int, fig: plt.Figure = None, save: bool = True) -> plt.Figure:
        if fig is None:
            fig = self.visualize_map()
        solution = self.solutions[plan_itr]
        n = solution.num_waypoints
        plt.plot(solution.path["y"], solution.path["x"], "r-", linewidth=2)
        plt.plot(solution.path["y"][0], solution.path["x"][0], "*", markersize=20, color="yellow", label="start")
        plt.plot(solution.path["y"][n - 1], solution.path["x"][n - 1], "*", markersize=20, color="red", label="goal")
        plt.title(f"{self.sol_name} - plan iteration {plan_itr}", fontdict={"fontsize": 20})
        if save:
            plt.savefig(os.path.join(self.output_dir, f"{self.sol_name}-path-plan_iter_{solution.plan_itr}.png"))
        plt.pause(0.1)
        return fig

    def visualize_opened_states(self) -> plt.Figure:
        fig = None
        opened_states_by_heuristic_id = self.opened_states.groupby(["heuristic_id"])
        for heuristic_id in range(self.num_heuristics - 1, -1, -1):
            fig = self.visualize_map()
            path = self.solutions[self.latest_plan_itr].path
            n = self.solutions[self.latest_plan_itr].num_waypoints
            plt.plot(path["y"][0], path["x"][0], "*", markersize=20, color="yellow", label="start")
            plt.plot(path["y"][n - 1], path["x"][n - 1], "*", markersize=20, color="red", label="goal")
            plt.pause(0.001)
            opened_states_by_expand_itr = opened_states_by_heuristic_id.get_group(heuristic_id).groupby(["expand_itr"])
            stride = max(1, opened_states_by_expand_itr.ngroups // 20)
            cnt = 0
            expand_itr = 0
            states_x = []
            states_y = []
            for expand_itr, opened_states in opened_states_by_expand_itr:
                if cnt % stride == 0 or (cnt == (opened_states_by_expand_itr.ngroups - 1)):
                    expand_itr = int(expand_itr[0])
                    states_x.append(opened_states["x"])
                    states_y.append(opened_states["y"])
                    states_x = np.concatenate(states_x)
                    states_y = np.concatenate(states_y)
                    plot_data = plt.plot(states_y, states_x, "o", markersize=1, color="blue")[0]
                    plt.title(
                        f"opened states, expand_itr: {expand_itr}, heuristic_id: {heuristic_id}",
                        fontdict={"fontsize": 20},
                    )
                    plt.savefig(
                        os.path.join(
                            self.output_image_dir,
                            f"{self.sol_name}-expand_itr_{expand_itr:09d}-heuristic_id_{heuristic_id}.png",
                        )
                    )
                    fig.canvas.blit(plot_data.get_window_extent())
                    fig.canvas.flush_events()
                    states_x = []
                    states_y = []
                else:
                    states_x.append(opened_states["x"])
                    states_y.append(opened_states["y"])
                cnt += 1

            self.visualize_path(self.latest_plan_itr, fig, save=False)
            plt.savefig(
                os.path.join(
                    self.output_image_dir,
                    f"{self.sol_name}-expand_itr_{expand_itr:09d}-heuristic_id_{heuristic_id}.png",
                )
            )
            plt.savefig(os.path.join(self.output_dir, f"{self.sol_name}-opened_states-heuristic_id_{heuristic_id}.png"))

            images = glob.glob(
                os.path.join(self.output_image_dir, f"{self.sol_name}-expand_itr_*-heuristic_id_{heuristic_id}.png")
            )
            if len(images) > 1:
                images = sorted(images)
                gif_path = os.path.join(
                    self.output_video_dir, f"{self.sol_name}-opened_states-heuristic_id_{heuristic_id}.gif"
                )
                with imageio.get_writer(gif_path, mode="I", loop=10000) as writer:
                    for image_path in images:
                        image = imageio.imread(image_path)
                        writer.append_data(image)
        return fig

    def visualize_closed_states(self) -> plt.Figure:
        fig = None
        closed_states_by_resolution_level = self.closed_states.groupby(["action_resolution_level"])
        for resolution_level in range(self.num_resolution_levels, -1, -1):
            fig = self.visualize_map()
            path = self.solutions[self.latest_plan_itr].path
            n = self.solutions[self.latest_plan_itr].num_waypoints
            plt.plot(path["y"][0], path["x"][0], "*", markersize=20, color="yellow", label="start")
            plt.plot(path["y"][n - 1], path["x"][n - 1], "*", markersize=20, color="red", label="goal")
            plt.pause(0.001)
            closed_states = closed_states_by_resolution_level.get_group(resolution_level)
            stride = max(1, closed_states.size // 20)
            cnt = 0
            expand_itr = 0
            states_x = []
            states_y = []
            for _, row in closed_states.iterrows():
                if cnt % stride == 0 or (cnt == (closed_states_by_resolution_level.ngroups - 1)):
                    states_x.append(row["x"])
                    states_y.append(row["y"])
                    plot_data = plt.plot(states_y, states_x, "o", markersize=1, color="green")[0]
                    plt.title(
                        f"closed states, expand_itr: {expand_itr}, resolution_level: {resolution_level}",
                        fontdict={"fontsize": 20},
                    )
                    expand_itr = int(row["expand_itr"])
                    plt.savefig(
                        os.path.join(
                            self.output_image_dir,
                            f"{self.sol_name}-expand_itr_{expand_itr:09d}-resolution_level_{resolution_level}.png",
                        )
                    )
                    fig.canvas.blit(plot_data.get_window_extent())
                    fig.canvas.flush_events()
                    states_x = []
                    states_y = []
                else:
                    states_x.append(row["x"])
                    states_y.append(row["y"])
                cnt += 1

            self.visualize_path(self.latest_plan_itr, fig, save=False)
            plt.savefig(
                os.path.join(
                    self.output_image_dir,
                    f"{self.sol_name}-expand_itr_{expand_itr:09d}-resolution_level_{resolution_level}.png",
                )
            )
            plt.savefig(
                os.path.join(self.output_dir, f"{self.sol_name}-closed_states-resolution_level_{resolution_level}.png")
            )

            images = glob.glob(
                os.path.join(
                    self.output_image_dir, f"{self.sol_name}-expand_itr_*-resolution_level_{resolution_level}.png"
                )
            )
            if len(images) > 1:
                images = sorted(images)
                gif_filepath = os.path.join(
                    self.output_video_dir, f"{self.sol_name}-closed_states-resolution_level_{resolution_level}.gif"
                )
                with imageio.get_writer(gif_filepath, mode="I", loop=10000) as writer:
                    for image_filepath in images:
                        image = imageio.imread(image_filepath)
                        writer.append_data(image)
        return fig

    def visualize_inconsistent_states(self) -> plt.Figure:
        fig = self.visualize_map()
        path = self.solutions[self.latest_plan_itr].path
        n = self.solutions[self.latest_plan_itr].num_waypoints
        plt.plot(path["y"][0], path["x"][0], "*", markersize=20, color="yellow", label="start")
        plt.plot(path["y"][n - 1], path["x"][n - 1], "*", markersize=20, color="red", label="goal")
        plt.pause(0.001)
        inconsistent_states_by_expand_itr = self.inconsistent_states.groupby(["expand_itr"])
        stride = max(1, inconsistent_states_by_expand_itr.ngroups // 20)
        cnt = 0
        expand_itr = 0
        states_x = []
        states_y = []

        for expand_itr, inconsistent_states in inconsistent_states_by_expand_itr:
            if cnt % stride == 0 or (cnt == (inconsistent_states_by_expand_itr.ngroups - 1)):
                expand_itr = int(expand_itr[0])
                states_x.append(inconsistent_states["x"])
                states_y.append(inconsistent_states["y"])
                states_x = np.concatenate(states_x)
                states_y = np.concatenate(states_y)
                plot_data = plt.plot(states_y, states_x, "o", markersize=1, color="red")[0]
                plt.title(f"inconsistent states, expand_itr: {expand_itr}", fontdict={"fontsize": 20})
                plt.savefig(
                    os.path.join(
                        self.output_image_dir,
                        f"{self.sol_name}-expand_itr_{expand_itr:09d}-inconsistent_states.png",
                    )
                )
                fig.canvas.blit(plot_data.get_window_extent())
                fig.canvas.flush_events()
                states_x = []
                states_y = []
            else:
                states_x.append(inconsistent_states["x"])
                states_y.append(inconsistent_states["y"])
            cnt += 1

        plt.savefig(
            os.path.join(
                self.output_image_dir,
                f"{self.sol_name}-expand_itr_{expand_itr:09d}-inconsistent_states.png",
            )
        )
        plt.savefig(os.path.join(self.output_dir, f"{self.sol_name}-inconsistent_states.png"))

        images = glob.glob(os.path.join(self.output_image_dir, f"{self.sol_name}-expand_itr_*-inconsistent_states.png"))
        if len(images) > 1:
            images = sorted(images)
            gif_filepath = os.path.join(self.output_video_dir, f"{self.sol_name}-inconsistent_states.gif")
            with imageio.get_writer(gif_filepath, mode="I", loop=10000) as writer:
                for image_filepath in images:
                    image = imageio.imread(image_filepath)
                    writer.append_data(image)
        return fig
