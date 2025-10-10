import argparse
import glob
import os.path

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np

from erl_path_planning.amra_star.solution import AMRAStarSolution, VisualizationFor3DSceneGraph


class VisualizationFor2DGridMap:
    def __init__(self, sol_filepath: str, sol_name: str):
        self.amra_sol = AMRAStarSolution(sol_filepath)
        self.sol_name = sol_name
        self.output_dir = os.path.dirname(sol_filepath)
        self.output_image_dir = os.path.join(self.output_dir, "images")
        self.output_video_dir = os.path.join(self.output_dir, "videos")
        os.makedirs(self.output_image_dir, exist_ok=True)
        os.makedirs(self.output_video_dir, exist_ok=True)

        img_path = sol_filepath.replace(".solution", ".png")
        self.img = plt.imread(img_path)
        self.x_min = 0
        self.x_max = self.img.shape[0]
        self.y_min = 0
        self.y_max = self.img.shape[1]

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
        for plan_itr in self.amra_sol.iterations():
            self.visualize_path(plan_itr, save=True)

    def visualize_path(self, plan_itr: int, fig: plt.Figure = None, save: bool = True) -> plt.Figure:
        if fig is None:
            fig = self.visualize_map()
        solution = self.amra_sol[plan_itr]
        n = solution.num_waypoints
        plt.plot(solution.path["pos[1]"], solution.path["pos[0]"], "r-", linewidth=2)
        plt.plot(
            solution.path["pos[1]"][0],
            solution.path["pos[0]"][0],
            "*",
            markersize=20,
            color="yellow",
            label="start",
        )
        plt.plot(
            solution.path["pos[1]"][n - 1],
            solution.path["pos[0]"][n - 1],
            "*",
            markersize=20,
            color="red",
            label="goal",
        )
        plt.title(f"{self.sol_name} - plan iteration {plan_itr}", fontdict={"fontsize": 20})
        if save:
            plt.savefig(os.path.join(self.output_dir, f"{self.sol_name}-path-plan_iter_{solution.plan_itr}.png"))
        plt.pause(0.1)
        return fig

    def visualize_opened_states(self) -> plt.Figure:
        fig = None
        path = self.amra_sol.latest_solution.path
        n = self.amra_sol.latest_solution.num_waypoints
        opened_states_by_heuristic_id = self.amra_sol.opened_states.groupby(["heuristic_id"])
        for heuristic_id in range(self.amra_sol.num_heuristics - 1, -1, -1):
            fig = self.visualize_map()
            plt.plot(path["pos[1]"][0], path["pos[0]"][0], "*", markersize=20, color="yellow", label="start")
            plt.plot(path["pos[1]"][n - 1], path["pos[0]"][n - 1], "*", markersize=20, color="red", label="goal")
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
                    states_x.append(opened_states["pos[0]"])
                    states_y.append(opened_states["pos[1]"])
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
                    states_x.append(opened_states["pos[0]"])
                    states_y.append(opened_states["pos[1]"])
                cnt += 1

            self.visualize_path(self.amra_sol.latest_plan_itr, fig, save=False)
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
        path = self.amra_sol.latest_solution.path
        n = self.amra_sol.latest_solution.num_waypoints
        closed_states_by_resolution_level = self.amra_sol.closed_states.groupby(["action_resolution_level"])
        for resolution_level in range(self.amra_sol.num_resolution_levels - 1, -1, -1):
            fig = self.visualize_map()
            plt.plot(path["pos[1]"][0], path["pos[0]"][0], "*", markersize=20, color="yellow", label="start")
            plt.plot(path["pos[1]"][n - 1], path["pos[0]"][n - 1], "*", markersize=20, color="red", label="goal")
            plt.pause(0.001)
            closed_states = closed_states_by_resolution_level.get_group(resolution_level)
            stride = max(1, closed_states.size // 20)
            cnt = 0
            expand_itr = 0
            states_x = []
            states_y = []
            for _, row in closed_states.iterrows():
                if cnt % stride == 0 or (cnt == (closed_states_by_resolution_level.ngroups - 1)):
                    states_x.append(row["pos[0]"])
                    states_y.append(row["pos[1]"])
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
                    states_x.append(row["pos[0]"])
                    states_y.append(row["pos[1]"])
                cnt += 1

            self.visualize_path(self.amra_sol.latest_plan_itr, fig, save=False)
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
        path = self.amra_sol.latest_solution.path
        n = self.amra_sol.latest_solution.num_waypoints
        plt.plot(path["pos[1]"][0], path["pos[0]"][0], "*", markersize=20, color="yellow", label="start")
        plt.plot(path["pos[1]"][n - 1], path["pos[0]"][n - 1], "*", markersize=20, color="red", label="goal")
        plt.pause(0.001)
        inconsistent_states_by_expand_itr = self.amra_sol.inconsistent_states.groupby(["expand_itr"])
        stride = max(1, inconsistent_states_by_expand_itr.ngroups // 20)
        cnt = 0
        expand_itr = 0
        states_x = []
        states_y = []

        for expand_itr, inconsistent_states in inconsistent_states_by_expand_itr:
            if cnt % stride == 0 or (cnt == (inconsistent_states_by_expand_itr.ngroups - 1)):
                expand_itr = int(expand_itr[0])
                states_x.append(inconsistent_states["pos[0]"])
                states_y.append(inconsistent_states["pos[1]"])
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
                states_x.append(inconsistent_states["pos[0]"])
                states_y.append(inconsistent_states["pos[1]"])
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


def main(args=None):
    parser = argparse.ArgumentParser(description="Visualize AMRA* solution")
    parser.add_argument("filename", type=str, help="filename of AMRA* solution")
    parser.add_argument("--sol-name", type=str, default="AMRA* Solution", help="name of AMRA* solution")
    parser.add_argument("--scene-graph", action="store_true", help="visualize scene graph")
    parser.add_argument("--scene-graph-data-dir", type=str, help="directory containing scene graph data")
    parser.add_argument("--mesh-filepath", type=str, help="filepath of mesh")
    args = parser.parse_args(args)
    if args.scene_graph:
        assert args.scene_graph_data_dir is not None, "--scene-graph-data-dir must be specified"
        assert args.mesh_filepath is not None, "--mesh-filepath must be specified"
        sol = VisualizationFor3DSceneGraph(args.filename, args.sol_name, args.scene_graph_data_dir, args.mesh_filepath)
        sol.visualize_path_animation_2d()
        sol.visualize_path_animation_3d()
        sol.visualize_closed_states()
        sol.visualize_opened_states()
        sol.visualize_inconsistent_states()
    else:
        sol = VisualizationFor2DGridMap(args.filename, args.sol_name)
        sol.visualize_paths()
        sol.visualize_opened_states()
        sol.visualize_closed_states()
        sol.visualize_inconsistent_states()


if __name__ == "__main__":
    main()
