import os.path
import unittest

test_dir = os.path.dirname(__file__)


class VisualizeAMRAStar(unittest.TestCase):
    def test_2d_grid_map(self):
        data_dir = os.path.join(test_dir, "../gtest/results/AMRAStar2D_MultiResolutions/Boston_0_1024.map")
        self.assertTrue(os.path.exists(data_dir), "Please run the gtest `test_amra_star_2d` first")
        from erl_search_planning.amra_star.visualize_solution import main

        main([os.path.join(data_dir, "amra.solution")])

    def test_3d_scene_graph_benevolence_mission1(self):
        data_dir = os.path.join(test_dir, "data/amra_scene_graph_benevolence")
        sol_filepath = os.path.join(data_dir, "amra.solution")
        mesh_filepath = os.path.join(data_dir, "mesh_Benevolence.obj")
        self.assertTrue(os.path.exists(sol_filepath), f"{sol_filepath} does not exist")
        self.assertTrue(os.path.exists(mesh_filepath), f"{mesh_filepath} does not exist")
        from erl_search_planning.amra_star.visualize_solution import main

        main(
            [
                os.path.join(data_dir, "amra.solution"),
                "--scene-graph",
                "--scene-graph-data-dir",
                data_dir,
                "--mesh-filepath",
                mesh_filepath,
            ]
        )

    def test_3d_scene_graph_collierville_mission4(self):
        data_dir = os.path.join(test_dir, "data/amra_scene_graph_collierville")
        sol_filepath = os.path.join(data_dir, "amra.solution")
        mesh_filepath = os.path.join(data_dir, "mesh_Collierville.obj")
        self.assertTrue(os.path.exists(sol_filepath), f"{sol_filepath} does not exist")
        self.assertTrue(os.path.exists(mesh_filepath), f"{mesh_filepath} does not exist")
        from erl_search_planning.amra_star.visualize_solution import main

        main(
            [
                os.path.join(data_dir, "amra.solution"),
                "--scene-graph",
                "--scene-graph-data-dir",
                data_dir,
                "--mesh-filepath",
                mesh_filepath,
            ]
        )


if __name__ == "__main__":
    unittest.main()
