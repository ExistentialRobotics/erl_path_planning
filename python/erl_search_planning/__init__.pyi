from typing import Callable
from typing import List
from typing import overload

import numpy as np
import numpy.typing as npt
from erl_common.storage import GridMapUnsigned2D
from erl_env import DdcMotionPrimitive
from erl_env import Environment2D
from erl_env import EnvironmentBase
from erl_env import EnvironmentSe2
from erl_env import Successor

__all__ = [
    "PlanningInterface",
    "Planning2D",
    "PlanningSe2",
    "PlanningGridSe2",
]

class PlanningInterface(EnvironmentBase):
    def __init__(
        self: PlanningInterface,
        metric_goals_coords: npt.NDArray[np.float64],
        metric_goals_tolerance: npt.NDArray[np.float64],
        terminal_costs: npt.NDArray[np.float64],
    ) -> None: ...
    def connect_compute_cost_callback(
        self: PlanningInterface,
        callback: Callable[
            [npt.NDArray[np.float64], npt.NDArray[np.int32], npt.NDArray[np.float64], npt.NDArray[np.int32], float],
            float,
        ],
    ) -> None:
        """
        Args:
            callback: A callback function that takes in the following arguments:
                - current_metric_state: The current state in metric space.
                - current_grid_state: The current state in grid space.
                - next_metric_state: The next state in metric space.
                - next_grid_state: The next state in grid space.
                - transition_cost: The default state transition cost.
        Returns:
            The cost of the action.
        """
        ...
    def connect_compute_heuristic_callback(
        self: PlanningInterface,
        callback: Callable[
            [
                npt.NDArray[np.float64],
                npt.NDArray[np.int32],
                npt.NDArray[np.float64],
                npt.NDArray[np.float64],
                npt.NDArray[np.float64],
                npt.NDArray[np.bool],
            ],
            float,
        ],
    ) -> None:
        """
        Args:
            callback: A callback function that takes in the following arguments:
                - current_metric_state: The current state in metric space.
                - current_grid_state: The current state in grid space.
                - metric_goals: The goal states in metric space.
                - metric_goals_tolerance: The goal tolerance in metric space.
                - terminal_costs: The terminal costs for each goal.
                - goals_reached: A boolean array indicating whether each goal has been reached.
        """
        ...
    def get_successors(self: PlanningInterface, current_grid_state: npt.NDArray[np.int32]) -> List[Successor]: ...
    def compute_heuristic(self: PlanningInterface, current_metric_state: npt.NDArray[np.float64]) -> float: ...
    @property
    def num_goals(self: PlanningInterface) -> int: ...
    def get_terminal_cost(self: PlanningInterface, goal_index: int) -> float: ...
    def is_goal(self: PlanningInterface, metric_state: npt.NDArray[np.float64]) -> int: ...

class Planning2D(PlanningInterface, Environment2D):
    @overload
    def __init__(
        self: Planning2D,
        metric_goals_coords: npt.NDArray[np.float64],
        metric_goals_tolerance: npt.NDArray[np.float64],
        terminal_costs: npt.NDArray[np.float64],
        allow_diagonal: bool,
        step_size: int,
        grid_map: GridMapUnsigned2D,
    ) -> None: ...
    @overload
    def __init__(
        self: Planning2D,
        metric_goals_coords: npt.NDArray[np.float64],
        metric_goals_tolerance: npt.NDArray[np.float64],
        terminal_costs: npt.NDArray[np.float64],
        allow_diagonal: bool,
        step_size: int,
        grid_map: GridMapUnsigned2D,
        inflate_scale: float,
        shape_metric_vertices: npt.NDArray[np.float64],
    ) -> None: ...

class PlanningSe2(PlanningInterface, EnvironmentSe2):
    @overload
    def __init__(
        self: PlanningSe2,
        metric_goals_coords: npt.NDArray[np.float64],
        metric_goals_tolerance: npt.NDArray[np.float64],
        terminal_costs: npt.NDArray[np.float64],
        collision_check_dt: float,
        motion_primitives: List[DdcMotionPrimitive],
        grid_map: GridMapUnsigned2D,
        num_thetas: int,
    ) -> None: ...
    @overload
    def __init__(
        self: PlanningSe2,
        metric_goals_coords: npt.NDArray[np.float64],
        metric_goals_tolerance: npt.NDArray[np.float64],
        terminal_costs: npt.NDArray[np.float64],
        collision_check_dt: float,
        motion_primitives: List[DdcMotionPrimitive],
        grid_map: GridMapUnsigned2D,
        num_thetas: int,
        inflate_scale: float,
        shape_metric_vertices: npt.NDArray[np.float64],
    ) -> None: ...

class PlanningGridSe2(PlanningInterface, EnvironmentGridSe2):
    @overload
    def __init__(
        self: PlanningGridSe2,
        metric_goals_coords: npt.NDArray[np.float64],
        metric_goals_tolerance: npt.NDArray[np.float64],
        terminal_costs: npt.NDArray[np.float64],
        linear_velocity_min: float,
        linear_velocity_max: float,
        linear_velocity_step: float,
        euclidean_square_distance_cost_weight: float,
        angular_velocity_min: float,
        angular_velocity_max: float,
        angular_velocity_step: float,
        angular_square_distance_cost_weight: float,
        duration_step: float,
        duration: float,
        max_step_size: int,
        grid_map: GridMapUnsigned2D,
        num_orientations: int,
    ) -> None: ...
    @overload
    def __init__(
        self: PlanningGridSe2,
        metric_goals_coords: npt.NDArray[np.float64],
        metric_goals_tolerance: npt.NDArray[np.float64],
        terminal_costs: npt.NDArray[np.float64],
        linear_velocity_min: float,
        linear_velocity_max: float,
        linear_velocity_step: float,
        euclidean_square_distance_cost_weight: float,
        angular_velocity_min: float,
        angular_velocity_max: float,
        angular_velocity_step: float,
        angular_square_distance_cost_weight: float,
        duration_step: float,
        duration: float,
        max_step_size: int,
        grid_map: GridMapUnsigned2D,
        num_orientations: int,
        inflate_scale: float,
        shape_metric_vertices: npt.NDArray[np.float64],
    ) -> None: ...
