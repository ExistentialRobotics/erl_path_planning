from typing import Callable
from typing import List
from typing import overload
from typing import Tuple
import numpy as np
import numpy.typing as npt
from erl_env import EnvironmentState
from erl_env import EnvironmentBase
from erl_env import Successor

__all__ = [
    "HeuristicBase",
    "EuclideanDistanceHeuristic2D",
    "ManhattanDistanceHeuristic2D",
    "DictionaryHeuristic",
    "MultiGoalsHeuristic",
    "PlanningInterface",
    "PlanningInterfaceMultiResolutions",
]

class HeuristicBase:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(
        self: HeuristicBase,
        goal: npt.NDArray[np.float64],
        goal_tolerance: npt.NDArray[np.float64],
        terminal_cost: float,
    ) -> None: ...
    def __call__(self: HeuristicBase, env_state: EnvironmentState) -> float: ...

class EuclideanDistanceHeuristic2D(HeuristicBase):
    def __init__(
        self: EuclideanDistanceHeuristic2D,
        goal: npt.NDArray[np.float64],
        goal_tolerance: npt.NDArray[np.float64],
        terminal_cost: float,
    ) -> None: ...

class ManhattanDistanceHeuristic2D(HeuristicBase):
    def __init__(
        self: ManhattanDistanceHeuristic2D,
        goal: npt.NDArray[np.float64],
        goal_tolerance: npt.NDArray[np.float64],
        terminal_cost: float,
    ) -> None: ...

class DictionaryHeuristic(HeuristicBase):
    def __init__(
        self: DictionaryHeuristic,
        csv_path: str,
        state_hashing_func: Callable[[EnvironmentState], int],
        assert_on_missing: bool = True,
    ) -> None: ...

class MultiGoalsHeuristic(HeuristicBase):
    def __init__(self: MultiGoalsHeuristic, goal_heuristics: List[HeuristicBase]) -> None: ...

class PlanningInterface(EnvironmentBase):
    @overload
    def __init__(
        self: PlanningInterface,
        env: EnvironmentBase,
        metric_start_coords: npt.NDArray[np.float64],
        metric_goals_coords: List[npt.NDArray[np.float64]],
        metric_goals_tolerances: List[npt.NDArray[np.float64]],
        terminal_costs: List[float],
        heuristic: HeuristicBase = None,
    ) -> None: ...
    @overload
    def __init__(
        self: PlanningInterface,
        env: EnvironmentBase,
        metric_start_coords: npt.NDArray[np.float64],
        metric_goal_coords: List[npt.NDArray[np.float64]],
        metric_goal_tolerance: List[npt.NDArray[np.float64]],
        terminal_cost: float,
        heuristic: HeuristicBase = None,
    ) -> None: ...
    # def connect_compute_cost_callback(
    #     self: PlanningInterface,
    #     callback: Callable[
    #         [npt.NDArray[np.float64], npt.NDArray[np.int32], npt.NDArray[np.float64], npt.NDArray[np.int32], float],
    #         float,
    #     ],
    # ) -> None:
    #     """
    #     Args:
    #         callback: A callback function that takes in the following arguments:
    #             - current_metric_state: The current state in metric space.
    #             - current_grid_state: The current state in grid space.
    #             - next_metric_state: The next state in metric space.
    #             - next_grid_state: The next state in grid space.
    #             - transition_cost: The default state transition cost.
    #     Returns:
    #         The cost of the action.
    #     """
    #     ...
    # def connect_compute_heuristic_callback(
    #     self: PlanningInterface,
    #     callback: Callable[
    #         [
    #             npt.NDArray[np.float64],
    #             npt.NDArray[np.int32],
    #             npt.NDArray[np.float64],
    #             npt.NDArray[np.float64],
    #             npt.NDArray[np.float64],
    #             npt.NDArray[np.bool],
    #         ],
    #         float,
    #     ],
    # ) -> None:
    #     """
    #     Args:
    #         callback: A callback function that takes in the following arguments:
    #             - current_metric_state: The current state in metric space.
    #             - current_grid_state: The current state in grid space.
    #             - metric_goals: The goal states in metric space.
    #             - metric_goals_tolerance: The goal tolerance in metric space.
    #             - terminal_costs: The terminal costs for each goal.
    #             - goals_reached: A boolean array indicating whether each goal has been reached.
    #     """
    #     ...
    def get_successors(self: PlanningInterface, current_grid_state: npt.NDArray[np.int32]) -> List[Successor]: ...
    def compute_heuristic(self: PlanningInterface, current_metric_state: npt.NDArray[np.float64]) -> float: ...
    @property
    def num_goals(self: PlanningInterface) -> int: ...
    def get_terminal_cost(self: PlanningInterface, goal_index: int) -> float: ...
    def is_goal(self: PlanningInterface, metric_state: npt.NDArray[np.float64]) -> int: ...

class PlanningInterfaceMultiResolutions:
    @overload
    def __init__(
        self: PlanningInterfaceMultiResolutions,
        environments: List[EnvironmentBase],
        heuristics: List[Tuple[HeuristicBase, int]],
        metric_start_coords: npt.NDArray[np.float64],
        metric_goals_coords: List[npt.NDArray[np.float64]],
        metric_goals_tolerances: List[npt.NDArray[np.float64]],
        terminal_costs: List[float],
    ) -> None: ...
    @overload
    def __init__(
        self: PlanningInterfaceMultiResolutions,
        environments: List[EnvironmentBase],
        heuristics: List[Tuple[HeuristicBase, int]],
        metric_start_coords: npt.NDArray[np.float64],
        metric_goal_coords: List[npt.NDArray[np.float64]],
        metric_goal_tolerance: List[npt.NDArray[np.float64]],
    ) -> None: ...
    @property
    def num_heuristics(self: PlanningInterfaceMultiResolutions) -> int: ...
    @property
    def num_resolution_levels(self: PlanningInterfaceMultiResolutions) -> int: ...
    @property
    def start_state(self: PlanningInterfaceMultiResolutions) -> EnvironmentState: ...
    @property
    def num_goals(self: PlanningInterfaceMultiResolutions) -> int: ...
    def get_heuristic(self: PlanningInterfaceMultiResolutions, heuristic_id: int) -> HeuristicBase: ...
    def get_resolution_assignment(self: PlanningInterfaceMultiResolutions, heuristic_id: int) -> int: ...
    def get_in_resolution_level_flag(
        self: PlanningInterfaceMultiResolutions, env_state: EnvironmentState
    ) -> List[bool]: ...
    def get_successors(
        self: PlanningInterfaceMultiResolutions, env_state: EnvironmentState, env_resolution_level: int
    ) -> List[Successor]: ...
    def get_heuristic_values(self: PlanningInterfaceMultiResolutions, env_state: EnvironmentState) -> List[float]: ...
    def is_metric_goal(self: PlanningInterfaceMultiResolutions, env_state: EnvironmentState) -> bool: ...
    def is_virtual_goal(self: PlanningInterfaceMultiResolutions, env_state: EnvironmentState) -> bool: ...
    def reach_goal(self: PlanningInterfaceMultiResolutions, env_state: EnvironmentState) -> int: ...
    def state_hashing(self: PlanningInterfaceMultiResolutions, env_state: EnvironmentState) -> int: ...
    def get_path(
        self: PlanningInterfaceMultiResolutions, env_state: EnvironmentState, action_coords: List[int]
    ) -> List[EnvironmentState]: ...
