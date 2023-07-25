from typing import Dict, List

import numpy as np
import numpy.typing as npt

from .. import PlanningInterface

class Output:
    paths: Dict[int, List[npt.NDArray[np.float64]]]
    action_ids: Dict[int, List[int]]
    path_costs: Dict[int, float]
    opened_list: Dict[int, List[npt.NDArray[np.int32]]]
    closed_list: Dict[int, npt.NDArray[np.int32]]
    inconsistent_list: Dict[int, List[npt.NDArray[np.int32]]]

class AStar:
    def __init__(
        self: AStar,
        metric_start_coords: npt.NDArray[np.float64],
        planning_interface: PlanningInterface,
        eps: float = 1.0,
        max_num_reached_goals: int = 1,
        max_num_iterations: int = -1,
        reopen_inconsistent: bool = False,
        rebuild_queue_on_goal_reached: bool = False,
        log: bool = False,
    ) -> None: ...
    def plan(self: AStar) -> Output: ...
