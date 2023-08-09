from typing import Dict
from typing import List

import numpy as np
import numpy.typing as npt
from erl_common.yaml import YamlableBase
from .. import PlanningInterface

class Output:
    paths: npt.NDArray[np.float64]
    action_coords: List[int]
    cost: float
    opened_list: Dict[int, List[npt.NDArray[np.int32]]]
    closed_list: Dict[int, npt.NDArray[np.int32]]
    inconsistent_list: Dict[int, List[npt.NDArray[np.int32]]]

class AStar:
    class Setting(YamlableBase):
        eps: float
        max_num_iterations: int
        log: bool
        reopen_inconsistent: bool
    def __init__(self: AStar, planning_interface: PlanningInterface, setting: Setting = None) -> None: ...
    def plan(self: AStar) -> Output: ...
