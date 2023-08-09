from typing import Dict
from typing import List

import numpy as np
import numpy.typing as npt
from erl_common.yaml import YamlableBase
from .. import PlanningInterfaceMultiResolutions

class Output:
    goal_index: int
    path: npt.NDArray[np.float64]
    action_coords: List[int]
    cost: float
    w1_solve: float
    w2_solve: float
    search_time: float
    opened_states: Dict[int, Dict[int, List[npt.NDArray[np.float64]]]]
    closed_states: Dict[int, Dict[int, List[npt.NDArray[np.float64]]]]
    inconsistent_states: Dict[int, List[npt.NDArray[np.float64]]]

    def save(self: Output, file_path: str) -> None: ...

class AMRAStar:
    class Setting(YamlableBase):
        time_limit: int
        w1_init: float
        w2_init: float
        w1_final: float
        w2_final: float
        w1_decay_factor: float
        w2_decay_factor: float
        log: bool
    def __init__(
        self: AMRAStar, planning_interface: PlanningInterfaceMultiResolutions, setting: Setting = None
    ) -> None: ...
    def plan(self: AMRAStar) -> Output: ...
