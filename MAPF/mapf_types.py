from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import FrozenSet, Optional, Tuple


FineCell = Tuple[int, int]


class ActionType(Enum):
    """離散遷移の種類。"""

    WAIT = "wait"
    MOVE = "move"
    ROTATE = "rotate"
    MOVE_ROTATE = "move_rotate"


class ConflictType(Enum):
    """CBS高レベルで扱う競合種別。"""

    STATE_OCCUPANCY = "state_occupancy"
    SWEEP_COLLISION = "sweep_collision"
    RESERVED_REGION = "reserved_region"


class ConstraintType(Enum):
    """CBS高レベル制約の種別。"""

    FORBID_STATE = "forbid_state"
    FORBID_TRANSITION = "forbid_transition"
    FORBID_REGION = "forbid_region"


@dataclass(frozen=True)
class GridNode:
    """MAPF用の離散位置ノード。

    Attributes:
        row: 行インデックス。
        col: 列インデックス。
    """

    row: int
    col: int


@dataclass(frozen=True)
class OrientationMode:
    """姿勢モード。

    Attributes:
        degree: 姿勢角度。0, 90, 180, 270 を想定。
    """

    degree: int


@dataclass(frozen=True)
class TimedState:
    """時刻付き状態。

    Attributes:
        node: 離散位置ノード。
        mode: 姿勢モード。
        time: 離散時刻。
    """

    node: GridNode
    mode: OrientationMode
    time: int


@dataclass(frozen=True)
class Action:
    """環境が返す1ステップ遷移候補。

    Attributes:
        action_type: 遷移種別。
        next_node: 遷移後ノード。
        next_mode: 遷移後姿勢。
    """

    action_type: ActionType
    next_node: GridNode
    next_mode: OrientationMode


@dataclass(frozen=True)
class Transition:
    """1ステップ遷移。

    Attributes:
        from_node: 遷移前ノード。
        from_mode: 遷移前姿勢。
        to_node: 遷移後ノード。
        to_mode: 遷移後姿勢。
        time: 遷移開始時刻。
        action_type: 遷移種別。
    """

    from_node: GridNode
    from_mode: OrientationMode
    to_node: GridNode
    to_mode: OrientationMode
    time: int
    action_type: ActionType


@dataclass(frozen=True)
class Primitive:
    """1ステップ遷移プリミティブ。

    Attributes:
        dr: 行方向差分。
        dc: 列方向差分。
        dtheta: 角度差分。0, 90, -90 を想定。
    """

    dr: int
    dc: int
    dtheta: int


@dataclass
class Path:
    """単一ロボットの離散経路。

    Attributes:
        states: 状態列。
        transitions: 遷移列。
        cost: 経路コスト。
    """

    states: list[TimedState]
    transitions: list[Transition]
    cost: float


@dataclass(frozen=True)
class Conflict:
    """CBS高レベルで扱う競合。

    Attributes:
        robot_i: 一方のロボットID。
        robot_j: 他方のロボットID。
        time: 競合時刻。
        conflict_type: 競合種別。
        state_i: 状態占有競合時のロボットi状態。
        state_j: 状態占有競合時のロボットj状態。
        transition_i: 掃引/領域競合時のロボットi遷移。
        transition_j: 掃引/領域競合時のロボットj遷移。
        region_id: 領域競合時の領域ID。
    """

    robot_i: int
    robot_j: int
    time: int
    conflict_type: ConflictType
    state_i: Optional[TimedState] = None
    state_j: Optional[TimedState] = None
    transition_i: Optional[Transition] = None
    transition_j: Optional[Transition] = None
    region_id: Optional[int] = None


@dataclass(frozen=True)
class Constraint:
    """CBS高レベル制約。

    Attributes:
        robot_id: 制約対象ロボットID。
        constraint_type: 制約種別。
        time: 制約が効く時刻。
        node: 状態禁止制約時のノード。
        mode: 状態禁止制約時の姿勢。
        transition: 遷移禁止制約時の対象遷移。
        region_id: 領域禁止制約時の領域ID。
    """

    robot_id: int
    constraint_type: ConstraintType
    time: int
    node: Optional[GridNode] = None
    mode: Optional[OrientationMode] = None
    transition: Optional[Transition] = None
    region_id: Optional[int] = None


@dataclass(frozen=True)
class RobotGeometrySpec:
    """細グリッド上でのロボット形状仕様。

    Attributes:
        fine_scale: MAPF 1セルを何分割するか。
        occupied_cells_at_zero_deg: 0度姿勢での占有セル集合。
    """

    fine_scale: int
    occupied_cells_at_zero_deg: FrozenSet[FineCell]