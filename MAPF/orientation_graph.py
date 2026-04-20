from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

from environment import PlanningEnvironment
from mapf_types import (
    Action,
    ActionType,
    GridNode,
    OrientationMode,
    Path as RobotPath,
    TimedState,
    Transition,
)


@dataclass(frozen=True)
class OrientedState:
    """姿勢付き状態。

    Attributes:
        row: 行インデックス。
        col: 列インデックス。
        mode_deg: 姿勢角度。
    """

    row: int
    col: int
    mode_deg: int


@dataclass(frozen=True)
class OrientedEdge:
    """姿勢付き状態間の遷移辺。

    Attributes:
        src: 始点状態。
        dst: 終点状態。
        action_type: 遷移種別。
        cost: 遷移コスト。
    """

    src: OrientedState
    dst: OrientedState
    action_type: ActionType
    cost: float


@dataclass(frozen=True)
class OrientedProblem:
    """姿勢付きMAPFの単一ケースをグラフ問題として表したもの。

    Attributes:
        graph: 姿勢付き状態グラフ。
        start_state_ids: ロボットID -> 開始状態ID。
        goal_state_ids: ロボットID -> 目標状態ID。
    """

    graph: "OrientationGraph"
    start_state_ids: Dict[int, int]
    goal_state_ids: Dict[int, int]


class OrientationGraph:
    """姿勢付き状態グラフ。

    `PlanningEnvironment` を背後に持ち、
    状態ID変換・近傍生成・経路復元を提供する。
    """

    def __init__(
        self,
        env: PlanningEnvironment,
        move_cost: float = 1.0,
        wait_cost: float = 1.0,
        rotate_cost: float = 1.15,
        move_rotate_cost: float = 1.05,
    ) -> None:
        """初期化する。

        Args:
            env: 計画環境。
            move_cost: MOVE コスト。
            wait_cost: WAIT コスト。
            rotate_cost: ROTATE コスト。
            move_rotate_cost: MOVE_ROTATE コスト。
        """
        self.env = env
        self.move_cost = move_cost
        self.wait_cost = wait_cost
        self.rotate_cost = rotate_cost
        self.move_rotate_cost = move_rotate_cost

        self._mode_to_index: Dict[int, int] = {
            mode.degree: idx for idx, mode in enumerate(env.modes)
        }
        self._index_to_mode: Dict[int, OrientationMode] = {
            idx: mode for idx, mode in enumerate(env.modes)
        }

    @property
    def num_rows(self) -> int:
        """行数を返す。"""
        return self.env.rows

    @property
    def num_cols(self) -> int:
        """列数を返す。"""
        return self.env.cols

    @property
    def num_modes(self) -> int:
        """姿勢モード数を返す。"""
        return len(self.env.modes)

    def state_to_id(
        self,
        state: OrientedState,
    ) -> int:
        """状態を状態IDへ変換する。

        Args:
            state: 姿勢付き状態。

        Returns:
            状態ID。
        """
        mode_idx = self._mode_to_index[state.mode_deg]
        flat_pos = state.row * self.num_cols + state.col
        return flat_pos * self.num_modes + mode_idx

    def id_to_state(
        self,
        state_id: int,
    ) -> OrientedState:
        """状態IDを状態へ変換する。

        Args:
            state_id: 状態ID。

        Returns:
            姿勢付き状態。
        """
        flat_pos, mode_idx = divmod(state_id, self.num_modes)
        row, col = divmod(flat_pos, self.num_cols)
        mode = self._index_to_mode[mode_idx]
        return OrientedState(
            row=row,
            col=col,
            mode_deg=mode.degree,
        )

    def oriented_state_to_internal(
        self,
        state: OrientedState,
    ) -> Tuple[GridNode, OrientationMode]:
        """内部表現へ変換する。

        Args:
            state: 姿勢付き状態。

        Returns:
            (GridNode, OrientationMode)。
        """
        return (
            GridNode(row=state.row, col=state.col),
            OrientationMode(state.mode_deg),
        )

    def internal_to_oriented_state(
        self,
        node: GridNode,
        mode: OrientationMode,
    ) -> OrientedState:
        """内部表現から姿勢付き状態へ変換する。

        Args:
            node: ノード。
            mode: 姿勢。

        Returns:
            姿勢付き状態。
        """
        return OrientedState(
            row=node.row,
            col=node.col,
            mode_deg=mode.degree,
        )

    def transition_cost(
        self,
        action_type: ActionType,
    ) -> float:
        """遷移コストを返す。

        Args:
            action_type: アクション種別。

        Returns:
            遷移コスト。
        """
        if action_type == ActionType.WAIT:
            return self.wait_cost
        if action_type == ActionType.MOVE:
            return self.move_cost
        if action_type == ActionType.ROTATE:
            return self.rotate_cost
        if action_type == ActionType.MOVE_ROTATE:
            return self.move_rotate_cost
        raise ValueError(f"Unsupported action type: {action_type}")

    def neighbors(
        self,
        state_id: int,
    ) -> List[Tuple[int, OrientedEdge]]:
        """近傍状態一覧を返す。

        Args:
            state_id: 状態ID。

        Returns:
            [(next_state_id, edge), ...]
        """
        state = self.id_to_state(state_id)
        node, mode = self.oriented_state_to_internal(state)

        actions = self.env.generate_actions(node=node, mode=mode)
        neighbors: List[Tuple[int, OrientedEdge]] = []

        for action in actions:
            next_state = self.internal_to_oriented_state(
                node=action.next_node,
                mode=action.next_mode,
            )
            next_state_id = self.state_to_id(next_state)
            edge = OrientedEdge(
                src=state,
                dst=next_state,
                action_type=action.action_type,
                cost=self.transition_cost(action.action_type),
            )
            neighbors.append((next_state_id, edge))

        return neighbors

    def edge_from_state_ids(
        self,
        src_state_id: int,
        dst_state_id: int,
    ) -> OrientedEdge:
        """2状態IDから対応する辺を構成する。

        Args:
            src_state_id: 始点状態ID。
            dst_state_id: 終点状態ID。

        Returns:
            遷移辺。

        Raises:
            ValueError: 直接遷移できない場合。
        """
        src = self.id_to_state(src_state_id)
        dst = self.id_to_state(dst_state_id)

        src_node, src_mode = self.oriented_state_to_internal(src)
        actions = self.env.generate_actions(node=src_node, mode=src_mode)

        for action in actions:
            cand = self.internal_to_oriented_state(
                node=action.next_node,
                mode=action.next_mode,
            )
            if cand == dst:
                return OrientedEdge(
                    src=src,
                    dst=dst,
                    action_type=action.action_type,
                    cost=self.transition_cost(action.action_type),
                )

        raise ValueError(
            f"No valid edge from state_id={src_state_id} to {dst_state_id}"
        )

    def is_goal(
        self,
        state_id: int,
        goal_state_id: int,
    ) -> bool:
        """ゴール判定を行う。

        Args:
            state_id: 現在状態ID。
            goal_state_id: ゴール状態ID。

        Returns:
            ゴールならTrue。
        """
        return state_id == goal_state_id

    def path_state_ids_to_robot_path(
        self,
        state_ids: List[int],
    ) -> RobotPath:
        """状態ID列を `RobotPath` へ復元する。

        Args:
            state_ids: 状態ID列。

        Returns:
            `RobotPath`。
        """
        if not state_ids:
            raise ValueError("state_ids must not be empty")

        states: List[TimedState] = []
        transitions: List[Transition] = []

        oriented_states = [self.id_to_state(state_id) for state_id in state_ids]

        for time_idx, state in enumerate(oriented_states):
            states.append(
                TimedState(
                    node=GridNode(row=state.row, col=state.col),
                    mode=OrientationMode(state.mode_deg),
                    time=time_idx,
                )
            )

        total_cost = 0.0
        for time_idx in range(len(oriented_states) - 1):
            src = oriented_states[time_idx]
            dst = oriented_states[time_idx + 1]

            edge = self.edge_from_state_ids(
                src_state_id=state_ids[time_idx],
                dst_state_id=state_ids[time_idx + 1],
            )
            total_cost += edge.cost

            transitions.append(
                Transition(
                    from_node=GridNode(row=src.row, col=src.col),
                    from_mode=OrientationMode(src.mode_deg),
                    to_node=GridNode(row=dst.row, col=dst.col),
                    to_mode=OrientationMode(dst.mode_deg),
                    time=time_idx,
                    action_type=edge.action_type,
                )
            )

        return RobotPath(
            states=states,
            transitions=transitions,
            cost=total_cost,
        )

    def state_ids_to_internal_states(
        self,
        state_ids: List[int],
    ) -> List[Tuple[GridNode, OrientationMode]]:
        """状態ID列を内部表現列へ変換する。

        Args:
            state_ids: 状態ID列。

        Returns:
            [(GridNode, OrientationMode), ...]
        """
        result: List[Tuple[GridNode, OrientationMode]] = []
        for state_id in state_ids:
            oriented_state = self.id_to_state(state_id)
            result.append(self.oriented_state_to_internal(oriented_state))
        return result


def build_oriented_problem(
    env: PlanningEnvironment,
    starts: Dict[int, Tuple[GridNode, OrientationMode]],
    goals: Dict[int, Tuple[GridNode, OrientationMode]],
    move_cost: float = 1.0,
    wait_cost: float = 1.0,
    rotate_cost: float = 1.15,
    move_rotate_cost: float = 1.05,
) -> OrientedProblem:
    """姿勢付き問題を `OrientedProblem` へ変換する。

    Args:
        env: 計画環境。
        starts: ロボットID -> (開始ノード, 開始姿勢)。
        goals: ロボットID -> (目標ノード, 目標姿勢)。
        move_cost: MOVE コスト。
        wait_cost: WAIT コスト。
        rotate_cost: ROTATE コスト。
        move_rotate_cost: MOVE_ROTATE コスト。

    Returns:
        `OrientedProblem`。
    """
    graph = OrientationGraph(
        env=env,
        move_cost=move_cost,
        wait_cost=wait_cost,
        rotate_cost=rotate_cost,
        move_rotate_cost=move_rotate_cost,
    )

    start_state_ids: Dict[int, int] = {}
    goal_state_ids: Dict[int, int] = {}

    for robot_id, (node, mode) in starts.items():
        state = graph.internal_to_oriented_state(node=node, mode=mode)
        start_state_ids[robot_id] = graph.state_to_id(state)

    for robot_id, (node, mode) in goals.items():
        state = graph.internal_to_oriented_state(node=node, mode=mode)
        goal_state_ids[robot_id] = graph.state_to_id(state)

    return OrientedProblem(
        graph=graph,
        start_state_ids=start_state_ids,
        goal_state_ids=goal_state_ids,
    )