from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Dict, Optional, Set, Tuple

from environment import PlanningEnvironment
from mapf_types import (
    Constraint,
    ConstraintType,
    GridNode,
    OrientationMode,
    Path,
    TimedState,
    Transition,
)


@dataclass(order=True)
class LowLevelQueueItem:
    """A*オープンリスト用要素。

    Attributes:
        priority: f = g + h。
        g_cost: 始点からの実コスト。
        state: 現在状態。
    """

    priority: int
    g_cost: int
    state: TimedState = field(compare=False)


def heuristic(
    current_node: GridNode,
    current_mode: OrientationMode,
    goal_node: GridNode,
    goal_mode: OrientationMode,
) -> int:
    """単純な下界ヒューリスティックを返す。

    Args:
        current_node: 現在ノード。
        current_mode: 現在姿勢。
        goal_node: 目標ノード。
        goal_mode: 目標姿勢。

    Returns:
        ヒューリスティック値。
    """
    position_lb = abs(current_node.row - goal_node.row) + abs(
        current_node.col - goal_node.col
    )
    mode_diff = abs(current_mode.degree - goal_mode.degree) % 360
    rotation_lb = min(mode_diff, 360 - mode_diff) // 90
    return position_lb + rotation_lb


def transition_cost(
    transition: Transition,
) -> float:
    """遷移コストを返す。"""
    if transition.action_type.value == "wait":
        return 1.0
    if transition.action_type.value == "move":
        return 1.0
    if transition.action_type.value == "rotate":
        return 1.15
    if transition.action_type.value == "move_rotate":
        return 1.05
    raise ValueError(f"Unsupported action type: {transition.action_type}")


def violates_state_constraint(
    robot_id: int,
    state: TimedState,
    constraints: Set[Constraint],
) -> bool:
    """状態禁止制約に違反するか判定する。

    Args:
        robot_id: ロボットID。
        state: 判定対象状態。
        constraints: 制約集合。

    Returns:
        違反する場合True。
    """
    for constraint in constraints:
        if constraint.robot_id != robot_id:
            continue
        if constraint.constraint_type != ConstraintType.FORBID_STATE:
            continue
        if constraint.time != state.time:
            continue
        if constraint.node == state.node and constraint.mode == state.mode:
            return True
    return False


def violates_transition_constraint(
    robot_id: int,
    transition: Transition,
    env: PlanningEnvironment,
    constraints: Set[Constraint],
) -> bool:
    """遷移禁止制約・領域禁止制約に違反するか判定する。

    Args:
        robot_id: ロボットID。
        transition: 判定対象遷移。
        env: 計画環境。
        constraints: 制約集合。

    Returns:
        違反する場合True。
    """
    for constraint in constraints:
        if constraint.robot_id != robot_id:
            continue

        if constraint.constraint_type == ConstraintType.FORBID_TRANSITION:
            if constraint.transition == transition:
                return True

        if constraint.constraint_type == ConstraintType.FORBID_REGION:
            if constraint.time != transition.time:
                continue
            region_ids = env.reserved_regions_for_transition(
                from_node=transition.from_node,
                from_mode=transition.from_mode,
                to_node=transition.to_node,
                to_mode=transition.to_mode,
            )
            if constraint.region_id in region_ids:
                return True

    return False


def reconstruct_path(
    came_from: Dict[TimedState, Tuple[TimedState, Transition]],
    goal_state: TimedState,
    total_cost: int,
) -> Path:
    """親参照から経路を復元する。

    Args:
        came_from: 親参照辞書。
        goal_state: 終端状態。
        total_cost: 総コスト。

    Returns:
        復元された経路。
    """
    states = [goal_state]
    transitions = []

    current = goal_state
    while current in came_from:
        parent, transition = came_from[current]
        states.append(parent)
        transitions.append(transition)
        current = parent

    states.reverse()
    transitions.reverse()
    return Path(states=states, transitions=transitions, cost=total_cost)


def low_level_a_star(
    robot_id: int,
    start_node: GridNode,
    start_mode: OrientationMode,
    goal_node: GridNode,
    goal_mode: OrientationMode,
    env: PlanningEnvironment,
    constraints: Set[Constraint],
    max_time: int,
    debug: bool = False,
) -> Optional[Path]:
    """姿勢付き単一ロボットA*を実行する。

    Args:
        robot_id: 計画対象ロボットID。
        start_node: 始点ノード。
        start_mode: 始点姿勢。
        goal_node: 目標ノード。
        goal_mode: 目標姿勢。
        env: 計画環境。
        constraints: CBS高レベル制約集合。
        max_time: 最大探索時刻。
        debug: デバッグ出力有効化フラグ。

    Returns:
        経路。見つからなければNone。
    """
    start_state = TimedState(
        node=start_node,
        mode=start_mode,
        time=0,
    )

    if debug:
        print(
            "[LL] Start"
            f" robot={robot_id}"
            f" start=({start_node.row},{start_node.col},{start_mode.degree})"
            f" goal=({goal_node.row},{goal_node.col},{goal_mode.degree})"
            f" constraints={len(constraints)}"
            f" max_time={max_time}"
        )

    open_heap: list[LowLevelQueueItem] = []
    heapq.heappush(
        open_heap,
        LowLevelQueueItem(
            priority=heuristic(
                current_node=start_node,
                current_mode=start_mode,
                goal_node=goal_node,
                goal_mode=goal_mode,
            ),
            g_cost=0,
            state=start_state,
        ),
    )

    best_g: Dict[TimedState, int] = {start_state: 0}
    came_from: Dict[TimedState, Tuple[TimedState, Transition]] = {}

    expanded_count = 0
    generated_count = 0
    state_constraint_skip = 0
    transition_constraint_skip = 0
    repeated_skip = 0

    while open_heap:
        item = heapq.heappop(open_heap)
        current = item.state
        g_cost = item.g_cost
        expanded_count += 1

        if debug and expanded_count % 1000 == 0:
            print(
                "[LL] Progress"
                f" robot={robot_id}"
                f" expanded={expanded_count}"
                f" open={len(open_heap)}"
                f" current=({current.node.row},{current.node.col},"
                f"{current.mode.degree},t={current.time})"
                f" g={g_cost}"
            )

        if current.node == goal_node and current.mode == goal_mode:
            if debug:
                print(
                    "[LL] Goal reached"
                    f" robot={robot_id}"
                    f" expanded={expanded_count}"
                    f" generated={generated_count}"
                    f" state_skip={state_constraint_skip}"
                    f" transition_skip={transition_constraint_skip}"
                    f" repeated_skip={repeated_skip}"
                    f" cost={g_cost}"
                )
            return reconstruct_path(
                came_from=came_from,
                goal_state=current,
                total_cost=g_cost,
            )

        if current.time >= max_time:
            continue

        actions = env.generate_actions(current.node, current.mode)
        if debug and expanded_count <= 20:
            print(
                "[LL] Expand"
                f" robot={robot_id}"
                f" state=({current.node.row},{current.node.col},"
                f"{current.mode.degree},t={current.time})"
                f" actions={len(actions)}"
            )

        for action in actions:
            generated_count += 1

            next_state = TimedState(
                node=action.next_node,
                mode=action.next_mode,
                time=current.time + 1,
            )

            transition = Transition(
                from_node=current.node,
                from_mode=current.mode,
                to_node=action.next_node,
                to_mode=action.next_mode,
                time=current.time,
                action_type=action.action_type,
            )

            if violates_state_constraint(
                robot_id=robot_id,
                state=next_state,
                constraints=constraints,
            ):
                state_constraint_skip += 1
                continue

            if violates_transition_constraint(
                robot_id=robot_id,
                transition=transition,
                env=env,
                constraints=constraints,
            ):
                transition_constraint_skip += 1
                continue

            next_g = g_cost + transition_cost(transition)

            if next_state in best_g and best_g[next_state] <= next_g:
                repeated_skip += 1
                continue

            best_g[next_state] = next_g
            came_from[next_state] = (current, transition)

            h_cost = heuristic(
                current_node=next_state.node,
                current_mode=next_state.mode,
                goal_node=goal_node,
                goal_mode=goal_mode,
            )
            heapq.heappush(
                open_heap,
                LowLevelQueueItem(
                    priority=next_g + h_cost,
                    g_cost=next_g,
                    state=next_state,
                ),
            )

    if debug:
        print(
            "[LL] Failed"
            f" robot={robot_id}"
            f" expanded={expanded_count}"
            f" generated={generated_count}"
            f" state_skip={state_constraint_skip}"
            f" transition_skip={transition_constraint_skip}"
            f" repeated_skip={repeated_skip}"
        )

    return None