from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Dict, Optional, Set, Tuple

from environment import PlanningEnvironment
from low_level_astar import low_level_a_star
from mapf_types import (
    ActionType,
    Conflict,
    ConflictType,
    Constraint,
    ConstraintType,
    GridNode,
    OrientationMode,
    Path,
    TimedState,
    Transition,
)


@dataclass(order=True)
class CBSQueueItem:
    """CBSオープンリスト用要素。"""

    priority: float
    node_id: int = field(compare=False)


@dataclass
class CBSNode:
    """CBS高レベルノード。"""

    constraints: Set[Constraint]
    paths: Dict[int, Path]
    total_cost: float


def get_state_at_time(
    path: Path,
    time: int,
) -> TimedState:
    """指定時刻の状態を返す。"""
    if time < len(path.states):
        return path.states[time]
    return path.states[-1]


def get_effective_transition_at_time(
    path: Path,
    time: int,
) -> Transition:
    """指定時刻の有効遷移を返す。

    終端後は、その場WAIT遷移を返す。
    """
    if time < len(path.transitions):
        return path.transitions[time]

    terminal_state = path.states[-1]
    return Transition(
        from_node=terminal_state.node,
        from_mode=terminal_state.mode,
        to_node=terminal_state.node,
        to_mode=terminal_state.mode,
        time=time,
        action_type=ActionType.WAIT,
    )


def detect_first_conflict(
    paths: Dict[int, Path],
    env: PlanningEnvironment,
) -> Optional[Conflict]:
    """経路集合から最初の競合を検出する。"""
    robot_ids = sorted(paths.keys())
    max_time = max(len(path.states) for path in paths.values())

    for time in range(max_time):
        for idx_i in range(len(robot_ids)):
            for idx_j in range(idx_i + 1, len(robot_ids)):
                robot_i = robot_ids[idx_i]
                robot_j = robot_ids[idx_j]

                state_i = get_state_at_time(paths[robot_i], time)
                state_j = get_state_at_time(paths[robot_j], time)

                occ_i = env.state_occupancy_key(
                    node=state_i.node,
                    mode=state_i.mode,
                )
                occ_j = env.state_occupancy_key(
                    node=state_j.node,
                    mode=state_j.mode,
                )

                if occ_i & occ_j:
                    return Conflict(
                        robot_i=robot_i,
                        robot_j=robot_j,
                        time=time,
                        conflict_type=ConflictType.STATE_OCCUPANCY,
                        state_i=state_i,
                        state_j=state_j,
                    )

                transition_i = get_effective_transition_at_time(
                    paths[robot_i], time
                )
                transition_j = get_effective_transition_at_time(
                    paths[robot_j], time
                )

                env_i = env.transition_envelope(
                    from_node=transition_i.from_node,
                    from_mode=transition_i.from_mode,
                    to_node=transition_i.to_node,
                    to_mode=transition_i.to_mode,
                )
                env_j = env.transition_envelope(
                    from_node=transition_j.from_node,
                    from_mode=transition_j.from_mode,
                    to_node=transition_j.to_node,
                    to_mode=transition_j.to_mode,
                )

                if env_i & env_j:
                    return Conflict(
                        robot_i=robot_i,
                        robot_j=robot_j,
                        time=time,
                        conflict_type=ConflictType.SWEEP_COLLISION,
                        transition_i=transition_i,
                        transition_j=transition_j,
                    )

                region_i = env.reserved_regions_for_transition(
                    from_node=transition_i.from_node,
                    from_mode=transition_i.from_mode,
                    to_node=transition_i.to_node,
                    to_mode=transition_i.to_mode,
                )
                region_j = env.reserved_regions_for_transition(
                    from_node=transition_j.from_node,
                    from_mode=transition_j.from_mode,
                    to_node=transition_j.to_node,
                    to_mode=transition_j.to_mode,
                )

                common_regions = region_i & region_j
                if common_regions:
                    region_id = next(iter(common_regions))
                    return Conflict(
                        robot_i=robot_i,
                        robot_j=robot_j,
                        time=time,
                        conflict_type=ConflictType.RESERVED_REGION,
                        transition_i=transition_i,
                        transition_j=transition_j,
                        region_id=region_id,
                    )

    return None


def is_specific_conflict_present(
    paths: Dict[int, Path],
    conflict: Conflict,
    env: PlanningEnvironment,
) -> bool:
    """指定された競合が経路集合上に依然として存在するか判定する。"""
    robot_i = conflict.robot_i
    robot_j = conflict.robot_j
    time = conflict.time

    if conflict.conflict_type == ConflictType.STATE_OCCUPANCY:
        state_i = get_state_at_time(paths[robot_i], time)
        state_j = get_state_at_time(paths[robot_j], time)

        occ_i = env.state_occupancy_key(
            node=state_i.node,
            mode=state_i.mode,
        )
        occ_j = env.state_occupancy_key(
            node=state_j.node,
            mode=state_j.mode,
        )
        return bool(occ_i & occ_j)

    transition_i = get_effective_transition_at_time(paths[robot_i], time)
    transition_j = get_effective_transition_at_time(paths[robot_j], time)

    if conflict.conflict_type == ConflictType.SWEEP_COLLISION:
        env_i = env.transition_envelope(
            from_node=transition_i.from_node,
            from_mode=transition_i.from_mode,
            to_node=transition_i.to_node,
            to_mode=transition_i.to_mode,
        )
        env_j = env.transition_envelope(
            from_node=transition_j.from_node,
            from_mode=transition_j.from_mode,
            to_node=transition_j.to_node,
            to_mode=transition_j.to_mode,
        )
        return bool(env_i & env_j)

    if conflict.conflict_type == ConflictType.RESERVED_REGION:
        region_i = env.reserved_regions_for_transition(
            from_node=transition_i.from_node,
            from_mode=transition_i.from_mode,
            to_node=transition_i.to_node,
            to_mode=transition_i.to_mode,
        )
        region_j = env.reserved_regions_for_transition(
            from_node=transition_j.from_node,
            from_mode=transition_j.from_mode,
            to_node=transition_j.to_node,
            to_mode=transition_j.to_mode,
        )
        return bool(region_i & region_j)

    raise ValueError(f"Unsupported conflict type: {conflict.conflict_type}")


def split_conflict_into_constraints(
    conflict: Conflict,
) -> list[Constraint]:
    """競合をCBS分岐用制約に変換する。"""
    constraints: list[Constraint] = []

    if conflict.conflict_type == ConflictType.STATE_OCCUPANCY:
        assert conflict.state_i is not None
        assert conflict.state_j is not None

        constraints.append(
            Constraint(
                robot_id=conflict.robot_i,
                constraint_type=ConstraintType.FORBID_STATE,
                time=conflict.time,
                node=conflict.state_i.node,
                mode=conflict.state_i.mode,
            )
        )
        constraints.append(
            Constraint(
                robot_id=conflict.robot_j,
                constraint_type=ConstraintType.FORBID_STATE,
                time=conflict.time,
                node=conflict.state_j.node,
                mode=conflict.state_j.mode,
            )
        )
        return constraints

    if conflict.conflict_type == ConflictType.SWEEP_COLLISION:
        assert conflict.transition_i is not None
        assert conflict.transition_j is not None

        constraints.append(
            Constraint(
                robot_id=conflict.robot_i,
                constraint_type=ConstraintType.FORBID_TRANSITION,
                time=conflict.time,
                transition=conflict.transition_i,
            )
        )
        constraints.append(
            Constraint(
                robot_id=conflict.robot_j,
                constraint_type=ConstraintType.FORBID_TRANSITION,
                time=conflict.time,
                transition=conflict.transition_j,
            )
        )
        return constraints

    if conflict.conflict_type == ConflictType.RESERVED_REGION:
        assert conflict.region_id is not None

        constraints.append(
            Constraint(
                robot_id=conflict.robot_i,
                constraint_type=ConstraintType.FORBID_REGION,
                time=conflict.time,
                region_id=conflict.region_id,
            )
        )
        constraints.append(
            Constraint(
                robot_id=conflict.robot_j,
                constraint_type=ConstraintType.FORBID_REGION,
                time=conflict.time,
                region_id=conflict.region_id,
            )
        )
        return constraints

    raise ValueError(f"Unsupported conflict type: {conflict.conflict_type}")


def compute_total_cost(
    paths: Dict[int, Path],
) -> float:
    """全ロボット経路コスト合計を返す。"""
    return sum(path.cost for path in paths.values())


def build_constraint_key(
    constraints: Set[Constraint],
) -> frozenset[Constraint]:
    """制約集合を高レベル重複判定用のキーへ変換する。"""
    return frozenset(constraints)


def build_path_signature(
    paths: Dict[int, Path],
) -> tuple:
    """経路集合を高レベル重複判定用の署名へ変換する。"""
    robot_signatures = []
    for robot_id in sorted(paths.keys()):
        path = paths[robot_id]
        state_signature = tuple(
            (
                state.node.row,
                state.node.col,
                state.mode.degree,
                state.time,
            )
            for state in path.states
        )
        robot_signatures.append((robot_id, state_signature))
    return tuple(robot_signatures)


def cbs_plan(
    starts: Dict[int, Tuple[GridNode, OrientationMode]],
    goals: Dict[int, Tuple[GridNode, OrientationMode]],
    env: PlanningEnvironment,
    max_time: int,
    debug: bool = False,
) -> Optional[Dict[int, Path]]:
    """姿勢付きCBSで複数ロボット経路を計画する。"""
    root_constraints: Set[Constraint] = set()
    root_paths: Dict[int, Path] = {}

    if debug:
        print("[CBS] Build root node")

    for robot_id in sorted(starts.keys()):
        start_node, start_mode = starts[robot_id]
        goal_node, goal_mode = goals[robot_id]

        path = low_level_a_star(
            robot_id=robot_id,
            start_node=start_node,
            start_mode=start_mode,
            goal_node=goal_node,
            goal_mode=goal_mode,
            env=env,
            constraints=root_constraints,
            max_time=max_time,
            debug=debug,
        )
        if path is None:
            if debug:
                print(f"[CBS] Root planning failed for robot={robot_id}")
            return None

        root_paths[robot_id] = path

    root = CBSNode(
        constraints=root_constraints,
        paths=root_paths,
        total_cost=compute_total_cost(root_paths),
    )

    if debug:
        print(
            "[CBS] Root ready"
            f" total_cost={root.total_cost}"
            f" num_paths={len(root.paths)}"
        )

    node_store: Dict[int, CBSNode] = {0: root}
    open_heap: list[CBSQueueItem] = [
        CBSQueueItem(priority=root.total_cost, node_id=0)
    ]
    next_node_id = 1

    visited_constraint_sets: set[frozenset[Constraint]] = set()
    visited_constraint_sets.add(build_constraint_key(root_constraints))

    visited_path_signatures: set[tuple] = set()
    visited_path_signatures.add(build_path_signature(root_paths))

    expanded_high_level = 0
    duplicate_high_level_skip = 0
    duplicate_branch_skip = 0
    duplicate_path_skip = 0
    bypass_count = 0

    while open_heap:
        queue_item = heapq.heappop(open_heap)
        current_node = node_store[queue_item.node_id]
        expanded_high_level += 1

        if debug:
            print(
                "[CBS] Expand"
                f" hl_node={queue_item.node_id}"
                f" expanded={expanded_high_level}"
                f" open={len(open_heap)}"
                f" total_cost={current_node.total_cost}"
                f" constraints={len(current_node.constraints)}"
                f" dup_child_skip={duplicate_high_level_skip}"
                f" dup_branch_skip={duplicate_branch_skip}"
                f" dup_path_skip={duplicate_path_skip}"
                f" bypass={bypass_count}"
            )

        conflict = detect_first_conflict(
            paths=current_node.paths,
            env=env,
        )
        if conflict is None:
            if debug:
                print(
                    "[CBS] Solution found"
                    f" hl_expanded={expanded_high_level}"
                    f" total_cost={current_node.total_cost}"
                    f" dup_child_skip={duplicate_high_level_skip}"
                    f" dup_branch_skip={duplicate_branch_skip}"
                    f" dup_path_skip={duplicate_path_skip}"
                    f" bypass={bypass_count}"
                )
            return current_node.paths

        if debug:
            print(
                "[CBS] Conflict"
                f" type={conflict.conflict_type.value}"
                f" time={conflict.time}"
                f" robots=({conflict.robot_i},{conflict.robot_j})"
            )

        branch_constraints = split_conflict_into_constraints(conflict)

        bypass_child: Optional[CBSNode] = None
        bypass_constraint_key: Optional[frozenset[Constraint]] = None
        bypass_path_signature: Optional[tuple] = None

        for new_constraint in branch_constraints:
            if debug:
                print(
                    "[CBS] Branch"
                    f" robot={new_constraint.robot_id}"
                    f" type={new_constraint.constraint_type.value}"
                    f" time={new_constraint.time}"
                )

            if new_constraint in current_node.constraints:
                duplicate_branch_skip += 1
                if debug:
                    print(
                        "[CBS] Branch skipped"
                        " reason=constraint already exists"
                    )
                continue

            child_constraints = set(current_node.constraints)
            child_constraints.add(new_constraint)

            constraint_key = build_constraint_key(child_constraints)
            if constraint_key in visited_constraint_sets:
                duplicate_high_level_skip += 1
                if debug:
                    print(
                        "[CBS] Child skipped"
                        " reason=duplicate constraint set"
                    )
                continue

            child_paths = dict(current_node.paths)
            replanned_robot = new_constraint.robot_id

            start_node, start_mode = starts[replanned_robot]
            goal_node, goal_mode = goals[replanned_robot]

            new_path = low_level_a_star(
                robot_id=replanned_robot,
                start_node=start_node,
                start_mode=start_mode,
                goal_node=goal_node,
                goal_mode=goal_mode,
                env=env,
                constraints=child_constraints,
                max_time=max_time,
                debug=debug,
            )
            if new_path is None:
                if debug:
                    print(
                        "[CBS] Branch pruned"
                        f" robot={replanned_robot}"
                        " reason=no feasible low-level path"
                    )
                continue

            child_paths[replanned_robot] = new_path

            path_signature = build_path_signature(child_paths)
            if path_signature in visited_path_signatures:
                duplicate_path_skip += 1
                if debug:
                    print(
                        "[CBS] Child skipped"
                        " reason=duplicate path signature"
                    )
                continue

            child_total_cost = compute_total_cost(child_paths)

            if not is_specific_conflict_present(child_paths, conflict, env):
                if child_total_cost <= current_node.total_cost:
                    bypass_child = CBSNode(
                        constraints=child_constraints,
                        paths=child_paths,
                        total_cost=child_total_cost,
                    )
                    bypass_constraint_key = constraint_key
                    bypass_path_signature = path_signature

                    if debug:
                        print(
                            "[CBS] Bypass candidate accepted"
                            f" total_cost={child_total_cost}"
                            f" constraints={len(child_constraints)}"
                        )
                    break

            child = CBSNode(
                constraints=child_constraints,
                paths=child_paths,
                total_cost=child_total_cost,
            )

            if detect_first_conflict(child.paths, env) is None:
                if debug:
                    print(
                        "[CBS] Solution found in child"
                        f" total_cost={child.total_cost}"
                    )
                return child.paths

            node_store[next_node_id] = child
            heapq.heappush(
                open_heap,
                CBSQueueItem(
                    priority=child.total_cost,
                    node_id=next_node_id,
                ),
            )

            visited_constraint_sets.add(constraint_key)
            visited_path_signatures.add(path_signature)

            if debug:
                print(
                    "[CBS] Child inserted"
                    f" hl_node={next_node_id}"
                    f" total_cost={child.total_cost}"
                    f" constraints={len(child.constraints)}"
                )

            next_node_id += 1

        if bypass_child is not None:
            node_store[next_node_id] = bypass_child
            heapq.heappush(
                open_heap,
                CBSQueueItem(
                    priority=bypass_child.total_cost,
                    node_id=next_node_id,
                ),
            )

            assert bypass_constraint_key is not None
            assert bypass_path_signature is not None
            visited_constraint_sets.add(bypass_constraint_key)
            visited_path_signatures.add(bypass_path_signature)

            if debug:
                print(
                    "[CBS] Bypass inserted"
                    f" hl_node={next_node_id}"
                    f" total_cost={bypass_child.total_cost}"
                    f" constraints={len(bypass_child.constraints)}"
                )

            bypass_count += 1
            next_node_id += 1

    if debug:
        print(
            "[CBS] Failed: open list exhausted"
            f" hl_expanded={expanded_high_level}"
            f" dup_child_skip={duplicate_high_level_skip}"
            f" dup_branch_skip={duplicate_branch_skip}"
            f" dup_path_skip={duplicate_path_skip}"
            f" bypass={bypass_count}"
        )

    return None