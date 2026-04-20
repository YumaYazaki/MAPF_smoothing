from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Sequence, Tuple

from lacam_adapter import LaCAMProblem, LaCAMSolution
from lacam_runner import LaCAMBackend, LaCAMRunnerConfig

Config = Tuple[int, ...]


@dataclass(frozen=True)
class LaCAMConstraint:
    """LaCAM の禁止制約。

    Attributes:
        robot_id: 対象ロボットID。
        time_step: 制約が適用される到着時刻。
        forbidden_state_ids: 時刻 `time_step` に入ってはいけない状態ID群。
        forbidden_transitions: 時刻 `time_step - 1 -> time_step` に
            使ってはいけない遷移 `(src_state_id, dst_state_id)` 群。
    """

    robot_id: int
    time_step: int
    forbidden_state_ids: FrozenSet[int] = frozenset()
    forbidden_transitions: FrozenSet[Tuple[int, int]] = frozenset()


@dataclass(frozen=True)
class JointConflict:
    """同時遷移に対する競合情報。

    Attributes:
        conflict_type: "state_overlap" または "transition_overlap"。
        time_step: 競合が発生する到着時刻。
        robot_i: 競合ロボット i。
        robot_j: 競合ロボット j。
        state_i: robot_i の到着状態ID。
        state_j: robot_j の到着状態ID。
        transition_i: robot_i の遷移 `(src, dst)`。
        transition_j: robot_j の遷移 `(src, dst)`。
        notes: 補助メモ。
    """

    conflict_type: str
    time_step: int
    robot_i: int
    robot_j: int
    state_i: Optional[int] = None
    state_j: Optional[int] = None
    transition_i: Optional[Tuple[int, int]] = None
    transition_j: Optional[Tuple[int, int]] = None
    notes: str = ""


@dataclass(order=True)
class LaCAMHighLevelNode:
    """High-level 探索ノード。

    Attributes:
        priority: open list 用優先度。
        cost_so_far: これまでの累積遷移コスト。
        lower_bound: 残りコスト込みの下界。
        config_sequence: root から現在までの configuration 列。
        constraints: このノードに紐づく制約集合。
        notes: 補助メモ。
    """

    priority: float
    cost_so_far: float = field(compare=False)
    lower_bound: float = field(compare=False)
    config_sequence: List[Config] = field(compare=False)
    constraints: Tuple[LaCAMConstraint, ...] = field(compare=False)
    notes: str = field(default="", compare=False)


class OrientationLaCAMBackend(LaCAMBackend):
    """姿勢付き MAPF 向け LaCAM 骨格 backend。

    v1 の方針:
        - 探索中心を `paths_by_agent` ではなく `config_sequence` に置く
        - occupancy / transition envelope を conflict oracle に使う
        - branching は state / transition 禁止制約で行う
        - AAMAS-24 的な高度な engineering は後段に回す

    Notes:
        これは「論文の本質を外さない最初の実装骨格」であり、
        まだ強い solver ではない。
        特に `_branch_when_no_successor()` は粗い fallback であり、
        今後の主な改善対象である。
    """

    def __init__(
        self,
        max_high_level_expansions: int = 10_000,
        max_depth: int = 128,
        max_branch_children: int = 8,
    ) -> None:
        """初期化する。

        Args:
            max_high_level_expansions: High-level 展開回数上限。
            max_depth: `config_sequence` の最大長。
            max_branch_children: successor 不成立時の分岐上限。
        """
        self.max_high_level_expansions = max_high_level_expansions
        self.max_depth = max_depth
        self.max_branch_children = max_branch_children

    def solve(
        self,
        problem: LaCAMProblem,
        config: LaCAMRunnerConfig,
    ) -> LaCAMSolution:
        """LaCAM 骨格で問題を解く。

        Args:
            problem: LaCAM 問題。
            config: Runner 設定。

        Returns:
            LaCAM 解。
        """
        del config  # 現段階では runner 側の timing / timeout を使用する。

        start_config = self._build_start_config(problem)
        goal_config = self._build_goal_config(problem)

        root = self._make_root_node(
            problem=problem,
            start_config=start_config,
            goal_config=goal_config,
        )

        open_heap: List[LaCAMHighLevelNode] = [root]
        closed_best_cost: Dict[
            Tuple[Config, Tuple[LaCAMConstraint, ...]],
            float,
        ] = {}

        expansions = 0

        while open_heap:
            node = heapq.heappop(open_heap)
            expansions += 1

            if expansions > self.max_high_level_expansions:
                return LaCAMSolution(
                    solved=False,
                    comp_time_sec=0.0,
                    paths_by_agent={},
                    raw_status="max_high_level_expansions_exceeded",
                    notes=(
                        "Exceeded max_high_level_expansions="
                        f"{self.max_high_level_expansions}"
                    ),
                )

            curr_config = node.config_sequence[-1]
            closed_key = (curr_config, node.constraints)
            best_seen = closed_best_cost.get(closed_key)

            if best_seen is not None and best_seen <= node.cost_so_far:
                continue
            closed_best_cost[closed_key] = node.cost_so_far

            if self._is_goal_config(curr_config, goal_config):
                paths_by_agent = self._reconstruct_paths_from_config_sequence(
                    node.config_sequence
                )
                return LaCAMSolution(
                    solved=True,
                    comp_time_sec=0.0,
                    paths_by_agent=paths_by_agent,
                    raw_status="solved",
                    notes="orientation_lacam_v1",
                )

            if len(node.config_sequence) >= self.max_depth:
                continue

            next_time_step = len(node.config_sequence)
            next_config = self._low_level_expand_one_step(
                problem=problem,
                curr_config=curr_config,
                goal_config=goal_config,
                constraints=node.constraints,
                next_time_step=next_time_step,
            )

            if next_config is None:
                children = self._branch_when_no_successor(
                    problem=problem,
                    node=node,
                    curr_config=curr_config,
                    next_time_step=next_time_step,
                    goal_config=goal_config,
                )
                for child in children:
                    heapq.heappush(open_heap, child)
                continue

            explicit_conflict = self._detect_first_joint_conflict(
                problem=problem,
                curr_config=curr_config,
                next_config=next_config,
                time_step=next_time_step,
            )
            if explicit_conflict is not None:
                children = self._branch_on_conflict(
                    problem=problem,
                    node=node,
                    conflict=explicit_conflict,
                    goal_config=goal_config,
                )
                for child in children:
                    heapq.heappush(open_heap, child)
                continue

            child = self._extend_node(
                problem=problem,
                node=node,
                next_config=next_config,
                goal_config=goal_config,
            )
            heapq.heappush(open_heap, child)

        return LaCAMSolution(
            solved=False,
            comp_time_sec=0.0,
            paths_by_agent={},
            raw_status="search_failed",
            notes="open list exhausted",
        )

    def _build_start_config(self, problem: LaCAMProblem) -> Config:
        """開始 configuration を返す。

        Args:
            problem: LaCAM 問題。

        Returns:
            開始 configuration。
        """
        robot_ids = sorted(problem.start_state_ids.keys())
        return tuple(problem.start_state_ids[robot_id] for robot_id in robot_ids)

    def _build_goal_config(self, problem: LaCAMProblem) -> Config:
        """目標 configuration を返す。

        Args:
            problem: LaCAM 問題。

        Returns:
            目標 configuration。
        """
        robot_ids = sorted(problem.goal_state_ids.keys())
        return tuple(problem.goal_state_ids[robot_id] for robot_id in robot_ids)

    def _make_root_node(
        self,
        problem: LaCAMProblem,
        start_config: Config,
        goal_config: Config,
    ) -> LaCAMHighLevelNode:
        """Root ノードを作る。

        Args:
            problem: LaCAM 問題。
            start_config: 開始 configuration。
            goal_config: 目標 configuration。

        Returns:
            Root ノード。
        """
        h = self._joint_heuristic(
            problem=problem,
            config=start_config,
            goal_config=goal_config,
        )
        return LaCAMHighLevelNode(
            priority=h,
            cost_so_far=0.0,
            lower_bound=h,
            config_sequence=[start_config],
            constraints=tuple(),
            notes="root",
        )

    def _is_goal_config(
        self,
        config: Config,
        goal_config: Config,
    ) -> bool:
        """Goal configuration 判定を行う。

        Args:
            config: 現在 configuration。
            goal_config: 目標 configuration。

        Returns:
            一致していれば True。
        """
        return config == goal_config

    def _joint_heuristic(
        self,
        problem: LaCAMProblem,
        config: Config,
        goal_config: Config,
    ) -> float:
        """Joint heuristic を返す。

        各 agent の粗いマンハッタン距離 + 姿勢差の総和を用いる。

        Args:
            problem: LaCAM 問題。
            config: 現在 configuration。
            goal_config: 目標 configuration。

        Returns:
            Heuristic 値。
        """
        graph = problem.graph
        total = 0.0

        for state_id, goal_state_id in zip(config, goal_config, strict=True):
            state = graph.id_to_state(state_id)
            goal_state = graph.id_to_state(goal_state_id)

            dist = abs(state.row - goal_state.row) + abs(
                state.col - goal_state.col
            )
            rot = 0.0 if state.mode_deg == goal_state.mode_deg else 1.0
            total += float(dist) + rot

        return total

    def _single_agent_heuristic(
        self,
        problem: LaCAMProblem,
        state_id: int,
        goal_state_id: int,
    ) -> float:
        """単一エージェントの粗い heuristic を返す。

        Args:
            problem: LaCAM 問題。
            state_id: 現在状態ID。
            goal_state_id: 目標状態ID。

        Returns:
            Heuristic 値。
        """
        graph = problem.graph
        state = graph.id_to_state(state_id)
        goal_state = graph.id_to_state(goal_state_id)

        dist = abs(state.row - goal_state.row) + abs(state.col - goal_state.col)
        rot = 0.0 if state.mode_deg == goal_state.mode_deg else 1.0
        return float(dist) + rot

    def _low_level_expand_one_step(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        goal_config: Config,
        constraints: Tuple[LaCAMConstraint, ...],
        next_time_step: int,
    ) -> Optional[Config]:
        """制約付きで次の joint configuration を 1 つ構築する。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            goal_config: 目標 configuration。
            constraints: 現在有効な制約集合。
            next_time_step: 次に到達する時刻。

        Returns:
            構築できた next_config。できなければ None。
        """
        candidate_moves: Dict[int, List[int]] = {}

        for robot_id, curr_state_id in enumerate(curr_config):
            next_candidates = self._enumerate_feasible_single_agent_moves(
                problem=problem,
                robot_id=robot_id,
                curr_state_id=curr_state_id,
                goal_state_id=goal_config[robot_id],
                constraints=constraints,
                next_time_step=next_time_step,
            )
            if not next_candidates:
                return None
            candidate_moves[robot_id] = next_candidates

        return self._construct_joint_next_config(
            problem=problem,
            curr_config=curr_config,
            candidate_moves=candidate_moves,
            next_time_step=next_time_step,
        )

    def _enumerate_feasible_single_agent_moves(
        self,
        problem: LaCAMProblem,
        robot_id: int,
        curr_state_id: int,
        goal_state_id: int,
        constraints: Tuple[LaCAMConstraint, ...],
        next_time_step: int,
    ) -> List[int]:
        """単一エージェントの候補遷移先を列挙して優先順に返す。

        Args:
            problem: LaCAM 問題。
            robot_id: ロボットID。
            curr_state_id: 現在状態ID。
            goal_state_id: 目標状態ID。
            constraints: 制約集合。
            next_time_step: 次に到達する時刻。

        Returns:
            候補状態IDの優先順リスト。
        """
        graph = problem.graph
        candidates: List[Tuple[float, int]] = []

        for next_state_id, edge in graph.neighbors(curr_state_id):
            if self._violates_constraints(
                robot_id=robot_id,
                src_state_id=curr_state_id,
                dst_state_id=next_state_id,
                next_time_step=next_time_step,
                constraints=constraints,
            ):
                continue

            h = self._single_agent_heuristic(
                problem=problem,
                state_id=next_state_id,
                goal_state_id=goal_state_id,
            )

            score = h + 0.1 * edge.cost
            candidates.append((score, next_state_id))

        candidates.sort(key=lambda item: item[0])
        return [state_id for _, state_id in candidates]

    def _violates_constraints(
        self,
        robot_id: int,
        src_state_id: int,
        dst_state_id: int,
        next_time_step: int,
        constraints: Tuple[LaCAMConstraint, ...],
    ) -> bool:
        """禁止制約に違反するか判定する。

        Args:
            robot_id: ロボットID。
            src_state_id: 現在状態ID。
            dst_state_id: 遷移先状態ID。
            next_time_step: 次に到達する時刻。
            constraints: 制約集合。

        Returns:
            制約違反なら True。
        """
        for constraint in constraints:
            if constraint.robot_id != robot_id:
                continue
            if constraint.time_step != next_time_step:
                continue
            if dst_state_id in constraint.forbidden_state_ids:
                return True
            if (src_state_id, dst_state_id) in constraint.forbidden_transitions:
                return True
        return False

    def _construct_joint_next_config(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        candidate_moves: Dict[int, List[int]],
        next_time_step: int,
    ) -> Optional[Config]:
        """各 agent 候補から conflict-free な next_config を作る。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            candidate_moves: ロボットID -> 候補状態ID列。
            next_time_step: 次に到達する時刻。

        Returns:
            conflict-free な next_config。作れなければ None。
        """
        robot_order = self._decide_assignment_order(
            problem=problem,
            curr_config=curr_config,
            candidate_moves=candidate_moves,
        )

        partial_next: Dict[int, int] = {}
        success = self._backtrack_assign(
            problem=problem,
            curr_config=curr_config,
            robot_order=robot_order,
            candidate_moves=candidate_moves,
            index=0,
            partial_next=partial_next,
            next_time_step=next_time_step,
        )
        if not success:
            return None

        return tuple(partial_next[robot_id] for robot_id in range(len(curr_config)))

    def _decide_assignment_order(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        candidate_moves: Dict[int, List[int]],
    ) -> List[int]:
        """Joint assignment の順序を決める。

        v1 では、候補数が少ない順 -> ゴール遠い順で並べる。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            candidate_moves: ロボットID -> 候補状態ID列。

        Returns:
            割当順のロボットID列。
        """
        goal_config = self._build_goal_config(problem)
        robot_ids = list(range(len(curr_config)))

        robot_ids.sort(
            key=lambda robot_id: (
                len(candidate_moves[robot_id]),
                -self._single_agent_heuristic(
                    problem=problem,
                    state_id=curr_config[robot_id],
                    goal_state_id=goal_config[robot_id],
                ),
            )
        )
        return robot_ids

    def _backtrack_assign(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        robot_order: Sequence[int],
        candidate_moves: Dict[int, List[int]],
        index: int,
        partial_next: Dict[int, int],
        next_time_step: int,
    ) -> bool:
        """部分割当を DFS で拡張する。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            robot_order: 割当順。
            candidate_moves: ロボットID -> 候補状態ID列。
            index: 現在の割当インデックス。
            partial_next: 部分的に確定した next_config。
            next_time_step: 次に到達する時刻。

        Returns:
            完成できれば True。
        """
        if index >= len(robot_order):
            return True

        robot_id = robot_order[index]
        curr_state_id = curr_config[robot_id]

        for next_state_id in candidate_moves[robot_id]:
            if self._violates_partial_joint_feasibility(
                problem=problem,
                robot_id=robot_id,
                curr_state_id=curr_state_id,
                next_state_id=next_state_id,
                curr_config=curr_config,
                partial_next=partial_next,
                next_time_step=next_time_step,
            ):
                continue

            partial_next[robot_id] = next_state_id
            if self._backtrack_assign(
                problem=problem,
                curr_config=curr_config,
                robot_order=robot_order,
                candidate_moves=candidate_moves,
                index=index + 1,
                partial_next=partial_next,
                next_time_step=next_time_step,
            ):
                return True

            del partial_next[robot_id]

        return False

    def _violates_partial_joint_feasibility(
        self,
        problem: LaCAMProblem,
        robot_id: int,
        curr_state_id: int,
        next_state_id: int,
        curr_config: Config,
        partial_next: Dict[int, int],
        next_time_step: int,
    ) -> bool:
        """部分割当との競合を判定する。

        Args:
            problem: LaCAM 問題。
            robot_id: 今回割り当てるロボットID。
            curr_state_id: 現在状態ID。
            next_state_id: 遷移先候補状態ID。
            curr_config: 現在 configuration。
            partial_next: 既に確定した部分 next_config。
            next_time_step: 次に到達する時刻。

        Returns:
            競合するなら True。
        """
        for other_robot_id, other_next_state_id in partial_next.items():
            other_curr_state_id = curr_config[other_robot_id]
            conflict = self._detect_pair_conflict(
                problem=problem,
                robot_i=robot_id,
                src_i=curr_state_id,
                dst_i=next_state_id,
                robot_j=other_robot_id,
                src_j=other_curr_state_id,
                dst_j=other_next_state_id,
                time_step=next_time_step,
            )
            if conflict is not None:
                return True
        return False

    def _detect_first_joint_conflict(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        next_config: Config,
        time_step: int,
    ) -> Optional[JointConflict]:
        """Joint transition 全体の最初の conflict を返す。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            next_config: 次 configuration。
            time_step: 到着時刻。

        Returns:
            見つかった競合。なければ None。
        """
        num_agents = len(curr_config)

        for robot_i in range(num_agents):
            for robot_j in range(robot_i + 1, num_agents):
                conflict = self._detect_pair_conflict(
                    problem=problem,
                    robot_i=robot_i,
                    src_i=curr_config[robot_i],
                    dst_i=next_config[robot_i],
                    robot_j=robot_j,
                    src_j=curr_config[robot_j],
                    dst_j=next_config[robot_j],
                    time_step=time_step,
                )
                if conflict is not None:
                    return conflict

        return None

    def _detect_pair_conflict(
        self,
        problem: LaCAMProblem,
        robot_i: int,
        src_i: int,
        dst_i: int,
        robot_j: int,
        src_j: int,
        dst_j: int,
        time_step: int,
    ) -> Optional[JointConflict]:
        """2 ロボット間の競合を返す。

        Args:
            problem: LaCAM 問題。
            robot_i: ロボット i。
            src_i: ロボット i の現在状態ID。
            dst_i: ロボット i の次状態ID。
            robot_j: ロボット j。
            src_j: ロボット j の現在状態ID。
            dst_j: ロボット j の次状態ID。
            time_step: 到着時刻。

        Returns:
            見つかった競合。なければ None。
        """
        if self._state_conflicts(
            problem=problem,
            state_id_i=dst_i,
            state_id_j=dst_j,
        ):
            return JointConflict(
                conflict_type="state_overlap",
                time_step=time_step,
                robot_i=robot_i,
                robot_j=robot_j,
                state_i=dst_i,
                state_j=dst_j,
                transition_i=(src_i, dst_i),
                transition_j=(src_j, dst_j),
                notes="state occupancy overlap",
            )

        if self._transition_conflicts(
            problem=problem,
            src_i=src_i,
            dst_i=dst_i,
            src_j=src_j,
            dst_j=dst_j,
        ):
            return JointConflict(
                conflict_type="transition_overlap",
                time_step=time_step,
                robot_i=robot_i,
                robot_j=robot_j,
                state_i=dst_i,
                state_j=dst_j,
                transition_i=(src_i, dst_i),
                transition_j=(src_j, dst_j),
                notes="transition envelope overlap",
            )

        return None

    def _state_conflicts(
        self,
        problem: LaCAMProblem,
        state_id_i: int,
        state_id_j: int,
    ) -> bool:
        """同時到着状態の occupancy overlap を判定する。

        Args:
            problem: LaCAM 問題。
            state_id_i: 状態ID i。
            state_id_j: 状態ID j。

        Returns:
            overlap があれば True。
        """
        graph = problem.graph

        state_i = graph.id_to_state(state_id_i)
        state_j = graph.id_to_state(state_id_j)

        node_i, mode_i = graph.oriented_state_to_internal(state_i)
        node_j, mode_j = graph.oriented_state_to_internal(state_j)

        occ_i = graph.env.state_occupancy_key(node_i, mode_i)
        occ_j = graph.env.state_occupancy_key(node_j, mode_j)

        return len(occ_i & occ_j) > 0

    def _transition_conflicts(
        self,
        problem: LaCAMProblem,
        src_i: int,
        dst_i: int,
        src_j: int,
        dst_j: int,
    ) -> bool:
        """同時遷移の envelope overlap を判定する。

        Args:
            problem: LaCAM 問題。
            src_i: 遷移 i の始点状態ID。
            dst_i: 遷移 i の終点状態ID。
            src_j: 遷移 j の始点状態ID。
            dst_j: 遷移 j の終点状態ID。

        Returns:
            overlap があれば True。
        """
        graph = problem.graph

        state_src_i = graph.id_to_state(src_i)
        state_dst_i = graph.id_to_state(dst_i)
        state_src_j = graph.id_to_state(src_j)
        state_dst_j = graph.id_to_state(dst_j)

        src_node_i, src_mode_i = graph.oriented_state_to_internal(state_src_i)
        dst_node_i, dst_mode_i = graph.oriented_state_to_internal(state_dst_i)
        src_node_j, src_mode_j = graph.oriented_state_to_internal(state_src_j)
        dst_node_j, dst_mode_j = graph.oriented_state_to_internal(state_dst_j)

        env_i = graph.env.transition_envelope(
            from_node=src_node_i,
            from_mode=src_mode_i,
            to_node=dst_node_i,
            to_mode=dst_mode_i,
        )
        env_j = graph.env.transition_envelope(
            from_node=src_node_j,
            from_mode=src_mode_j,
            to_node=dst_node_j,
            to_mode=dst_mode_j,
        )

        return len(env_i & env_j) > 0

    def _branch_on_conflict(
        self,
        problem: LaCAMProblem,
        node: LaCAMHighLevelNode,
        conflict: JointConflict,
        goal_config: Config,
    ) -> List[LaCAMHighLevelNode]:
        """明示 conflict に対して child node を作る。

        Args:
            problem: LaCAM 問題。
            node: 親ノード。
            conflict: 見つかった競合。
            goal_config: 目標 configuration。

        Returns:
            Child ノード列。
        """
        children: List[LaCAMHighLevelNode] = []

        if conflict.conflict_type == "state_overlap":
            assert conflict.state_i is not None
            assert conflict.state_j is not None

            child_i = LaCAMConstraint(
                robot_id=conflict.robot_i,
                time_step=conflict.time_step,
                forbidden_state_ids=frozenset({conflict.state_i}),
            )
            child_j = LaCAMConstraint(
                robot_id=conflict.robot_j,
                time_step=conflict.time_step,
                forbidden_state_ids=frozenset({conflict.state_j}),
            )

            children.append(
                self._make_child_with_added_constraint(
                    problem=problem,
                    node=node,
                    added_constraint=child_i,
                    goal_config=goal_config,
                    notes="branch_state_i",
                )
            )
            children.append(
                self._make_child_with_added_constraint(
                    problem=problem,
                    node=node,
                    added_constraint=child_j,
                    goal_config=goal_config,
                    notes="branch_state_j",
                )
            )
            return children

        if conflict.conflict_type == "transition_overlap":
            assert conflict.transition_i is not None
            assert conflict.transition_j is not None

            child_i = LaCAMConstraint(
                robot_id=conflict.robot_i,
                time_step=conflict.time_step,
                forbidden_transitions=frozenset({conflict.transition_i}),
            )
            child_j = LaCAMConstraint(
                robot_id=conflict.robot_j,
                time_step=conflict.time_step,
                forbidden_transitions=frozenset({conflict.transition_j}),
            )

            children.append(
                self._make_child_with_added_constraint(
                    problem=problem,
                    node=node,
                    added_constraint=child_i,
                    goal_config=goal_config,
                    notes="branch_transition_i",
                )
            )
            children.append(
                self._make_child_with_added_constraint(
                    problem=problem,
                    node=node,
                    added_constraint=child_j,
                    goal_config=goal_config,
                    notes="branch_transition_j",
                )
            )
            return children

        return children

    def _branch_when_no_successor(
        self,
        problem: LaCAMProblem,
        node: LaCAMHighLevelNode,
        curr_config: Config,
        next_time_step: int,
        goal_config: Config,
    ) -> List[LaCAMHighLevelNode]:
        """Joint successor が作れない場合の粗い fallback 分岐。

        v1 では、各 robot の「最もゴールに近そうな次状態」を禁止する child
        を少数作る。ここは将来、より LaCAM 本来に近い branching /
        queueing に差し替える想定。

        Args:
            problem: LaCAM 問題。
            node: 親ノード。
            curr_config: 現在 configuration。
            next_time_step: 次に到達する時刻。
            goal_config: 目標 configuration。

        Returns:
            Child ノード列。
        """
        graph = problem.graph
        candidates: List[Tuple[float, int, int]] = []

        for robot_id, curr_state_id in enumerate(curr_config):
            best_next_state_id: Optional[int] = None
            best_score = float("inf")

            for next_state_id, edge in graph.neighbors(curr_state_id):
                if self._violates_constraints(
                    robot_id=robot_id,
                    src_state_id=curr_state_id,
                    dst_state_id=next_state_id,
                    next_time_step=next_time_step,
                    constraints=node.constraints,
                ):
                    continue

                score = self._single_agent_heuristic(
                    problem=problem,
                    state_id=next_state_id,
                    goal_state_id=goal_config[robot_id],
                ) + 0.1 * edge.cost

                if score < best_score:
                    best_score = score
                    best_next_state_id = next_state_id

            if best_next_state_id is not None:
                candidates.append((best_score, robot_id, best_next_state_id))

        candidates.sort(key=lambda item: item[0])

        children: List[LaCAMHighLevelNode] = []
        for _, robot_id, next_state_id in candidates[: self.max_branch_children]:
            added_constraint = LaCAMConstraint(
                robot_id=robot_id,
                time_step=next_time_step,
                forbidden_state_ids=frozenset({next_state_id}),
            )
            children.append(
                self._make_child_with_added_constraint(
                    problem=problem,
                    node=node,
                    added_constraint=added_constraint,
                    goal_config=goal_config,
                    notes="branch_no_successor",
                )
            )

        return children

    def _make_child_with_added_constraint(
        self,
        problem: LaCAMProblem,
        node: LaCAMHighLevelNode,
        added_constraint: LaCAMConstraint,
        goal_config: Config,
        notes: str,
    ) -> LaCAMHighLevelNode:
        """Constraint を 1 個追加した child node を作る。

        Args:
            problem: LaCAM 問題。
            node: 親ノード。
            added_constraint: 追加制約。
            goal_config: 目標 configuration。
            notes: メモ。

        Returns:
            Child ノード。
        """
        new_constraints = node.constraints + (added_constraint,)
        curr_config = node.config_sequence[-1]
        h = self._joint_heuristic(
            problem=problem,
            config=curr_config,
            goal_config=goal_config,
        )

        return LaCAMHighLevelNode(
            priority=node.cost_so_far + h,
            cost_so_far=node.cost_so_far,
            lower_bound=node.cost_so_far + h,
            config_sequence=list(node.config_sequence),
            constraints=new_constraints,
            notes=notes,
        )

    def _extend_node(
        self,
        problem: LaCAMProblem,
        node: LaCAMHighLevelNode,
        next_config: Config,
        goal_config: Config,
    ) -> LaCAMHighLevelNode:
        """1 step 進めた child node を作る。

        Args:
            problem: LaCAM 問題。
            node: 親ノード。
            next_config: 次 configuration。
            goal_config: 目標 configuration。

        Returns:
            Child ノード。
        """
        curr_config = node.config_sequence[-1]
        step_cost = self._compute_joint_step_cost(
            problem=problem,
            curr_config=curr_config,
            next_config=next_config,
        )
        next_cost = node.cost_so_far + step_cost
        h = self._joint_heuristic(
            problem=problem,
            config=next_config,
            goal_config=goal_config,
        )

        return LaCAMHighLevelNode(
            priority=next_cost + h,
            cost_so_far=next_cost,
            lower_bound=next_cost + h,
            config_sequence=list(node.config_sequence) + [next_config],
            constraints=node.constraints,
            notes="extend",
        )

    def _compute_joint_step_cost(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        next_config: Config,
    ) -> float:
        """1 step 分の joint cost を返す。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            next_config: 次 configuration。

        Returns:
            1 step 分の総遷移コスト。
        """
        graph = problem.graph
        total = 0.0

        for src_state_id, dst_state_id in zip(curr_config, next_config, strict=True):
            edge = graph.edge_from_state_ids(src_state_id, dst_state_id)
            total += edge.cost

        return total

    def _reconstruct_paths_from_config_sequence(
        self,
        config_sequence: Sequence[Config],
    ) -> Dict[int, List[int]]:
        """Config sequence をロボット別状態列へ復元する。

        Args:
            config_sequence: Configuration 列。

        Returns:
            ロボットID -> 状態ID列。
        """
        if not config_sequence:
            raise ValueError("config_sequence must not be empty")

        num_agents = len(config_sequence[0])
        paths_by_agent: Dict[int, List[int]] = {
            robot_id: [] for robot_id in range(num_agents)
        }

        for config in config_sequence:
            if len(config) != num_agents:
                raise ValueError("Inconsistent config size in config_sequence")
            for robot_id in range(num_agents):
                paths_by_agent[robot_id].append(config[robot_id])

        return paths_by_agent