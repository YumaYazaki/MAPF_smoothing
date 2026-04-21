from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Sequence, Set, Tuple

from lacam_adapter import LaCAMProblem, LaCAMSolution
from lacam_runner import LaCAMBackend, LaCAMRunnerConfig
from mapf_types import ActionType

Config = Tuple[int, ...]


@dataclass(frozen=True)
class LaCAMConstraint:
    """High-level の禁止制約。

    Attributes:
        robot_id: 対象ロボットID。
        time_step: 制約が適用される到着時刻。
        forbidden_state_ids: 到着時刻 `time_step` に入ってはいけない状態ID群。
        forbidden_transitions: 遷移 `time_step - 1 -> time_step` で
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
        conflict_type: "state_overlap" または "transition_overlap" など。
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


@dataclass(frozen=True)
class CandidateMove:
    """単一ロボットの次状態候補。

    Attributes:
        next_state_id: 次状態ID。
        score: ranking 用スコア。小さいほど良い。
        edge_cost: 遷移コスト。
        action_type: 遷移種別。
    """

    next_state_id: int
    score: float
    edge_cost: float
    action_type: ActionType


@dataclass(frozen=True)
class JointSuccessorCandidate:
    """joint successor 候補。

    Attributes:
        next_config: 次 configuration。
        score: ranking 用スコア。小さいほど良い。
    """

    next_config: Config
    score: float


@dataclass(frozen=True)
class LowLevelExpandResult:
    """low-level 展開結果。

    Attributes:
        successors: feasible な joint successor 候補列。
        blocking_conflict: successor を作れなかった場合の blocking conflict。
        notes: 補助メモ。
    """

    successors: Tuple[JointSuccessorCandidate, ...]
    blocking_conflict: Optional[JointConflict]
    notes: str = ""


@dataclass(order=True)
class LaCAMHighLevelNode:
    """High-level 探索ノード。

    Attributes:
        priority: open list 用優先度。
        cost_so_far: これまでの累積遷移コスト。
        lower_bound: 残りコスト込みの下界。
        config_sequence: root から現在までの configuration 列。
        constraints: 現ノードに紐づく制約集合。
        notes: 補助メモ。
    """

    priority: float
    cost_so_far: float = field(compare=False)
    lower_bound: float = field(compare=False)
    config_sequence: List[Config] = field(compare=False)
    constraints: Tuple[LaCAMConstraint, ...] = field(compare=False)
    notes: str = field(default="", compare=False)


@dataclass(order=True)
class SingleAgentSearchNode:
    """単一エージェント A* 用ノード。"""

    priority: float
    g_cost: float
    state_id: int = field(compare=False)


@dataclass(frozen=True)
class PartialConstraintNode:
    """low-level の部分制約ノード。

    Attributes:
        assignments: `(robot_id, next_state_id)` の部分固定。
        depth: 何体固定済みか。
        score: low-level queue 用スコア。
    """

    assignments: Tuple[Tuple[int, int], ...]
    depth: int
    score: float


@dataclass(order=True)
class LowLevelQueueEntry:
    """low-level 探索用キュー要素。"""

    priority: float
    node: PartialConstraintNode = field(compare=False)


@dataclass
class PIBTContext:
    """1ステップ PIBT 構成用コンテキスト。

    Attributes:
        curr_config: 現在 configuration。
        next_time_step: 次に到達する時刻。
        fixed_next: 確定済み next state。
        occupied_next: `next_state_id -> robot_id` の予約表。
        priority_order: ロボット処理順。
        active_stack: 再帰中ロボット集合。
        candidate_moves: ロボットID -> 候補遷移列。
        partial_fixed_robots: 部分制約で固定されたロボット集合。
    """

    curr_config: Config
    next_time_step: int
    fixed_next: Dict[int, int]
    occupied_next: Dict[int, int]
    priority_order: Tuple[int, ...]
    active_stack: Set[int]
    candidate_moves: Dict[int, List[CandidateMove]]
    partial_fixed_robots: Set[int]


@dataclass(frozen=True)
class PIBTResult:
    """PIBT 実行結果。

    Attributes:
        success: 成功したかどうか。
        next_config: 成功時の next configuration。
        blocking_conflict: 失敗時の代表 conflict。
        notes: 補助メモ。
    """

    success: bool
    next_config: Optional[Config]
    blocking_conflict: Optional[JointConflict]
    notes: str = ""


class OrientationLaCAMBackendV5(LaCAMBackend):
    """姿勢付き MAPF 向け LaCAM 骨格 backend の v5。

    v5 の主眼:
        - high-level は既存 LaCAM 骨格を維持
        - low-level を `PartialConstraintNode + PIBT` に置換
        - pair / group の知見は priority order 候補生成に使う
    """

    def __init__(
        self,
        max_high_level_expansions: int = 30_000,
        max_depth: int = 160,
        max_branch_children: int = 10,
        max_single_agent_candidates: int = 6,
        max_joint_successors: int = 8,
        max_low_level_expansions_per_order: int = 64,
        max_partial_constraints_per_expand: int = 3,
        max_priority_orders: int = 6,
    ) -> None:
        """初期化する。

        Args:
            max_high_level_expansions: High-level 展開回数上限。
            max_depth: `config_sequence` の最大長。
            max_branch_children: branch で作る child 上限。
            max_single_agent_candidates: 各ロボットの候補数上限。
            max_joint_successors: 各ノードから返す successor 上限。
            max_low_level_expansions_per_order: 1 priority order あたりの
                low-level 部分制約展開上限。
            max_partial_constraints_per_expand:
                1 回の部分制約展開で作る child 数上限。
            max_priority_orders: 試す priority order 候補数上限。
        """
        self.max_high_level_expansions = max_high_level_expansions
        self.max_depth = max_depth
        self.max_branch_children = max_branch_children
        self.max_single_agent_candidates = max_single_agent_candidates
        self.max_joint_successors = max_joint_successors
        self.max_low_level_expansions_per_order = max_low_level_expansions_per_order
        self.max_partial_constraints_per_expand = (
            max_partial_constraints_per_expand
        )
        self.max_priority_orders = max_priority_orders

        # ranking 用重み
        self.wait_penalty = 1.5
        self.backward_penalty = 1.0
        self.no_progress_penalty = 0.75
        self.goal_leave_penalty = 4.0
        self.goal_wait_bonus = -2.0
        self.move_rotate_bonus = -0.2
        self.rotate_progress_bonus = -0.1

    def solve(
        self,
        problem: LaCAMProblem,
        config: LaCAMRunnerConfig,
    ) -> LaCAMSolution:
        """LaCAM 骨格 v5 で問題を解く。

        Args:
            problem: LaCAM 問題。
            config: runner 設定。

        Returns:
            LaCAM 解。
        """
        del config

        if problem.num_agents == 1:
            return self._solve_single_agent_fast_path(problem)

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
                return LaCAMSolution(
                    solved=True,
                    comp_time_sec=0.0,
                    paths_by_agent=self._reconstruct_paths_from_config_sequence(
                        node.config_sequence
                    ),
                    raw_status="solved",
                    notes="orientation_lacam_v5",
                )

            if len(node.config_sequence) >= self.max_depth:
                continue

            next_time_step = len(node.config_sequence)
            prev_config = (
                node.config_sequence[-2]
                if len(node.config_sequence) >= 2
                else None
            )

            expand_result = self._low_level_expand_with_pibt(
                problem=problem,
                curr_config=curr_config,
                goal_config=goal_config,
                constraints=node.constraints,
                next_time_step=next_time_step,
                prev_config=prev_config,
            )

            if expand_result.successors:
                for successor in expand_result.successors:
                    child = self._extend_node(
                        problem=problem,
                        node=node,
                        next_config=successor.next_config,
                        goal_config=goal_config,
                        successor_score_bias=successor.score,
                    )
                    heapq.heappush(open_heap, child)
                continue

            if expand_result.blocking_conflict is not None:
                children = self._branch_on_conflict(
                    problem=problem,
                    node=node,
                    conflict=expand_result.blocking_conflict,
                    goal_config=goal_config,
                )
                for child in children:
                    heapq.heappush(open_heap, child)
                continue

            children = self._branch_when_no_successor(
                problem=problem,
                node=node,
                curr_config=curr_config,
                next_time_step=next_time_step,
                goal_config=goal_config,
            )
            for child in children:
                heapq.heappush(open_heap, child)

        return LaCAMSolution(
            solved=False,
            comp_time_sec=0.0,
            paths_by_agent={},
            raw_status="search_failed",
            notes="open list exhausted",
        )

    # ------------------------------------------------------------------
    # Single-agent fast path
    # ------------------------------------------------------------------

    def _solve_single_agent_fast_path(
        self,
        problem: LaCAMProblem,
    ) -> LaCAMSolution:
        """単一エージェント問題を A* で直接解く。

        Args:
            problem: LaCAM 問題。

        Returns:
            LaCAM 解。
        """
        robot_id = sorted(problem.start_state_ids.keys())[0]
        start_state_id = problem.start_state_ids[robot_id]
        goal_state_id = problem.goal_state_ids[robot_id]

        state_ids = self._single_agent_astar(
            problem=problem,
            start_state_id=start_state_id,
            goal_state_id=goal_state_id,
        )
        if state_ids is None:
            return LaCAMSolution(
                solved=False,
                comp_time_sec=0.0,
                paths_by_agent={},
                raw_status="single_agent_astar_failed",
                notes="No path for single agent",
            )

        return LaCAMSolution(
            solved=True,
            comp_time_sec=0.0,
            paths_by_agent={robot_id: state_ids},
            raw_status="solved",
            notes="orientation_lacam_v5_single_agent_fast_path",
        )

    def _single_agent_astar(
        self,
        problem: LaCAMProblem,
        start_state_id: int,
        goal_state_id: int,
    ) -> Optional[List[int]]:
        """単一エージェント A* を行う。

        Args:
            problem: LaCAM 問題。
            start_state_id: 開始状態ID。
            goal_state_id: 目標状態ID。

        Returns:
            状態ID列。見つからなければ None。
        """
        graph = problem.graph

        open_heap: List[SingleAgentSearchNode] = []
        g_costs: Dict[int, float] = {start_state_id: 0.0}
        parents: Dict[int, Optional[int]] = {start_state_id: None}

        start_h = self._single_agent_heuristic(
            problem=problem,
            state_id=start_state_id,
            goal_state_id=goal_state_id,
        )
        heapq.heappush(
            open_heap,
            SingleAgentSearchNode(
                priority=start_h,
                g_cost=0.0,
                state_id=start_state_id,
            ),
        )

        while open_heap:
            current = heapq.heappop(open_heap)
            if current.state_id == goal_state_id:
                return self._reconstruct_single_agent_path(
                    goal_state_id=goal_state_id,
                    parents=parents,
                )

            if current.g_cost > g_costs.get(current.state_id, float("inf")):
                continue

            for next_state_id, edge in graph.neighbors(current.state_id):
                next_g = current.g_cost + edge.cost
                if next_g >= g_costs.get(next_state_id, float("inf")):
                    continue

                g_costs[next_state_id] = next_g
                parents[next_state_id] = current.state_id
                h = self._single_agent_heuristic(
                    problem=problem,
                    state_id=next_state_id,
                    goal_state_id=goal_state_id,
                )
                heapq.heappush(
                    open_heap,
                    SingleAgentSearchNode(
                        priority=next_g + h,
                        g_cost=next_g,
                        state_id=next_state_id,
                    ),
                )

        return None

    def _reconstruct_single_agent_path(
        self,
        goal_state_id: int,
        parents: Dict[int, Optional[int]],
    ) -> List[int]:
        """単一エージェント A* の経路を復元する。

        Args:
            goal_state_id: 目標状態ID。
            parents: 親ポインタ。

        Returns:
            状態ID列。
        """
        path: List[int] = []
        current: Optional[int] = goal_state_id

        while current is not None:
            path.append(current)
            current = parents[current]

        path.reverse()
        return path

    # ------------------------------------------------------------------
    # High-level helpers
    # ------------------------------------------------------------------

    def _build_start_config(self, problem: LaCAMProblem) -> Config:
        """開始 configuration を返す。"""
        robot_ids = sorted(problem.start_state_ids.keys())
        return tuple(problem.start_state_ids[robot_id] for robot_id in robot_ids)

    def _build_goal_config(self, problem: LaCAMProblem) -> Config:
        """目標 configuration を返す。"""
        robot_ids = sorted(problem.goal_state_ids.keys())
        return tuple(problem.goal_state_ids[robot_id] for robot_id in robot_ids)

    def _make_root_node(
        self,
        problem: LaCAMProblem,
        start_config: Config,
        goal_config: Config,
    ) -> LaCAMHighLevelNode:
        """Root ノードを作る。"""
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
        """Goal configuration 判定を行う。"""
        return config == goal_config

    def _joint_heuristic(
        self,
        problem: LaCAMProblem,
        config: Config,
        goal_config: Config,
    ) -> float:
        """Joint heuristic を返す。"""
        return sum(
            self._single_agent_heuristic(problem, state_id, goal_state_id)
            for state_id, goal_state_id in zip(config, goal_config, strict=True)
        )

    def _single_agent_heuristic(
        self,
        problem: LaCAMProblem,
        state_id: int,
        goal_state_id: int,
    ) -> float:
        """単一エージェントの粗い heuristic を返す。"""
        graph = problem.graph
        state = graph.id_to_state(state_id)
        goal_state = graph.id_to_state(goal_state_id)

        dist = abs(state.row - goal_state.row) + abs(state.col - goal_state.col)
        rot = 0.0 if state.mode_deg == goal_state.mode_deg else 1.0
        return float(dist) + rot

    # ------------------------------------------------------------------
    # Low-level with PIBT
    # ------------------------------------------------------------------

    def _low_level_expand_with_pibt(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        goal_config: Config,
        constraints: Tuple[LaCAMConstraint, ...],
        next_time_step: int,
        prev_config: Optional[Config],
    ) -> LowLevelExpandResult:
        """PIBT を用いて low-level successor を構成する。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            goal_config: 目標 configuration。
            constraints: High-level 制約。
            next_time_step: 次に到達する時刻。
            prev_config: 1つ前の configuration。

        Returns:
            low-level 展開結果。
        """
        candidate_moves: Dict[int, List[CandidateMove]] = {}
        for robot_id, curr_state_id in enumerate(curr_config):
            ranked = self._enumerate_ranked_single_agent_moves(
                problem=problem,
                robot_id=robot_id,
                curr_state_id=curr_state_id,
                goal_state_id=goal_config[robot_id],
                constraints=constraints,
                next_time_step=next_time_step,
                prev_state_id=(
                    prev_config[robot_id] if prev_config is not None else None
                ),
            )
            if not ranked:
                return LowLevelExpandResult(
                    successors=tuple(),
                    blocking_conflict=None,
                    notes=f"no_single_agent_moves_for_robot={robot_id}",
                )
            candidate_moves[robot_id] = ranked

        priority_orders = self._build_priority_order_candidates(
            problem=problem,
            curr_config=curr_config,
            goal_config=goal_config,
            candidate_moves=candidate_moves,
        )

        all_successors: List[JointSuccessorCandidate] = []
        best_blocking_conflict: Optional[JointConflict] = None

        for priority_order in priority_orders[: self.max_priority_orders]:
            root = PartialConstraintNode(
                assignments=tuple(),
                depth=0,
                score=0.0,
            )
            low_open: List[LowLevelQueueEntry] = [
                LowLevelQueueEntry(priority=0.0, node=root)
            ]
            low_expansions = 0

            while (
                low_open
                and low_expansions < self.max_low_level_expansions_per_order
                and len(all_successors) < self.max_joint_successors
            ):
                low_expansions += 1
                entry = heapq.heappop(low_open)
                partial_node = entry.node

                pibt_result = self._get_new_config_with_pibt(
                    problem=problem,
                    curr_config=curr_config,
                    partial_node=partial_node,
                    priority_order=priority_order,
                    next_time_step=next_time_step,
                    goal_config=goal_config,
                    constraints=constraints,
                    candidate_moves=candidate_moves,
                )

                if pibt_result.success and pibt_result.next_config is not None:
                    score = self._score_next_config(
                        problem=problem,
                        curr_config=curr_config,
                        next_config=pibt_result.next_config,
                        goal_config=goal_config,
                    )
                    all_successors.append(
                        JointSuccessorCandidate(
                            next_config=pibt_result.next_config,
                            score=score,
                        )
                    )
                    continue

                if (
                    pibt_result.blocking_conflict is not None
                    and best_blocking_conflict is None
                ):
                    best_blocking_conflict = pibt_result.blocking_conflict

                children = self._expand_partial_constraint_node(
                    problem=problem,
                    curr_config=curr_config,
                    partial_node=partial_node,
                    candidate_moves=candidate_moves,
                    priority_order=priority_order,
                )
                for child in children:
                    heapq.heappush(
                        low_open,
                        LowLevelQueueEntry(
                            priority=child.score,
                            node=child,
                        ),
                    )

        deduped = self._deduplicate_joint_successors(
            problem=problem,
            curr_config=curr_config,
            successors=all_successors,
        )

        return LowLevelExpandResult(
            successors=tuple(deduped[: self.max_joint_successors]),
            blocking_conflict=best_blocking_conflict,
            notes="low_level_expand_with_pibt",
        )

    def _build_priority_order_candidates(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        goal_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
    ) -> List[Tuple[int, ...]]:
        """priority order 候補を構築する。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            goal_config: 目標 configuration。
            candidate_moves: ロボットID -> 候補遷移列。

        Returns:
            priority order 候補列。
        """
        robot_ids = list(range(len(curr_config)))

        base = tuple(
            sorted(
                robot_ids,
                key=lambda rid: (
                    -self._estimate_priority_score(
                        problem=problem,
                        curr_config=curr_config,
                        goal_config=goal_config,
                        robot_id=rid,
                        candidate_moves=candidate_moves,
                    ),
                    len(candidate_moves[rid]),
                ),
            )
        )

        orders: List[Tuple[int, ...]] = [base]

        reverse = tuple(reversed(base))
        if reverse not in orders:
            orders.append(reverse)

        pair = self._find_critical_pair_from_top_choices(
            problem=problem,
            curr_config=curr_config,
            candidate_moves=candidate_moves,
            time_step=0,
        )
        if pair is not None:
            i, j = pair
            pair_first = self._move_ids_to_front(base, (i, j))
            if pair_first not in orders:
                orders.append(pair_first)
            pair_second = self._move_ids_to_front(base, (j, i))
            if pair_second not in orders:
                orders.append(pair_second)

        group = self._select_focus_group(
            problem=problem,
            curr_config=curr_config,
            goal_config=goal_config,
            candidate_moves=candidate_moves,
        )
        if group is not None and len(group) >= 3:
            group_order = self._enumerate_group_order_patterns(
                problem=problem,
                curr_config=curr_config,
                goal_config=goal_config,
                candidate_moves=candidate_moves,
                focus_group=group,
            )
            for order in group_order:
                merged = self._move_ids_to_front(base, order)
                if merged not in orders:
                    orders.append(merged)

        return orders

    def _estimate_priority_score(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        goal_config: Config,
        robot_id: int,
        candidate_moves: Dict[int, List[CandidateMove]],
    ) -> float:
        """priority order 用の粗いスコアを返す。"""
        h = self._single_agent_heuristic(
            problem=problem,
            state_id=curr_config[robot_id],
            goal_state_id=goal_config[robot_id],
        )
        move_rotate_gain = 0.0
        if candidate_moves[robot_id]:
            if candidate_moves[robot_id][0].action_type == ActionType.MOVE_ROTATE:
                move_rotate_gain = 0.5
        return h + move_rotate_gain

    def _move_ids_to_front(
        self,
        base_order: Sequence[int],
        front_ids: Sequence[int],
    ) -> Tuple[int, ...]:
        """指定した ID 群を順番を保って先頭へ移す。"""
        front_set = set(front_ids)
        tail = [rid for rid in base_order if rid not in front_set]
        return tuple(list(front_ids) + tail)

    def _get_new_config_with_pibt(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        partial_node: PartialConstraintNode,
        priority_order: Sequence[int],
        next_time_step: int,
        goal_config: Config,
        constraints: Tuple[LaCAMConstraint, ...],
        candidate_moves: Dict[int, List[CandidateMove]],
    ) -> PIBTResult:
        """部分制約を満たしつつ PIBT で next_config を構成する。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            partial_node: 部分制約ノード。
            priority_order: ロボット優先順。
            next_time_step: 次に到達する時刻。
            goal_config: 目標 configuration。
            constraints: High-level 制約。
            candidate_moves: 候補遷移列。

        Returns:
            PIBT 実行結果。
        """
        context = PIBTContext(
            curr_config=curr_config,
            next_time_step=next_time_step,
            fixed_next={},
            occupied_next={},
            priority_order=tuple(priority_order),
            active_stack=set(),
            candidate_moves=candidate_moves,
            partial_fixed_robots=set(),
        )

        # 1. 部分制約で固定されたロボットを先に置く
        for robot_id, next_state_id in partial_node.assignments:
            conflict = self._can_assign_next_state(
                problem=problem,
                robot_id=robot_id,
                src_state_id=curr_config[robot_id],
                dst_state_id=next_state_id,
                context=context,
                constraints=constraints,
            )
            if conflict is not None:
                return PIBTResult(
                    success=False,
                    next_config=None,
                    blocking_conflict=conflict,
                    notes="partial_assignment_conflict",
                )
            context.fixed_next[robot_id] = next_state_id
            context.occupied_next[next_state_id] = robot_id
            context.partial_fixed_robots.add(robot_id)

        # 2. 未確定ロボットを priority order 順に PIBT で埋める
        for robot_id in priority_order:
            if robot_id in context.fixed_next:
                continue
            success = self._func_pibt(
                problem=problem,
                robot_id=robot_id,
                context=context,
                goal_config=goal_config,
                constraints=constraints,
            )
            if not success:
                return PIBTResult(
                    success=False,
                    next_config=None,
                    blocking_conflict=None,
                    notes=f"pibt_failed_for_robot={robot_id}",
                )

        if len(context.fixed_next) != len(curr_config):
            return PIBTResult(
                success=False,
                next_config=None,
                blocking_conflict=None,
                notes="not_all_agents_assigned",
            )

        next_config = tuple(
            context.fixed_next[robot_id] for robot_id in range(len(curr_config))
        )
        return PIBTResult(
            success=True,
            next_config=next_config,
            blocking_conflict=None,
            notes="pibt_success",
        )

    def _func_pibt(
        self,
        problem: LaCAMProblem,
        robot_id: int,
        context: PIBTContext,
        goal_config: Config,
        constraints: Tuple[LaCAMConstraint, ...],
    ) -> bool:
        """priority inheritance 付きで robot の next state を決める。

        Args:
            problem: LaCAM 問題。
            robot_id: 対象ロボットID。
            context: PIBT 実行コンテキスト。
            goal_config: 目標 configuration。
            constraints: High-level 制約。

        Returns:
            割当成功なら True。
        """
        del goal_config

        if robot_id in context.fixed_next:
            return True

        if robot_id in context.active_stack:
            return False

        context.active_stack.add(robot_id)
        curr_state_id = context.curr_config[robot_id]

        try:
            for move in context.candidate_moves[robot_id]:
                next_state_id = move.next_state_id

                # 既に同じ頂点を他ロボットが予約している場合
                occupant = context.occupied_next.get(next_state_id)
                if occupant is not None and occupant != robot_id:
                    if occupant in context.partial_fixed_robots:
                        continue

                    # occupant を一旦外して再配置を試みる
                    old_next_state = context.fixed_next.get(occupant)
                    if old_next_state is None:
                        continue

                    del context.fixed_next[occupant]
                    del context.occupied_next[old_next_state]

                    displaced_success = self._func_pibt(
                        problem=problem,
                        robot_id=occupant,
                        context=context,
                        goal_config=goal_config,
                        constraints=constraints,
                    )

                    if not displaced_success:
                        context.fixed_next[occupant] = old_next_state
                        context.occupied_next[old_next_state] = occupant
                        continue

                conflict = self._can_assign_next_state(
                    problem=problem,
                    robot_id=robot_id,
                    src_state_id=curr_state_id,
                    dst_state_id=next_state_id,
                    context=context,
                    constraints=constraints,
                )
                if conflict is not None:
                    continue

                context.fixed_next[robot_id] = next_state_id
                context.occupied_next[next_state_id] = robot_id
                return True

            return False

        finally:
            context.active_stack.remove(robot_id)

    def _can_assign_next_state(
        self,
        problem: LaCAMProblem,
        robot_id: int,
        src_state_id: int,
        dst_state_id: int,
        context: PIBTContext,
        constraints: Tuple[LaCAMConstraint, ...],
    ) -> Optional[JointConflict]:
        """next state を割り当ててよいか判定する。

        Args:
            problem: LaCAM 問題。
            robot_id: 対象ロボットID。
            src_state_id: 現在状態ID。
            dst_state_id: 候補 next state。
            context: PIBT コンテキスト。
            constraints: High-level 制約。

        Returns:
            conflict があればそれを返し、なければ None。
        """
        if self._violates_constraints(
            robot_id=robot_id,
            src_state_id=src_state_id,
            dst_state_id=dst_state_id,
            next_time_step=context.next_time_step,
            constraints=constraints,
        ):
            return JointConflict(
                conflict_type="high_level_constraint",
                time_step=context.next_time_step,
                robot_i=robot_id,
                robot_j=robot_id,
                state_i=dst_state_id,
                transition_i=(src_state_id, dst_state_id),
                notes="violates high-level constraint",
            )

        for other_robot_id, other_next_state_id in context.fixed_next.items():
            if other_robot_id == robot_id:
                continue

            other_src_state_id = context.curr_config[other_robot_id]
            conflict = self._detect_pair_conflict(
                problem=problem,
                robot_i=robot_id,
                src_i=src_state_id,
                dst_i=dst_state_id,
                robot_j=other_robot_id,
                src_j=other_src_state_id,
                dst_j=other_next_state_id,
                time_step=context.next_time_step,
            )
            if conflict is not None:
                return conflict

            # swap 的な conflict も明示的に確認
            if (
                other_next_state_id == src_state_id
                and other_src_state_id == dst_state_id
            ):
                return JointConflict(
                    conflict_type="swap_conflict",
                    time_step=context.next_time_step,
                    robot_i=robot_id,
                    robot_j=other_robot_id,
                    state_i=dst_state_id,
                    state_j=other_next_state_id,
                    transition_i=(src_state_id, dst_state_id),
                    transition_j=(other_src_state_id, other_next_state_id),
                    notes="swap conflict",
                )

        return None

    def _expand_partial_constraint_node(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        partial_node: PartialConstraintNode,
        candidate_moves: Dict[int, List[CandidateMove]],
        priority_order: Sequence[int],
    ) -> List[PartialConstraintNode]:
        """部分制約ノードを 1 段深く展開する。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            partial_node: 展開対象部分制約。
            candidate_moves: ロボットID -> 候補遷移列。
            priority_order: ロボット優先順。

        Returns:
            子部分制約ノード列。
        """
        del problem

        assigned_robots = {robot_id for robot_id, _ in partial_node.assignments}
        target_robot: Optional[int] = None

        for robot_id in priority_order:
            if robot_id not in assigned_robots:
                target_robot = robot_id
                break

        if target_robot is None:
            return []

        children: List[PartialConstraintNode] = []
        base_assignments = list(partial_node.assignments)
        curr_state_id = curr_config[target_robot]

        for move in candidate_moves[target_robot][: self.max_partial_constraints_per_expand]:
            # 部分制約として固定する候補を追加
            new_assignments = base_assignments + [(target_robot, move.next_state_id)]
            score = (
                partial_node.score
                + move.score
                - 0.1 * partial_node.depth
            )
            children.append(
                PartialConstraintNode(
                    assignments=tuple(new_assignments),
                    depth=partial_node.depth + 1,
                    score=score,
                )
            )

        return children

    # ------------------------------------------------------------------
    # Pair / Group logic -> priority order generation
    # ------------------------------------------------------------------

    def _find_critical_pair_from_top_choices(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
        time_step: int,
    ) -> Optional[Tuple[int, int]]:
        """top-choice 衝突で最初に見つかる pair を返す。"""
        num_agents = len(curr_config)

        for robot_i in range(num_agents):
            if not candidate_moves[robot_i]:
                continue
            top_i = candidate_moves[robot_i][0].next_state_id

            for robot_j in range(robot_i + 1, num_agents):
                if not candidate_moves[robot_j]:
                    continue
                top_j = candidate_moves[robot_j][0].next_state_id

                conflict = self._detect_pair_conflict(
                    problem=problem,
                    robot_i=robot_i,
                    src_i=curr_config[robot_i],
                    dst_i=top_i,
                    robot_j=robot_j,
                    src_j=curr_config[robot_j],
                    dst_j=top_j,
                    time_step=time_step,
                )
                if conflict is not None:
                    return (robot_i, robot_j)

        return None

    def _build_interaction_graph(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
        time_step: int,
    ) -> Dict[int, Set[int]]:
        """top-choice 衝突に基づく interaction graph を作る。"""
        graph: Dict[int, Set[int]] = {
            robot_id: set() for robot_id in candidate_moves.keys()
        }

        robot_ids = list(candidate_moves.keys())
        for i_idx, robot_i in enumerate(robot_ids):
            if not candidate_moves[robot_i]:
                continue
            top_i = candidate_moves[robot_i][0].next_state_id

            for robot_j in robot_ids[i_idx + 1 :]:
                if not candidate_moves[robot_j]:
                    continue
                top_j = candidate_moves[robot_j][0].next_state_id

                conflict = self._detect_pair_conflict(
                    problem=problem,
                    robot_i=robot_i,
                    src_i=curr_config[robot_i],
                    dst_i=top_i,
                    robot_j=robot_j,
                    src_j=curr_config[robot_j],
                    dst_j=top_j,
                    time_step=time_step,
                )
                if conflict is not None:
                    graph[robot_i].add(robot_j)
                    graph[robot_j].add(robot_i)

        return graph

    def _detect_critical_groups(
        self,
        interaction_graph: Dict[int, Set[int]],
    ) -> List[Tuple[int, ...]]:
        """interaction graph の連結成分を critical group として返す。"""
        visited: Set[int] = set()
        groups: List[Tuple[int, ...]] = []

        for robot_id in interaction_graph:
            if robot_id in visited:
                continue

            stack = [robot_id]
            component: List[int] = []

            while stack:
                node = stack.pop()
                if node in visited:
                    continue
                visited.add(node)
                component.append(node)

                for nbr in interaction_graph[node]:
                    if nbr not in visited:
                        stack.append(nbr)

            if len(component) >= 2:
                groups.append(tuple(sorted(component)))

        return groups

    def _select_focus_group(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        goal_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
    ) -> Optional[Tuple[int, ...]]:
        """最も重要な focus group を 1 つ選ぶ。"""
        interaction_graph = self._build_interaction_graph(
            problem=problem,
            curr_config=curr_config,
            candidate_moves=candidate_moves,
            time_step=0,
        )
        critical_groups = self._detect_critical_groups(interaction_graph)
        if not critical_groups:
            return None

        scored_groups: List[Tuple[float, Tuple[int, ...]]] = []
        for group in critical_groups:
            size_score = 100.0 * len(group)
            bottleneck_score = sum(
                self._estimate_local_bottleneck_score(
                    problem=problem,
                    curr_config=curr_config,
                    robot_id=robot_id,
                    candidate_moves=candidate_moves,
                    goal_config=goal_config,
                    interaction_graph=interaction_graph,
                )
                for robot_id in group
            )
            total_score = size_score + bottleneck_score
            scored_groups.append((total_score, group))

        scored_groups.sort(key=lambda item: item[0], reverse=True)
        return scored_groups[0][1]

    def _estimate_local_bottleneck_score(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        robot_id: int,
        candidate_moves: Dict[int, List[CandidateMove]],
        goal_config: Config,
        interaction_graph: Dict[int, Set[int]],
    ) -> float:
        """局所 bottleneck score を返す。"""
        degree = len(interaction_graph.get(robot_id, set()))
        goal_dist = self._single_agent_heuristic(
            problem=problem,
            state_id=curr_config[robot_id],
            goal_state_id=goal_config[robot_id],
        )
        move_rotate_gain = 0.0
        if candidate_moves[robot_id]:
            top_move = candidate_moves[robot_id][0]
            if top_move.action_type == ActionType.MOVE_ROTATE:
                move_rotate_gain = 0.5
        return 10.0 * degree + goal_dist + move_rotate_gain

    def _enumerate_group_order_patterns(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        goal_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
        focus_group: Tuple[int, ...],
    ) -> List[Tuple[int, ...]]:
        """focus group に対する順序候補を列挙する。"""
        interaction_graph = self._build_interaction_graph(
            problem=problem,
            curr_config=curr_config,
            candidate_moves=candidate_moves,
            time_step=0,
        )

        scored = sorted(
            focus_group,
            key=lambda rid: self._estimate_local_bottleneck_score(
                problem=problem,
                curr_config=curr_config,
                robot_id=rid,
                candidate_moves=candidate_moves,
                goal_config=goal_config,
                interaction_graph=interaction_graph,
            ),
            reverse=True,
        )
        base = tuple(scored)
        patterns: List[Tuple[int, ...]] = [base]

        reverse = tuple(reversed(base))
        if reverse not in patterns:
            patterns.append(reverse)

        if len(base) >= 2:
            swapped = list(base)
            swapped[0], swapped[1] = swapped[1], swapped[0]
            swapped_t = tuple(swapped)
            if swapped_t not in patterns:
                patterns.append(swapped_t)

        if len(base) >= 3:
            rotated = tuple(list(base[1:]) + [base[0]])
            if rotated not in patterns:
                patterns.append(rotated)

        return patterns

    # ------------------------------------------------------------------
    # Move ranking
    # ------------------------------------------------------------------

    def _enumerate_ranked_single_agent_moves(
        self,
        problem: LaCAMProblem,
        robot_id: int,
        curr_state_id: int,
        goal_state_id: int,
        constraints: Tuple[LaCAMConstraint, ...],
        next_time_step: int,
        prev_state_id: Optional[int],
    ) -> List[CandidateMove]:
        """単一ロボットの候補遷移を ranking 付きで列挙する。"""
        graph = problem.graph
        curr_h = self._single_agent_heuristic(
            problem=problem,
            state_id=curr_state_id,
            goal_state_id=goal_state_id,
        )
        is_curr_goal = curr_state_id == goal_state_id

        ranked: List[CandidateMove] = []

        for next_state_id, edge in graph.neighbors(curr_state_id):
            if self._violates_constraints(
                robot_id=robot_id,
                src_state_id=curr_state_id,
                dst_state_id=next_state_id,
                next_time_step=next_time_step,
                constraints=constraints,
            ):
                continue

            next_h = self._single_agent_heuristic(
                problem=problem,
                state_id=next_state_id,
                goal_state_id=goal_state_id,
            )
            progress = curr_h - next_h
            score = next_h + 0.1 * edge.cost

            is_wait = (
                edge.action_type == ActionType.WAIT
                or next_state_id == curr_state_id
            )

            if is_wait:
                if is_curr_goal:
                    score += self.goal_wait_bonus
                else:
                    score += self.wait_penalty

            if is_curr_goal and next_state_id != curr_state_id:
                score += self.goal_leave_penalty

            if progress <= 0.0 and next_state_id != goal_state_id:
                score += self.no_progress_penalty

            if prev_state_id is not None and next_state_id == prev_state_id:
                score += self.backward_penalty

            if edge.action_type == ActionType.MOVE_ROTATE:
                score += self.move_rotate_bonus

            if edge.action_type == ActionType.ROTATE and progress > 0.0:
                score += self.rotate_progress_bonus

            ranked.append(
                CandidateMove(
                    next_state_id=next_state_id,
                    score=score,
                    edge_cost=edge.cost,
                    action_type=edge.action_type,
                )
            )

        ranked.sort(key=lambda move: move.score)
        return ranked[: self.max_single_agent_candidates]

    def _violates_constraints(
        self,
        robot_id: int,
        src_state_id: int,
        dst_state_id: int,
        next_time_step: int,
        constraints: Tuple[LaCAMConstraint, ...],
    ) -> bool:
        """禁止制約に違反するか判定する。"""
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

    # ------------------------------------------------------------------
    # Conflict detection
    # ------------------------------------------------------------------

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
        """2ロボット間の競合を返す。"""
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
        """同時到着状態の occupancy overlap を判定する。"""
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
        """同時遷移の envelope overlap を判定する。"""
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

    # ------------------------------------------------------------------
    # High-level branching / scoring / dedup
    # ------------------------------------------------------------------

    def _branch_on_conflict(
        self,
        problem: LaCAMProblem,
        node: LaCAMHighLevelNode,
        conflict: JointConflict,
        goal_config: Config,
    ) -> List[LaCAMHighLevelNode]:
        """競合に対して child node を作る。"""
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

        return self._branch_when_no_successor(
            problem=problem,
            node=node,
            curr_config=node.config_sequence[-1],
            next_time_step=conflict.time_step,
            goal_config=goal_config,
        )

    def _branch_when_no_successor(
        self,
        problem: LaCAMProblem,
        node: LaCAMHighLevelNode,
        curr_config: Config,
        next_time_step: int,
        goal_config: Config,
    ) -> List[LaCAMHighLevelNode]:
        """joint successor が作れない場合の fallback 分岐。"""
        graph = problem.graph
        candidates: List[Tuple[float, int, int]] = []

        for robot_id, curr_state_id in enumerate(curr_config):
            local_ranked: List[Tuple[float, int]] = []

            for next_state_id, edge in graph.neighbors(curr_state_id):
                if self._violates_constraints(
                    robot_id=robot_id,
                    src_state_id=curr_state_id,
                    dst_state_id=next_state_id,
                    next_time_step=next_time_step,
                    constraints=node.constraints,
                ):
                    continue

                next_h = self._single_agent_heuristic(
                    problem=problem,
                    state_id=next_state_id,
                    goal_state_id=goal_config[robot_id],
                )
                score = next_h + 0.1 * edge.cost
                local_ranked.append((score, next_state_id))

            local_ranked.sort(key=lambda item: item[0])

            for score, next_state_id in local_ranked[:2]:
                candidates.append((score, robot_id, next_state_id))

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
                    notes="branch_no_successor_v5",
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
        """Constraint を 1 個追加した child node を作る。"""
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
        successor_score_bias: float,
    ) -> LaCAMHighLevelNode:
        """1 step 進めた child node を作る。"""
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

        priority = next_cost + h + 0.05 * successor_score_bias

        return LaCAMHighLevelNode(
            priority=priority,
            cost_so_far=next_cost,
            lower_bound=next_cost + h,
            config_sequence=list(node.config_sequence) + [next_config],
            constraints=node.constraints,
            notes="extend_v5",
        )

    def _compute_joint_step_cost(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        next_config: Config,
    ) -> float:
        """1 step 分の joint cost を返す。"""
        graph = problem.graph
        total = 0.0

        for src_state_id, dst_state_id in zip(curr_config, next_config, strict=True):
            edge = graph.edge_from_state_ids(src_state_id, dst_state_id)
            total += edge.cost

        return total

    def _score_next_config(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        next_config: Config,
        goal_config: Config,
    ) -> float:
        """next_config の簡易スコアを返す。"""
        return self._joint_heuristic(
            problem=problem,
            config=next_config,
            goal_config=goal_config,
        ) + 0.1 * self._compute_joint_step_cost(
            problem=problem,
            curr_config=curr_config,
            next_config=next_config,
        )

    def _joint_successor_signature(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        next_config: Config,
    ) -> Tuple[Tuple[bool, str], ...]:
        """全体 successor の簡易シグネチャを返す。"""
        graph = problem.graph
        signature: List[Tuple[bool, str]] = []

        for src_state_id, dst_state_id in zip(curr_config, next_config, strict=True):
            edge = graph.edge_from_state_ids(src_state_id, dst_state_id)
            changed = src_state_id != dst_state_id
            signature.append((changed, edge.action_type.value))

        return tuple(signature)

    def _deduplicate_joint_successors(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        successors: Sequence[JointSuccessorCandidate],
    ) -> List[JointSuccessorCandidate]:
        """類似 successor を削減する。"""
        best_by_config: Dict[Config, JointSuccessorCandidate] = {}
        for successor in successors:
            existing = best_by_config.get(successor.next_config)
            if existing is None or successor.score < existing.score:
                best_by_config[successor.next_config] = successor

        best_by_signature: Dict[
            Tuple[Tuple[bool, str], ...],
            JointSuccessorCandidate,
        ] = {}
        for successor in best_by_config.values():
            signature = self._joint_successor_signature(
                problem=problem,
                curr_config=curr_config,
                next_config=successor.next_config,
            )
            existing = best_by_signature.get(signature)
            if existing is None or successor.score < existing.score:
                best_by_signature[signature] = successor

        deduped = list(best_by_signature.values())
        deduped.sort(key=lambda item: item.score)
        return deduped

    def _reconstruct_paths_from_config_sequence(
        self,
        config_sequence: Sequence[Config],
    ) -> Dict[int, List[int]]:
        """config sequence をロボット別状態列へ復元する。"""
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