from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Sequence, Tuple

from lacam_adapter import LaCAMProblem, LaCAMSolution
from lacam_runner import LaCAMBackend, LaCAMRunnerConfig
from mapf_types import ActionType

Config = Tuple[int, ...]


@dataclass(frozen=True)
class LaCAMConstraint:
    """LaCAM の禁止制約。

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


class OrientationLaCAMBackendV3(LaCAMBackend):
    """姿勢付き MAPF 向け LaCAM 骨格 backend の v3。

    v3 の改善点:
        - 単一エージェント fast path を導入
        - pair-priority successor を優先生成
        - WAIT を goal_hold / yield / idle で扱い分け
        - 類似 successor を signature で削減
    """

    def __init__(
        self,
        max_high_level_expansions: int = 20_000,
        max_depth: int = 128,
        max_branch_children: int = 8,
        max_single_agent_candidates: int = 6,
        max_joint_successors: int = 6,
        pair_priority_successors_per_pattern: int = 2,
    ) -> None:
        """初期化する。

        Args:
            max_high_level_expansions: High-level 展開回数上限。
            max_depth: `config_sequence` の最大長。
            max_branch_children: branch で作る child 上限。
            max_single_agent_candidates: 各ロボットの候補数上限。
            max_joint_successors: 各ノードから作る joint successor 数上限。
            pair_priority_successors_per_pattern:
                pair-priority パターンごとの生成上限。
        """
        self.max_high_level_expansions = max_high_level_expansions
        self.max_depth = max_depth
        self.max_branch_children = max_branch_children
        self.max_single_agent_candidates = max_single_agent_candidates
        self.max_joint_successors = max_joint_successors
        self.pair_priority_successors_per_pattern = (
            pair_priority_successors_per_pattern
        )

        # ranking 用重み
        self.wait_penalty = 1.5
        self.backward_penalty = 1.0
        self.no_progress_penalty = 0.75
        self.goal_leave_penalty = 4.0
        self.goal_wait_bonus = -2.0
        self.move_rotate_bonus = -0.2
        self.rotate_progress_bonus = -0.1
        self.yield_wait_bonus = -1.0
        self.advance_wait_penalty = 1.0

    def solve(
        self,
        problem: LaCAMProblem,
        config: LaCAMRunnerConfig,
    ) -> LaCAMSolution:
        """LaCAM 骨格 v3 で問題を解く。

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

            if self._is_goal_config(curr_config, self._build_goal_config(problem)):
                return LaCAMSolution(
                    solved=True,
                    comp_time_sec=0.0,
                    paths_by_agent=self._reconstruct_paths_from_config_sequence(
                        node.config_sequence
                    ),
                    raw_status="solved",
                    notes="orientation_lacam_v3",
                )

            if len(node.config_sequence) >= self.max_depth:
                continue

            next_time_step = len(node.config_sequence)
            prev_config = (
                node.config_sequence[-2]
                if len(node.config_sequence) >= 2
                else None
            )
            goal_config = self._build_goal_config(problem)

            expand_result = self._low_level_expand(
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
        start_state_id = next(iter(problem.start_state_ids.values()))
        goal_state_id = next(iter(problem.goal_state_ids.values()))

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
            paths_by_agent={0: state_ids},
            raw_status="solved",
            notes="orientation_lacam_v3_single_agent_fast_path",
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
        """単一エージェント A* の経路を復元する。"""
        path: List[int] = []
        current: Optional[int] = goal_state_id

        while current is not None:
            path.append(current)
            current = parents[current]

        path.reverse()
        return path

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

    def _low_level_expand(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        goal_config: Config,
        constraints: Tuple[LaCAMConstraint, ...],
        next_time_step: int,
        prev_config: Optional[Config],
    ) -> LowLevelExpandResult:
        """複数 joint successor を生成する low-level 展開。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            goal_config: 目標 configuration。
            constraints: 制約集合。
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

        pair_priority_successors = self._generate_pair_priority_successors(
            problem=problem,
            curr_config=curr_config,
            candidate_moves=candidate_moves,
            next_time_step=next_time_step,
        )

        general_result = self._construct_joint_successors(
            problem=problem,
            curr_config=curr_config,
            candidate_moves=candidate_moves,
            next_time_step=next_time_step,
            max_successors=self.max_joint_successors,
        )

        merged = list(pair_priority_successors) + list(general_result.successors)
        deduped = self._deduplicate_joint_successors(
            problem=problem,
            curr_config=curr_config,
            successors=merged,
        )

        return LowLevelExpandResult(
            successors=tuple(deduped[: self.max_joint_successors]),
            blocking_conflict=general_result.blocking_conflict,
            notes="pair_priority_plus_general",
        )

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
        """単一ロボットの候補遷移を ranking 付きで列挙する。

        Args:
            problem: LaCAM 問題。
            robot_id: ロボットID。
            curr_state_id: 現在状態ID。
            goal_state_id: 目標状態ID。
            constraints: 制約集合。
            next_time_step: 次に到達する時刻。
            prev_state_id: 1つ前の状態ID。

        Returns:
            ranking 済み候補列。
        """
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

    def _generate_pair_priority_successors(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
        next_time_step: int,
    ) -> List[JointSuccessorCandidate]:
        """2ロボットの譲り役を明示した successor を優先生成する。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            candidate_moves: ロボットID -> 候補遷移列。
            next_time_step: 次に到達する時刻。

        Returns:
            pair-priority から作った successor 候補列。
        """
        critical_pair = self._find_critical_pair_from_top_choices(
            problem=problem,
            curr_config=curr_config,
            candidate_moves=candidate_moves,
            time_step=next_time_step,
        )
        if critical_pair is None:
            return []

        robot_i, robot_j = critical_pair
        successors: List[JointSuccessorCandidate] = []

        for winner, loser in ((robot_i, robot_j), (robot_j, robot_i)):
            adjusted = self._build_role_adjusted_candidate_moves(
                curr_config=curr_config,
                candidate_moves=candidate_moves,
                winner=winner,
                loser=loser,
            )
            result = self._construct_joint_successors(
                problem=problem,
                curr_config=curr_config,
                candidate_moves=adjusted,
                next_time_step=next_time_step,
                max_successors=self.pair_priority_successors_per_pattern,
            )
            successors.extend(result.successors)

        return successors

    def _find_critical_pair_from_top_choices(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
        time_step: int,
    ) -> Optional[Tuple[int, int]]:
        """top 候補同士で最初に衝突する pair を返す。"""
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

    def _build_role_adjusted_candidate_moves(
        self,
        curr_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
        winner: int,
        loser: int,
    ) -> Dict[int, List[CandidateMove]]:
        """winner / loser の役割に応じて候補順を調整する。"""
        adjusted: Dict[int, List[CandidateMove]] = {}

        for robot_id, moves in candidate_moves.items():
            if robot_id == winner:
                adjusted[robot_id] = self._sort_moves_for_advance(
                    curr_state_id=curr_config[robot_id],
                    moves=moves,
                )
            elif robot_id == loser:
                adjusted[robot_id] = self._sort_moves_for_yield(
                    curr_state_id=curr_config[robot_id],
                    moves=moves,
                )
            else:
                adjusted[robot_id] = list(moves)

        return adjusted

    def _sort_moves_for_advance(
        self,
        curr_state_id: int,
        moves: Sequence[CandidateMove],
    ) -> List[CandidateMove]:
        """advance 役に向く順に並べる。"""
        def key_fn(move: CandidateMove) -> Tuple[float, float]:
            is_wait = move.next_state_id == curr_state_id
            score = move.score + (self.advance_wait_penalty if is_wait else 0.0)
            return (score, move.edge_cost)

        return sorted(moves, key=key_fn)

    def _sort_moves_for_yield(
        self,
        curr_state_id: int,
        moves: Sequence[CandidateMove],
    ) -> List[CandidateMove]:
        """yield 役に向く順に並べる。"""
        def key_fn(move: CandidateMove) -> Tuple[int, float]:
            is_wait = move.next_state_id == curr_state_id
            adjusted_score = move.score + (
                self.yield_wait_bonus if is_wait else 0.0
            )
            return (0 if is_wait else 1, adjusted_score)

        return sorted(moves, key=key_fn)

    def _construct_joint_successors(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        candidate_moves: Dict[int, List[CandidateMove]],
        next_time_step: int,
        max_successors: int,
    ) -> LowLevelExpandResult:
        """複数の feasible joint successor を構築する。

        Args:
            problem: LaCAM 問題。
            curr_config: 現在 configuration。
            candidate_moves: ロボットID -> 候補遷移列。
            next_time_step: 次に到達する時刻。
            max_successors: 生成上限。

        Returns:
            low-level 展開結果。
        """
        robot_order = self._decide_assignment_order(candidate_moves)
        partial_next: Dict[int, int] = {}
        collected: List[JointSuccessorCandidate] = []
        best_blocking_conflict: Optional[JointConflict] = None

        def dfs(index: int, cumulative_score: float) -> None:
            nonlocal best_blocking_conflict

            if len(collected) >= max_successors:
                return

            if index >= len(robot_order):
                next_config = tuple(
                    partial_next[robot_id] for robot_id in range(len(curr_config))
                )
                collected.append(
                    JointSuccessorCandidate(
                        next_config=next_config,
                        score=cumulative_score,
                    )
                )
                return

            robot_id = robot_order[index]
            curr_state_id = curr_config[robot_id]
            local_had_candidate = False

            for move in candidate_moves[robot_id]:
                conflict = self._detect_partial_conflict(
                    problem=problem,
                    robot_id=robot_id,
                    src_state_id=curr_state_id,
                    candidate_state_id=move.next_state_id,
                    curr_config=curr_config,
                    partial_next=partial_next,
                    time_step=next_time_step,
                )
                if conflict is not None:
                    if best_blocking_conflict is None:
                        best_blocking_conflict = conflict
                    continue

                local_had_candidate = True
                partial_next[robot_id] = move.next_state_id
                dfs(index + 1, cumulative_score + move.score)
                del partial_next[robot_id]

            if not local_had_candidate and best_blocking_conflict is None:
                best_blocking_conflict = JointConflict(
                    conflict_type="assignment_failed",
                    time_step=next_time_step,
                    robot_i=robot_id,
                    robot_j=robot_id,
                    notes="all candidates blocked or infeasible",
                )

        dfs(0, 0.0)
        collected.sort(key=lambda item: item.score)

        unique: List[JointSuccessorCandidate] = []
        seen: set[Config] = set()
        for item in collected:
            if item.next_config in seen:
                continue
            seen.add(item.next_config)
            unique.append(item)

        return LowLevelExpandResult(
            successors=tuple(unique[:max_successors]),
            blocking_conflict=best_blocking_conflict,
            notes="constructed_joint_successors",
        )

    def _decide_assignment_order(
        self,
        candidate_moves: Dict[int, List[CandidateMove]],
    ) -> List[int]:
        """joint assignment の順序を決める。"""
        robot_ids = list(candidate_moves.keys())
        robot_ids.sort(
            key=lambda rid: (
                len(candidate_moves[rid]),
                candidate_moves[rid][0].score
                if candidate_moves[rid]
                else float("inf"),
            )
        )
        return robot_ids

    def _detect_partial_conflict(
        self,
        problem: LaCAMProblem,
        robot_id: int,
        src_state_id: int,
        candidate_state_id: int,
        curr_config: Config,
        partial_next: Dict[int, int],
        time_step: int,
    ) -> Optional[JointConflict]:
        """部分割当との競合を返す。"""
        for other_robot_id, other_next_state_id in partial_next.items():
            other_curr_state_id = curr_config[other_robot_id]
            conflict = self._detect_pair_conflict(
                problem=problem,
                robot_i=robot_id,
                src_i=src_state_id,
                dst_i=candidate_state_id,
                robot_j=other_robot_id,
                src_j=other_curr_state_id,
                dst_j=other_next_state_id,
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
                    notes="branch_no_successor_v3",
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
            notes="extend_v3",
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

    def _joint_successor_signature(
        self,
        problem: LaCAMProblem,
        curr_config: Config,
        next_config: Config,
    ) -> Tuple[Tuple[bool, str], ...]:
        """joint successor の簡易シグネチャを返す。

        類似 successor を削減するために、
        各ロボットについて「変化したか」と「行動種別」の組を並べる。
        """
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