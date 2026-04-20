from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Optional, Tuple

from lacam_adapter import LaCAMProblem, LaCAMSolution
from lacam_runner import LaCAMBackend, LaCAMRunnerConfig


@dataclass(frozen=True)
class ReservationTable:
    """固定済みロボット群から作る予約表。

    Attributes:
        state_reservations: 時刻 -> occupied fine cell 集合リスト。
        transition_reservations: 時刻 -> occupied envelope 集合リスト。
        final_time_by_agent: ロボットごとの終端時刻。
    """

    state_reservations: Dict[int, List[FrozenSet[tuple[int, int]]]]
    transition_reservations: Dict[int, List[FrozenSet[tuple[int, int]]]]
    final_time_by_agent: Dict[int, int]


@dataclass(order=True)
class SearchNode:
    """時間拡張A*用ノード。"""

    priority: float
    g_cost: float
    state_id: int
    time_step: int


@dataclass(frozen=True)
class RepairCandidate:
    """repair 候補。

    Attributes:
        paths_by_agent: 候補経路群。
        repaired_agents: 再計画したロボットID列。
        num_conflicts: 候補に残る競合数。
        makespan: makespan。
        sum_of_costs: cost 総和。
        score: 総合スコア。
        notes: 補助説明。
    """

    paths_by_agent: Dict[int, List[int]]
    repaired_agents: Tuple[int, ...]
    num_conflicts: int
    makespan: float
    sum_of_costs: float
    score: float
    notes: str


class PythonOrientationLaCAMBackend(LaCAMBackend):
    """`OrientationGraph` を直接使う改修版 backend。

    注意:
        これは LaCAM* の完全再現ではなく、
        Phase 2 比較のための姿勢付き backend 実装である。
    """

    def __init__(
        self,
        max_repair_iterations: int = 100,
        max_time_expansion: int = 80,
    ) -> None:
        """初期化する。

        Args:
            max_repair_iterations: 競合解消反復回数上限。
            max_time_expansion: 単一ロボット再計画時の時間展開上限。
        """
        self.max_repair_iterations = max_repair_iterations
        self.max_time_expansion = max_time_expansion

    def solve(
        self,
        problem: LaCAMProblem,
        config: LaCAMRunnerConfig,
    ) -> LaCAMSolution:
        """問題を解く。

        Args:
            problem: LaCAM 問題。
            config: 実行設定。

        Returns:
            LaCAM 解。
        """
        paths = self._build_initial_solution(problem=problem)
        if paths is None:
            return LaCAMSolution(
                solved=False,
                comp_time_sec=0.0,
                paths_by_agent={},
                raw_status="initial_planning_failed",
                notes="Failed to build initial solution",
            )

        for _ in range(self.max_repair_iterations):
            conflict = self._detect_first_conflict(
                problem=problem,
                paths_by_agent=paths,
            )
            if conflict is None:
                return LaCAMSolution(
                    solved=True,
                    comp_time_sec=0.0,
                    paths_by_agent=paths,
                    raw_status="solved",
                    notes="python_orientation_lacam_step2",
                )

            robot_i, robot_j = conflict

            repaired = self._repair_conflicted_agents(
                problem=problem,
                paths_by_agent=paths,
                robot_i=robot_i,
                robot_j=robot_j,
            )
            if repaired is None:
                return LaCAMSolution(
                    solved=False,
                    comp_time_sec=0.0,
                    paths_by_agent={},
                    raw_status="repair_failed",
                    notes=(
                        "Conflict repair failed for "
                        f"robots=({robot_i}, {robot_j})"
                    ),
                )

            paths = repaired

        return LaCAMSolution(
            solved=False,
            comp_time_sec=0.0,
            paths_by_agent={},
            raw_status="max_repair_iterations_exceeded",
            notes=(
                f"Exceeded max_repair_iterations={self.max_repair_iterations}"
            ),
        )

    def _build_initial_solution(
        self,
        problem: LaCAMProblem,
    ) -> Optional[Dict[int, List[int]]]:
        """各ロボットを独立に解いて初期解を作る。"""
        paths: Dict[int, List[int]] = {}

        for robot_id in sorted(problem.start_state_ids.keys()):
            path = self._plan_single_agent_path(
                problem=problem,
                robot_id=robot_id,
                fixed_paths={},
            )
            if path is None:
                return None
            paths[robot_id] = path

        return paths

    def _plan_single_agent_path(
        self,
        problem: LaCAMProblem,
        robot_id: int,
        fixed_paths: Dict[int, List[int]],
    ) -> Optional[List[int]]:
        """単一ロボットの時間拡張A*を実行する。"""
        reservation = self._build_reservation_table(
            problem=problem,
            fixed_paths=fixed_paths,
        )

        graph = problem.graph
        start_state_id = problem.start_state_ids[robot_id]
        goal_state_id = problem.goal_state_ids[robot_id]

        open_heap: List[SearchNode] = []
        g_costs: Dict[Tuple[int, int], float] = {}
        parents: Dict[Tuple[int, int], Tuple[int, int] | None] = {}

        start_h = self._heuristic(
            graph=graph,
            state_id=start_state_id,
            goal_state_id=goal_state_id,
        )
        start_key = (start_state_id, 0)
        g_costs[start_key] = 0.0
        parents[start_key] = None
        heapq.heappush(
            open_heap,
            SearchNode(
                priority=start_h,
                g_cost=0.0,
                state_id=start_state_id,
                time_step=0,
            ),
        )

        best_goal_key: Optional[Tuple[int, int]] = None

        while open_heap:
            current = heapq.heappop(open_heap)
            current_key = (current.state_id, current.time_step)

            if current.g_cost > g_costs.get(current_key, float("inf")):
                continue

            if (
                graph.is_goal(current.state_id, goal_state_id)
                and not self._violates_goal_hold(
                    problem=problem,
                    state_id=current.state_id,
                    time_step=current.time_step,
                    reservation=reservation,
                )
            ):
                best_goal_key = current_key
                break

            if current.time_step >= self.max_time_expansion:
                continue

            for next_state_id, edge in graph.neighbors(current.state_id):
                next_time = current.time_step + 1

                if self._transition_conflicts_with_reservations(
                    problem=problem,
                    src_state_id=current.state_id,
                    dst_state_id=next_state_id,
                    time_step=current.time_step,
                    reservation=reservation,
                ):
                    continue

                next_g = current.g_cost + edge.cost
                next_key = (next_state_id, next_time)
                if next_g >= g_costs.get(next_key, float("inf")):
                    continue

                g_costs[next_key] = next_g
                parents[next_key] = current_key
                h = self._heuristic(
                    graph=graph,
                    state_id=next_state_id,
                    goal_state_id=goal_state_id,
                )
                heapq.heappush(
                    open_heap,
                    SearchNode(
                        priority=next_g + h,
                        g_cost=next_g,
                        state_id=next_state_id,
                        time_step=next_time,
                    ),
                )

        if best_goal_key is None:
            return None

        return self._reconstruct_path(
            goal_key=best_goal_key,
            parents=parents,
        )

    def _build_reservation_table(
        self,
        problem: LaCAMProblem,
        fixed_paths: Dict[int, List[int]],
    ) -> ReservationTable:
        """固定済み経路群から予約表を作る。"""
        graph = problem.graph
        state_res: Dict[int, List[FrozenSet[tuple[int, int]]]] = {}
        trans_res: Dict[int, List[FrozenSet[tuple[int, int]]]] = {}
        final_time_by_agent: Dict[int, int] = {}

        for robot_id, state_ids in fixed_paths.items():
            final_time_by_agent[robot_id] = len(state_ids) - 1

            for time_step in range(len(state_ids)):
                state_id = state_ids[min(time_step, len(state_ids) - 1)]
                oriented_state = graph.id_to_state(state_id)
                node, mode = graph.oriented_state_to_internal(oriented_state)
                occ = graph.env.state_occupancy_key(node=node, mode=mode)
                state_res.setdefault(time_step, []).append(occ)

            for time_step in range(len(state_ids) - 1):
                src = graph.id_to_state(state_ids[time_step])
                dst = graph.id_to_state(state_ids[time_step + 1])

                src_node, src_mode = graph.oriented_state_to_internal(src)
                dst_node, dst_mode = graph.oriented_state_to_internal(dst)
                env_cells = graph.env.transition_envelope(
                    from_node=src_node,
                    from_mode=src_mode,
                    to_node=dst_node,
                    to_mode=dst_mode,
                )
                trans_res.setdefault(time_step, []).append(env_cells)

            final_state = graph.id_to_state(state_ids[-1])
            final_node, final_mode = graph.oriented_state_to_internal(final_state)
            final_env = graph.env.transition_envelope(
                from_node=final_node,
                from_mode=final_mode,
                to_node=final_node,
                to_mode=final_mode,
            )
            for time_step in range(len(state_ids) - 1, self.max_time_expansion):
                trans_res.setdefault(time_step, []).append(final_env)

        return ReservationTable(
            state_reservations=state_res,
            transition_reservations=trans_res,
            final_time_by_agent=final_time_by_agent,
        )

    def _detect_first_conflict(
        self,
        problem: LaCAMProblem,
        paths_by_agent: Dict[int, List[int]],
    ) -> Optional[Tuple[int, int]]:
        """最初の競合ロボット対を返す。"""
        graph = problem.graph
        robot_ids = sorted(paths_by_agent.keys())
        max_len = max(len(path) for path in paths_by_agent.values())

        for time_step in range(max_len):
            for idx_i in range(len(robot_ids)):
                for idx_j in range(idx_i + 1, len(robot_ids)):
                    robot_i = robot_ids[idx_i]
                    robot_j = robot_ids[idx_j]

                    if self._pair_conflicts(
                        problem=problem,
                        paths_by_agent=paths_by_agent,
                        robot_i=robot_i,
                        robot_j=robot_j,
                        time_step=time_step,
                    ):
                        return robot_i, robot_j

        return None

    def _pair_conflicts(
        self,
        problem: LaCAMProblem,
        paths_by_agent: Dict[int, List[int]],
        robot_i: int,
        robot_j: int,
        time_step: int,
    ) -> bool:
        """ロボット対が特定時刻で競合するか判定する。"""
        graph = problem.graph

        state_i = graph.id_to_state(
            self._state_id_at_time(paths_by_agent[robot_i], time_step)
        )
        state_j = graph.id_to_state(
            self._state_id_at_time(paths_by_agent[robot_j], time_step)
        )

        node_i, mode_i = graph.oriented_state_to_internal(state_i)
        node_j, mode_j = graph.oriented_state_to_internal(state_j)

        occ_i = graph.env.state_occupancy_key(node_i, mode_i)
        occ_j = graph.env.state_occupancy_key(node_j, mode_j)
        if occ_i & occ_j:
            return True

        src_i = graph.id_to_state(
            self._state_id_at_time(paths_by_agent[robot_i], time_step)
        )
        dst_i = graph.id_to_state(
            self._state_id_at_time(paths_by_agent[robot_i], time_step + 1)
        )
        src_j = graph.id_to_state(
            self._state_id_at_time(paths_by_agent[robot_j], time_step)
        )
        dst_j = graph.id_to_state(
            self._state_id_at_time(paths_by_agent[robot_j], time_step + 1)
        )

        src_i_node, src_i_mode = graph.oriented_state_to_internal(src_i)
        dst_i_node, dst_i_mode = graph.oriented_state_to_internal(dst_i)
        src_j_node, src_j_mode = graph.oriented_state_to_internal(src_j)
        dst_j_node, dst_j_mode = graph.oriented_state_to_internal(dst_j)

        env_i = graph.env.transition_envelope(
            from_node=src_i_node,
            from_mode=src_i_mode,
            to_node=dst_i_node,
            to_mode=dst_i_mode,
        )
        env_j = graph.env.transition_envelope(
            from_node=src_j_node,
            from_mode=src_j_mode,
            to_node=dst_j_node,
            to_mode=dst_j_mode,
        )
        return bool(env_i & env_j)

    def _count_conflicts(
        self,
        problem: LaCAMProblem,
        paths_by_agent: Dict[int, List[int]],
    ) -> int:
        """経路群全体に含まれる競合数を数える。"""
        robot_ids = sorted(paths_by_agent.keys())
        max_len = max(len(path) for path in paths_by_agent.values())
        num_conflicts = 0

        for time_step in range(max_len):
            for idx_i in range(len(robot_ids)):
                for idx_j in range(idx_i + 1, len(robot_ids)):
                    robot_i = robot_ids[idx_i]
                    robot_j = robot_ids[idx_j]
                    if self._pair_conflicts(
                        problem=problem,
                        paths_by_agent=paths_by_agent,
                        robot_i=robot_i,
                        robot_j=robot_j,
                        time_step=time_step,
                    ):
                        num_conflicts += 1

        return num_conflicts

    def _compute_path_metrics(
        self,
        problem: LaCAMProblem,
        paths_by_agent: Dict[int, List[int]],
    ) -> Tuple[float, float]:
        """候補経路群の makespan と sum_of_costs を返す。"""
        graph = problem.graph
        robot_paths = {
            robot_id: graph.path_state_ids_to_robot_path(state_ids)
            for robot_id, state_ids in paths_by_agent.items()
        }
        makespan = max(path.cost for path in robot_paths.values())
        sum_of_costs = sum(path.cost for path in robot_paths.values())
        return makespan, sum_of_costs

    def _evaluate_repair_candidate(
        self,
        problem: LaCAMProblem,
        paths_by_agent: Dict[int, List[int]],
        repaired_agents: Tuple[int, ...],
        notes: str,
    ) -> RepairCandidate:
        """候補を評価して `RepairCandidate` を返す。"""
        num_conflicts = self._count_conflicts(
            problem=problem,
            paths_by_agent=paths_by_agent,
        )
        makespan, sum_of_costs = self._compute_path_metrics(
            problem=problem,
            paths_by_agent=paths_by_agent,
        )
        score = 1000.0 * num_conflicts + 10.0 * makespan + sum_of_costs

        return RepairCandidate(
            paths_by_agent=paths_by_agent,
            repaired_agents=repaired_agents,
            num_conflicts=num_conflicts,
            makespan=makespan,
            sum_of_costs=sum_of_costs,
            score=score,
            notes=notes,
        )

    def _generate_single_agent_repair_candidates(
        self,
        problem: LaCAMProblem,
        paths_by_agent: Dict[int, List[int]],
        robot_id: int,
    ) -> List[RepairCandidate]:
        """1台だけ再計画した候補を生成する。"""
        fixed_paths = {
            rid: path
            for rid, path in paths_by_agent.items()
            if rid != robot_id
        }
        repaired_path = self._plan_single_agent_path(
            problem=problem,
            robot_id=robot_id,
            fixed_paths=fixed_paths,
        )
        if repaired_path is None:
            return []

        candidate_paths = dict(paths_by_agent)
        candidate_paths[robot_id] = repaired_path

        return [
            self._evaluate_repair_candidate(
                problem=problem,
                paths_by_agent=candidate_paths,
                repaired_agents=(robot_id,),
                notes=f"single_agent_repair: robot={robot_id}",
            )
        ]

    def _generate_two_agent_repair_candidates(
        self,
        problem: LaCAMProblem,
        paths_by_agent: Dict[int, List[int]],
        robot_i: int,
        robot_j: int,
    ) -> List[RepairCandidate]:
        """2台順次再計画した候補を生成する。"""
        candidates: List[RepairCandidate] = []

        for first_robot, second_robot in ((robot_i, robot_j), (robot_j, robot_i)):
            fixed_except_first = {
                rid: path
                for rid, path in paths_by_agent.items()
                if rid != first_robot
            }
            new_first = self._plan_single_agent_path(
                problem=problem,
                robot_id=first_robot,
                fixed_paths=fixed_except_first,
            )
            if new_first is None:
                continue

            temp_paths = dict(paths_by_agent)
            temp_paths[first_robot] = new_first

            fixed_except_second = {
                rid: path
                for rid, path in temp_paths.items()
                if rid != second_robot
            }
            new_second = self._plan_single_agent_path(
                problem=problem,
                robot_id=second_robot,
                fixed_paths=fixed_except_second,
            )
            if new_second is None:
                continue

            temp_paths[second_robot] = new_second

            candidates.append(
                self._evaluate_repair_candidate(
                    problem=problem,
                    paths_by_agent=temp_paths,
                    repaired_agents=(first_robot, second_robot),
                    notes=(
                        "two_agent_repair: "
                        f"{first_robot}->{second_robot}"
                    ),
                )
            )

        return candidates

    def _select_best_repair_candidate(
        self,
        candidates: List[RepairCandidate],
    ) -> Optional[RepairCandidate]:
        """候補群の中から最良候補を返す。"""
        if not candidates:
            return None

        return min(
            candidates,
            key=lambda c: (
                c.score,
                c.num_conflicts,
                c.makespan,
                c.sum_of_costs,
            ),
        )

    def _repair_conflicted_agents(
        self,
        problem: LaCAMProblem,
        paths_by_agent: Dict[int, List[int]],
        robot_i: int,
        robot_j: int,
    ) -> Optional[Dict[int, List[int]]]:
        """競合ロボット対の repair 候補を生成・比較して採用する。"""
        current_num_conflicts = self._count_conflicts(
            problem=problem,
            paths_by_agent=paths_by_agent,
        )

        candidates: List[RepairCandidate] = []
        candidates.extend(
            self._generate_single_agent_repair_candidates(
                problem=problem,
                paths_by_agent=paths_by_agent,
                robot_id=robot_i,
            )
        )
        candidates.extend(
            self._generate_single_agent_repair_candidates(
                problem=problem,
                paths_by_agent=paths_by_agent,
                robot_id=robot_j,
            )
        )
        candidates.extend(
            self._generate_two_agent_repair_candidates(
                problem=problem,
                paths_by_agent=paths_by_agent,
                robot_i=robot_i,
                robot_j=robot_j,
            )
        )

        best_candidate = self._select_best_repair_candidate(candidates)
        if best_candidate is None:
            return None

        if best_candidate.num_conflicts <= current_num_conflicts:
            return best_candidate.paths_by_agent

        return None

    def _heuristic(
        self,
        graph,
        state_id: int,
        goal_state_id: int,
    ) -> float:
        """単純ヒューリスティックを返す。"""
        state = graph.id_to_state(state_id)
        goal = graph.id_to_state(goal_state_id)

        manhattan = abs(state.row - goal.row) + abs(state.col - goal.col)
        rotate_penalty = 0.0 if state.mode_deg == goal.mode_deg else 0.15
        return float(manhattan) + rotate_penalty

    def _state_id_at_time(
        self,
        path: List[int],
        time_step: int,
    ) -> int:
        """終端保持込みで指定時刻の状態IDを返す。"""
        return path[min(time_step, len(path) - 1)]

    def _transition_conflicts_with_reservations(
        self,
        problem: LaCAMProblem,
        src_state_id: int,
        dst_state_id: int,
        time_step: int,
        reservation: ReservationTable,
    ) -> bool:
        """候補遷移が予約表と衝突するか判定する。"""
        graph = problem.graph
        src = graph.id_to_state(src_state_id)
        dst = graph.id_to_state(dst_state_id)

        src_node, src_mode = graph.oriented_state_to_internal(src)
        dst_node, dst_mode = graph.oriented_state_to_internal(dst)

        next_occ = graph.env.state_occupancy_key(dst_node, dst_mode)
        for reserved_occ in reservation.state_reservations.get(time_step + 1, []):
            if next_occ & reserved_occ:
                return True

        transition_env = graph.env.transition_envelope(
            from_node=src_node,
            from_mode=src_mode,
            to_node=dst_node,
            to_mode=dst_mode,
        )
        for reserved_env in reservation.transition_reservations.get(time_step, []):
            if transition_env & reserved_env:
                return True

        return False

    def _violates_goal_hold(
        self,
        problem: LaCAMProblem,
        state_id: int,
        time_step: int,
        reservation: ReservationTable,
    ) -> bool:
        """ゴール到達後にその場保持できるか確認する。"""
        graph = problem.graph
        state = graph.id_to_state(state_id)
        node, mode = graph.oriented_state_to_internal(state)

        hold_env = graph.env.transition_envelope(
            from_node=node,
            from_mode=mode,
            to_node=node,
            to_mode=mode,
        )
        hold_occ = graph.env.state_occupancy_key(node, mode)

        for t in range(time_step, self.max_time_expansion):
            for reserved_occ in reservation.state_reservations.get(t, []):
                if hold_occ & reserved_occ:
                    return True
            for reserved_env in reservation.transition_reservations.get(t, []):
                if hold_env & reserved_env:
                    return True

        return False

    def _reconstruct_path(
        self,
        goal_key: Tuple[int, int],
        parents: Dict[Tuple[int, int], Tuple[int, int] | None],
    ) -> List[int]:
        """親ポインタから状態ID列を復元する。"""
        state_ids: List[int] = []
        current: Tuple[int, int] | None = goal_key

        while current is not None:
            state_id, _ = current
            state_ids.append(state_id)
            current = parents[current]

        state_ids.reverse()
        return state_ids