from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from benchmark_common import (
    BenchmarkCaseSpec,
    BenchmarkResult,
    build_environment,
    parse_states,
    summarize_solution,
)
from mapf_types import Path as RobotPath
from orientation_graph import OrientationGraph, OrientedProblem, build_oriented_problem


@dataclass(frozen=True)
class LaCAMProblem:
    """LaCAM に渡す問題定義。

    Attributes:
        graph: 姿勢付き状態グラフ。
        start_state_ids: ロボットID -> 開始状態ID。
        goal_state_ids: ロボットID -> 目標状態ID。
        num_agents: エージェント数。
        metadata: 補助情報。
    """

    graph: OrientationGraph
    start_state_ids: Dict[int, int]
    goal_state_ids: Dict[int, int]
    num_agents: int
    metadata: Dict[str, str]


@dataclass(frozen=True)
class LaCAMSolution:
    """LaCAM 側から返ってくる解表現。

    Attributes:
        solved: 解けたかどうか。
        comp_time_sec: 計算時間 [sec]。
        paths_by_agent: ロボットID -> 状態ID列。
        raw_status: ソルバ生ステータス。
        notes: 補助メモ。
    """

    solved: bool
    comp_time_sec: float
    paths_by_agent: Dict[int, List[int]]
    raw_status: str
    notes: str = ""


def build_lacam_problem(
    case: BenchmarkCaseSpec,
    enable_move_rotate: bool,
) -> LaCAMProblem:
    """ベンチマークケースから LaCAM 問題を構築する。

    Args:
        case: ベンチマークケース。
        enable_move_rotate: move+rotate を許可するか。

    Returns:
        LaCAMProblem。
    """
    env = build_environment(
        case=case,
        enable_move_rotate=enable_move_rotate,
    )
    starts = parse_states(case.starts)
    goals = parse_states(case.goals)

    oriented_problem: OrientedProblem = build_oriented_problem(
        env=env,
        starts=starts,
        goals=goals,
    )

    return LaCAMProblem(
        graph=oriented_problem.graph,
        start_state_ids=oriented_problem.start_state_ids,
        goal_state_ids=oriented_problem.goal_state_ids,
        num_agents=len(oriented_problem.start_state_ids),
        metadata={
            "case_id": case.case_id,
            "case_name": case.case_name,
            "purpose": case.purpose,
            "tier": case.tier,
            "enable_move_rotate": str(enable_move_rotate),
        },
    )


def validate_lacam_solution(
    problem: LaCAMProblem,
    solution: LaCAMSolution,
) -> None:
    """LaCAM 解の基本妥当性を検証する。

    Args:
        problem: LaCAM 問題。
        solution: LaCAM 解。

    Raises:
        ValueError: 解の形式が不正な場合。
    """
    if not solution.solved:
        return

    missing_agents = set(problem.start_state_ids.keys()) - set(
        solution.paths_by_agent.keys()
    )
    if missing_agents:
        raise ValueError(
            f"Missing paths for agents: {sorted(missing_agents)}"
        )

    for robot_id, state_ids in solution.paths_by_agent.items():
        if not state_ids:
            raise ValueError(
                f"Empty state sequence for robot_id={robot_id}"
            )

        start_state_id = problem.start_state_ids[robot_id]
        goal_state_id = problem.goal_state_ids[robot_id]

        if state_ids[0] != start_state_id:
            raise ValueError(
                "LaCAM solution start mismatch "
                f"for robot_id={robot_id}: "
                f"{state_ids[0]} != {start_state_id}"
            )

        if state_ids[-1] != goal_state_id:
            raise ValueError(
                "LaCAM solution goal mismatch "
                f"for robot_id={robot_id}: "
                f"{state_ids[-1]} != {goal_state_id}"
            )

        # 経路連続性確認
        for idx in range(len(state_ids) - 1):
            _ = problem.graph.edge_from_state_ids(
                src_state_id=state_ids[idx],
                dst_state_id=state_ids[idx + 1],
            )


def lacam_solution_to_robot_paths(
    problem: LaCAMProblem,
    solution: LaCAMSolution,
) -> Dict[int, RobotPath]:
    """LaCAM の状態ID列を `RobotPath` 群へ復元する。

    Args:
        problem: LaCAM 問題。
        solution: LaCAM 解。

    Returns:
        ロボットID -> RobotPath。

    Raises:
        ValueError: solution が solved=False の場合。
    """
    if not solution.solved:
        raise ValueError("Cannot convert unsolved LaCAM solution")

    validate_lacam_solution(problem=problem, solution=solution)

    robot_paths: Dict[int, RobotPath] = {}
    for robot_id, state_ids in sorted(solution.paths_by_agent.items()):
        robot_paths[robot_id] = problem.graph.path_state_ids_to_robot_path(
            state_ids=state_ids
        )
    return robot_paths


def run_single_case_lacam_result(
    case: BenchmarkCaseSpec,
    enable_move_rotate: bool,
    planner_name: str,
    lacam_solution: LaCAMSolution,
) -> BenchmarkResult:
    """LaCAM 解から `BenchmarkResult` を構築する。

    Args:
        case: ベンチマークケース。
        enable_move_rotate: move+rotate 許可フラグ。
        planner_name: プランナ名。
        lacam_solution: LaCAM 解。

    Returns:
        `BenchmarkResult`。
    """
    if not lacam_solution.solved:
        return BenchmarkResult(
            planner_name=planner_name,
            case_id=case.case_id,
            case_name=case.case_name,
            purpose=case.purpose,
            tier=case.tier,
            enable_move_rotate=enable_move_rotate,
            success=False,
            planning_time_sec=lacam_solution.comp_time_sec,
            num_robots=len(case.starts),
            makespan=None,
            sum_of_costs=None,
            total_wait=None,
            total_move=None,
            total_rotate=None,
            total_move_rotate=None,
            per_robot=None,
            notes=lacam_solution.raw_status
            + (f" | {lacam_solution.notes}" if lacam_solution.notes else ""),
        )

    problem = build_lacam_problem(
        case=case,
        enable_move_rotate=enable_move_rotate,
    )
    robot_paths = lacam_solution_to_robot_paths(
        problem=problem,
        solution=lacam_solution,
    )

    result = summarize_solution(
        paths=robot_paths,
        planning_time_sec=lacam_solution.comp_time_sec,
        planner_name=planner_name,
        case=case,
        enable_move_rotate=enable_move_rotate,
    )

    notes = lacam_solution.raw_status
    if lacam_solution.notes:
        notes = notes + f" | {lacam_solution.notes}"

    return BenchmarkResult(
        planner_name=result.planner_name,
        case_id=result.case_id,
        case_name=result.case_name,
        purpose=result.purpose,
        tier=result.tier,
        enable_move_rotate=result.enable_move_rotate,
        success=result.success,
        planning_time_sec=result.planning_time_sec,
        num_robots=result.num_robots,
        makespan=result.makespan,
        sum_of_costs=result.sum_of_costs,
        total_wait=result.total_wait,
        total_move=result.total_move,
        total_rotate=result.total_rotate,
        total_move_rotate=result.total_move_rotate,
        per_robot=result.per_robot,
        notes=notes,
    )


def build_problem_and_metadata(
    case: BenchmarkCaseSpec,
    enable_move_rotate: bool,
) -> tuple[LaCAMProblem, Dict[str, str]]:
    """問題と補助メタデータをまとめて返す。

    Args:
        case: ベンチマークケース。
        enable_move_rotate: move+rotate 許可フラグ。

    Returns:
        (LaCAMProblem, metadata)
    """
    problem = build_lacam_problem(
        case=case,
        enable_move_rotate=enable_move_rotate,
    )
    metadata = {
        "planner_family": "LaCAM",
        "case_id": case.case_id,
        "case_name": case.case_name,
        "purpose": case.purpose,
        "tier": case.tier,
        "enable_move_rotate": str(enable_move_rotate),
        "num_agents": str(problem.num_agents),
    }
    return problem, metadata


def make_dummy_solution_from_state_ids(
    paths_by_agent: Dict[int, List[int]],
    comp_time_sec: float = 0.0,
    raw_status: str = "dummy",
    notes: str = "",
) -> LaCAMSolution:
    """状態ID列からテスト用ダミー解を構築する。

    Args:
        paths_by_agent: ロボットID -> 状態ID列。
        comp_time_sec: 計算時間。
        raw_status: ステータス文字列。
        notes: 補助メモ。

    Returns:
        `LaCAMSolution`。
    """
    return LaCAMSolution(
        solved=True,
        comp_time_sec=comp_time_sec,
        paths_by_agent=paths_by_agent,
        raw_status=raw_status,
        notes=notes,
    )


def make_failed_solution(
    comp_time_sec: float,
    raw_status: str,
    notes: str = "",
) -> LaCAMSolution:
    """失敗解を構築する。

    Args:
        comp_time_sec: 計算時間。
        raw_status: ステータス文字列。
        notes: 補助メモ。

    Returns:
        `LaCAMSolution`。
    """
    return LaCAMSolution(
        solved=False,
        comp_time_sec=comp_time_sec,
        paths_by_agent={},
        raw_status=raw_status,
        notes=notes,
    )