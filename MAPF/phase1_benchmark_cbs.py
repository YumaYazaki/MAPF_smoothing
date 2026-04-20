from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Tuple

from cbs import cbs_plan
from environment import PlanningEnvironment
from mapf_types import ActionType, GridNode, OrientationMode, Path as RobotPath
from mapf_types import RobotGeometrySpec


RESULT_DIR = Path("MAPF/results/phase1")


@dataclass(frozen=True)
class BenchmarkCaseSpec:
    """ベンチマークケース仕様。

    Attributes:
        case_id: ケース識別子。
        case_name: ケース名。
        purpose: ケースの狙い。
        rows: グリッド行数。
        cols: グリッド列数。
        obstacles: 障害物セル一覧。
        starts: ロボットID -> (row, col, deg)。
        goals: ロボットID -> (row, col, deg)。
        max_time: 最大探索時間。
        tier: ケース分類。
    """

    case_id: str
    case_name: str
    purpose: str
    rows: int
    cols: int
    obstacles: List[Tuple[int, int]]
    starts: Dict[int, Tuple[int, int, int]]
    goals: Dict[int, Tuple[int, int, int]]
    max_time: int
    tier: str


@dataclass(frozen=True)
class RobotMetrics:
    """ロボット単体のメトリクス。"""

    cost: float
    num_states: int
    num_transitions: int
    num_wait: int
    num_move: int
    num_rotate: int
    num_move_rotate: int


@dataclass(frozen=True)
class BenchmarkResult:
    """単一ケース・単一設定の実験結果。"""

    planner_name: str
    case_id: str
    case_name: str
    purpose: str
    tier: str
    enable_move_rotate: bool
    success: bool
    planning_time_sec: float
    num_robots: int
    makespan: float | None
    sum_of_costs: float | None
    total_wait: int | None
    total_move: int | None
    total_rotate: int | None
    total_move_rotate: int | None
    per_robot: Dict[int, RobotMetrics] | None
    notes: str


def build_default_modes() -> list[OrientationMode]:
    """初期PoC用の4姿勢モードを返す。"""
    return [
        OrientationMode(0),
        OrientationMode(90),
        OrientationMode(180),
        OrientationMode(270),
    ]


def build_default_robot_geometry() -> RobotGeometrySpec:
    """初期PoC用ロボット形状を返す。"""
    return RobotGeometrySpec(
        fine_scale=4,
        occupied_cells_at_zero_deg=frozenset(
            {
                (0, 0),  # ムーバー本体
                (0, 1),
                (0, 2),
                (0, 3),  # アーム
                (0, 4),  # 搬送物
            }
        ),
    )


def build_environment(
    case: BenchmarkCaseSpec,
    enable_move_rotate: bool,
) -> PlanningEnvironment:
    """ケース仕様から計画環境を構築する。

    Args:
        case: ケース仕様。
        enable_move_rotate: move+rotate を許可するか。

    Returns:
        計画環境。
    """
    return PlanningEnvironment(
        rows=case.rows,
        cols=case.cols,
        obstacles=set(case.obstacles),
        modes=build_default_modes(),
        robot_geometry=build_default_robot_geometry(),
        enable_diagonal_move=False,
        enable_move_rotate=enable_move_rotate,
        enable_reserved_regions=False,
    )


def parse_states(
    state_dict: Dict[int, Tuple[int, int, int]],
) -> Dict[int, Tuple[GridNode, OrientationMode]]:
    """シリアライズ用状態辞書を内部表現へ変換する。"""
    parsed: Dict[int, Tuple[GridNode, OrientationMode]] = {}
    for robot_id, (row, col, deg) in state_dict.items():
        parsed[robot_id] = (GridNode(row=row, col=col), OrientationMode(deg))
    return parsed


def count_actions(
    path: RobotPath,
) -> Dict[str, int]:
    """経路中のアクション数を集計する。"""
    counts = {
        "wait": 0,
        "move": 0,
        "rotate": 0,
        "move_rotate": 0,
    }

    for transition in path.transitions:
        if transition.action_type == ActionType.WAIT:
            counts["wait"] += 1
        elif transition.action_type == ActionType.MOVE:
            counts["move"] += 1
        elif transition.action_type == ActionType.ROTATE:
            counts["rotate"] += 1
        elif transition.action_type == ActionType.MOVE_ROTATE:
            counts["move_rotate"] += 1
        else:
            raise ValueError(
                f"Unsupported action type: {transition.action_type}"
            )

    return counts


def summarize_solution(
    paths: Dict[int, RobotPath],
    planning_time_sec: float,
    planner_name: str,
    case: BenchmarkCaseSpec,
    enable_move_rotate: bool,
) -> BenchmarkResult:
    """解全体のメトリクスを集計する。"""
    per_robot: Dict[int, RobotMetrics] = {}
    sum_of_costs = 0.0
    makespan = 0.0

    total_wait = 0
    total_move = 0
    total_rotate = 0
    total_move_rotate = 0

    for robot_id, path in sorted(paths.items()):
        action_counts = count_actions(path)
        sum_of_costs += path.cost
        makespan = max(makespan, path.cost)

        total_wait += action_counts["wait"]
        total_move += action_counts["move"]
        total_rotate += action_counts["rotate"]
        total_move_rotate += action_counts["move_rotate"]

        per_robot[robot_id] = RobotMetrics(
            cost=path.cost,
            num_states=len(path.states),
            num_transitions=len(path.transitions),
            num_wait=action_counts["wait"],
            num_move=action_counts["move"],
            num_rotate=action_counts["rotate"],
            num_move_rotate=action_counts["move_rotate"],
        )

    return BenchmarkResult(
        planner_name=planner_name,
        case_id=case.case_id,
        case_name=case.case_name,
        purpose=case.purpose,
        tier=case.tier,
        enable_move_rotate=enable_move_rotate,
        success=True,
        planning_time_sec=planning_time_sec,
        num_robots=len(paths),
        makespan=makespan,
        sum_of_costs=sum_of_costs,
        total_wait=total_wait,
        total_move=total_move,
        total_rotate=total_rotate,
        total_move_rotate=total_move_rotate,
        per_robot=per_robot,
        notes="",
    )


def run_single_case(
    case: BenchmarkCaseSpec,
    enable_move_rotate: bool,
    planner_name: str = "CBS+A*",
    debug: bool = False,
) -> BenchmarkResult:
    """単一ケースを実行し、結果を返す。

    Args:
        case: ベンチマークケース。
        enable_move_rotate: move+rotate を許可するか。
        planner_name: プランナ名。
        debug: デバッグ出力有効化フラグ。

    Returns:
        実験結果。
    """
    import time

    env = build_environment(case=case, enable_move_rotate=enable_move_rotate)
    starts = parse_states(case.starts)
    goals = parse_states(case.goals)

    start_time = time.perf_counter()
    solution = cbs_plan(
        starts=starts,
        goals=goals,
        env=env,
        max_time=case.max_time,
        debug=debug,
    )
    planning_time_sec = time.perf_counter() - start_time

    if solution is None:
        return BenchmarkResult(
            planner_name=planner_name,
            case_id=case.case_id,
            case_name=case.case_name,
            purpose=case.purpose,
            tier=case.tier,
            enable_move_rotate=enable_move_rotate,
            success=False,
            planning_time_sec=planning_time_sec,
            num_robots=len(case.starts),
            makespan=None,
            sum_of_costs=None,
            total_wait=None,
            total_move=None,
            total_rotate=None,
            total_move_rotate=None,
            per_robot=None,
            notes="No solution found",
        )

    return summarize_solution(
        paths=solution,
        planning_time_sec=planning_time_sec,
        planner_name=planner_name,
        case=case,
        enable_move_rotate=enable_move_rotate,
    )


def get_phase1_benchmark_cases() -> list[BenchmarkCaseSpec]:
    """Phase 1 で固定するベンチマークケース一覧を返す。"""
    return [
        BenchmarkCaseSpec(
            case_id="case_01",
            case_name="Case 01: rotation only",
            purpose="回転単体確認",
            rows=5,
            cols=7,
            obstacles=[],
            starts={0: (2, 2, 0)},
            goals={0: (2, 2, 180)},
            max_time=10,
            tier="Tier 1: primitive sanity",
        ),
        BenchmarkCaseSpec(
            case_id="case_02",
            case_name="Case 02: move+rotate",
            purpose="move+rotate 単体効果",
            rows=5,
            cols=7,
            obstacles=[],
            starts={0: (3, 1, 0)},
            goals={0: (2, 2, 90)},
            max_time=10,
            tier="Tier 1: primitive sanity",
        ),
        BenchmarkCaseSpec(
            case_id="case_03",
            case_name="Case 03: two robot corridor",
            purpose="単純な2台対向協調",
            rows=5,
            cols=7,
            obstacles=[],
            starts={
                0: (2, 1, 0),
                1: (2, 5, 180),
            },
            goals={
                0: (2, 5, 0),
                1: (2, 1, 180),
            },
            max_time=20,
            tier="Tier 2: simple coordination",
        ),
        BenchmarkCaseSpec(
            case_id="case_04",
            case_name="Case 04: two robot with obstacle wall",
            purpose="障害物あり協調",
            rows=5,
            cols=7,
            obstacles=[(1, 3), (2, 3), (3, 3)],
            starts={
                0: (0, 1, 0),
                1: (4, 5, 180),
            },
            goals={
                0: (4, 5, 90),
                1: (0, 1, 270),
            },
            max_time=30,
            tier="Tier 3: obstacle helpful",
        ),
        BenchmarkCaseSpec(
            case_id="case_a",
            case_name="Case A: move+rotate helpful",
            purpose="move+rotate 有効例",
            rows=5,
            cols=5,
            obstacles=[],
            starts={0: (4, 1, 0)},
            goals={0: (1, 4, 90)},
            max_time=20,
            tier="Tier 3: obstacle helpful",
        ),
        BenchmarkCaseSpec(
            case_id="case_b",
            case_name="Case B: move+rotate likely unhelpful",
            purpose="move+rotate 不要例",
            rows=5,
            cols=9,
            obstacles=[],
            starts={
                0: (2, 1, 0),
                1: (0, 7, 180),
            },
            goals={
                0: (2, 7, 0),
                1: (0, 1, 180),
            },
            max_time=25,
            tier="Tier 2: simple coordination",
        ),
        BenchmarkCaseSpec(
            case_id="case_c",
            case_name="Case C: coordination sensitive",
            purpose="干渉が強い協調ケース",
            rows=5,
            cols=7,
            obstacles=[(0, 3), (1, 3), (3, 3), (4, 3)],
            starts={
                0: (2, 1, 0),
                1: (1, 5, 180),
            },
            goals={
                0: (2, 5, 0),
                1: (3, 1, 180),
            },
            max_time=30,
            tier="Tier 4: hard coordination",
        ),
    ]


def benchmark_result_to_flat_row(
    result: BenchmarkResult,
) -> Dict[str, Any]:
    """CSV 出力用に結果を平坦化する。"""
    row: Dict[str, Any] = {
        "planner_name": result.planner_name,
        "case_id": result.case_id,
        "case_name": result.case_name,
        "purpose": result.purpose,
        "tier": result.tier,
        "enable_move_rotate": result.enable_move_rotate,
        "success": result.success,
        "planning_time_sec": result.planning_time_sec,
        "num_robots": result.num_robots,
        "makespan": result.makespan,
        "sum_of_costs": result.sum_of_costs,
        "total_wait": result.total_wait,
        "total_move": result.total_move,
        "total_rotate": result.total_rotate,
        "total_move_rotate": result.total_move_rotate,
        "notes": result.notes,
    }

    if result.per_robot is not None:
        for robot_id, metrics in sorted(result.per_robot.items()):
            row[f"R{robot_id}_cost"] = metrics.cost
            row[f"R{robot_id}_num_states"] = metrics.num_states
            row[f"R{robot_id}_num_transitions"] = metrics.num_transitions
            row[f"R{robot_id}_wait"] = metrics.num_wait
            row[f"R{robot_id}_move"] = metrics.num_move
            row[f"R{robot_id}_rotate"] = metrics.num_rotate
            row[f"R{robot_id}_move_rotate"] = metrics.num_move_rotate

    return row


def save_json(
    filepath: Path,
    payload: Any,
) -> None:
    """JSON を保存する。"""
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with filepath.open("w", encoding="utf-8") as fp:
        json.dump(payload, fp, ensure_ascii=False, indent=2)


def save_csv(
    filepath: Path,
    rows: list[Dict[str, Any]],
) -> None:
    """CSV を保存する。"""
    filepath.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError("rows must not be empty")

    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)

    with filepath.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_markdown_summary(
    filepath: Path,
    cases: list[BenchmarkCaseSpec],
    results: list[BenchmarkResult],
) -> None:
    """簡易Markdownサマリを保存する。"""
    filepath.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    lines.append("# Phase 1 Benchmark Summary")
    lines.append("")
    lines.append("## Benchmark Cases")
    lines.append("")
    lines.append("| Case | Purpose | Tier |")
    lines.append("|---|---|---|")
    for case in cases:
        lines.append(
            f"| {case.case_name} | {case.purpose} | {case.tier} |"
        )

    lines.append("")
    lines.append("## Baseline Results (CBS+A*)")
    lines.append("")
    lines.append(
        "| Case | move+rotate | success | planning_time_sec | makespan | "
        "sum_of_costs | total_wait | total_move_rotate |"
    )
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")

    for result in results:
        lines.append(
            f"| {result.case_name} | {result.enable_move_rotate} | "
            f"{result.success} | {result.planning_time_sec:.6f} | "
            f"{result.makespan} | {result.sum_of_costs} | "
            f"{result.total_wait} | {result.total_move_rotate} |"
        )

    filepath.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    """Phase 1 実行とベースライン保存を行う。"""
    RESULT_DIR.mkdir(parents=True, exist_ok=True)

    cases = get_phase1_benchmark_cases()
    planner_name = "CBS+A*"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    all_results: list[BenchmarkResult] = []

    print("=== Phase 1 benchmark start ===")
    for case in cases:
        for enable_move_rotate in (False, True):
            result = run_single_case(
                case=case,
                enable_move_rotate=enable_move_rotate,
                planner_name=planner_name,
                debug=False,
            )
            all_results.append(result)

            print(
                f"[DONE] {case.case_name} | "
                f"move+rotate={enable_move_rotate} | "
                f"success={result.success} | "
                f"time={result.planning_time_sec:.6f}s | "
                f"makespan={result.makespan} | "
                f"cost={result.sum_of_costs}"
            )

    benchmark_spec_payload = [asdict(case) for case in cases]
    result_payload = [asdict(result) for result in all_results]
    csv_rows = [benchmark_result_to_flat_row(r) for r in all_results]

    # 固定ファイル名（最新版）
    save_json(
        RESULT_DIR / "benchmark_spec.json",
        benchmark_spec_payload,
    )
    save_json(
        RESULT_DIR / "cbs_astar_baseline_latest.json",
        result_payload,
    )
    save_csv(
        RESULT_DIR / "cbs_astar_baseline_latest.csv",
        csv_rows,
    )
    save_markdown_summary(
        RESULT_DIR / "cbs_astar_baseline_latest.md",
        cases=cases,
        results=all_results,
    )

    # タイムスタンプ付き保存
    save_json(
        RESULT_DIR / f"cbs_astar_baseline_{timestamp}.json",
        result_payload,
    )
    save_csv(
        RESULT_DIR / f"cbs_astar_baseline_{timestamp}.csv",
        csv_rows,
    )
    save_markdown_summary(
        RESULT_DIR / f"cbs_astar_baseline_{timestamp}.md",
        cases=cases,
        results=all_results,
    )

    print("=== Phase 1 benchmark finished ===")
    print(f"Saved to: {RESULT_DIR.resolve()}")


if __name__ == "__main__":
    main()