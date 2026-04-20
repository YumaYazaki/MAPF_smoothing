from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

from environment import PlanningEnvironment
from mapf_types import ActionType, GridNode, OrientationMode, Path as RobotPath
from mapf_types import RobotGeometrySpec


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
    """ロボット単体のメトリクス。

    Attributes:
        cost: 経路コスト。
        num_states: 状態数。
        num_transitions: 遷移数。
        num_wait: WAIT 回数。
        num_move: MOVE 回数。
        num_rotate: ROTATE 回数。
        num_move_rotate: MOVE_ROTATE 回数。
    """

    cost: float
    num_states: int
    num_transitions: int
    num_wait: int
    num_move: int
    num_rotate: int
    num_move_rotate: int


@dataclass(frozen=True)
class BenchmarkResult:
    """単一ケース・単一設定の実験結果。

    Attributes:
        planner_name: プランナ名。
        case_id: ケース識別子。
        case_name: ケース名。
        purpose: ケースの狙い。
        tier: ケース分類。
        enable_move_rotate: move+rotate 許可フラグ。
        success: 解が得られたか。
        planning_time_sec: 計算時間 [sec]。
        num_robots: ロボット数。
        makespan: メイクスパン。
        sum_of_costs: コスト総和。
        total_wait: 全ロボットの WAIT 総回数。
        total_move: 全ロボットの MOVE 総回数。
        total_rotate: 全ロボットの ROTATE 総回数。
        total_move_rotate: 全ロボットの MOVE_ROTATE 総回数。
        per_robot: ロボット別メトリクス。
        notes: 任意メモ。
    """

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
    """初期PoC用の4姿勢モードを返す。

    Returns:
        姿勢モード一覧。
    """
    return [
        OrientationMode(0),
        OrientationMode(90),
        OrientationMode(180),
        OrientationMode(270),
    ]


def build_default_robot_geometry() -> RobotGeometrySpec:
    """初期PoC用ロボット形状を返す。

    Returns:
        ロボット形状仕様。
    """
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
        case: ベンチマークケース仕様。
        enable_move_rotate: move+rotate を許可するか。

    Returns:
        計画環境。
    """
    normalized_obstacles = {
        (int(obs[0]), int(obs[1])) for obs in case.obstacles
    }

    return PlanningEnvironment(
        rows=case.rows,
        cols=case.cols,
        obstacles=normalized_obstacles,
        modes=build_default_modes(),
        robot_geometry=build_default_robot_geometry(),
        enable_diagonal_move=False,
        enable_move_rotate=enable_move_rotate,
        enable_reserved_regions=False,
    )


def parse_states(
    state_dict: Dict[int, Tuple[int, int, int]],
) -> Dict[int, Tuple[GridNode, OrientationMode]]:
    """シリアライズ用状態辞書を内部表現へ変換する。

    Args:
        state_dict: ロボットID -> (row, col, deg)。

    Returns:
        ロボットID -> (GridNode, OrientationMode)。
    """
    parsed: Dict[int, Tuple[GridNode, OrientationMode]] = {}
    for robot_id, (row, col, deg) in state_dict.items():
        parsed[robot_id] = (GridNode(row=row, col=col), OrientationMode(deg))
    return parsed


def count_actions(
    path: RobotPath,
) -> Dict[str, int]:
    """経路中のアクション数を集計する。

    Args:
        path: 単一ロボット経路。

    Returns:
        アクション種別ごとの件数。
    """
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
    """解全体のメトリクスを集計する。

    Args:
        paths: ロボットID -> 経路。
        planning_time_sec: 計画時間 [sec]。
        planner_name: プランナ名。
        case: ベンチマークケース仕様。
        enable_move_rotate: move+rotate 許可フラグ。

    Returns:
        実験結果。
    """
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


def benchmark_result_to_flat_row(
    result: BenchmarkResult,
) -> Dict[str, Any]:
    """CSV 出力用に結果を平坦化する。

    Args:
        result: 実験結果。

    Returns:
        平坦化された辞書。
    """
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
    """JSON を保存する。

    Args:
        filepath: 保存先。
        payload: 保存内容。
    """
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with filepath.open("w", encoding="utf-8") as fp:
        json.dump(payload, fp, ensure_ascii=False, indent=2)


def save_csv(
    filepath: Path,
    rows: list[Dict[str, Any]],
) -> None:
    """CSV を保存する。

    Args:
        filepath: 保存先。
        rows: 行データ。
    """
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
    """簡易Markdownサマリを保存する。

    Args:
        filepath: 保存先。
        cases: ベンチマークケース一覧。
        results: 実験結果一覧。
    """
    filepath.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    lines.append("# Benchmark Summary")
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
    lines.append("## Results")
    lines.append("")
    lines.append(
        "| Case | Planner | move+rotate | success | planning_time_sec | "
        "makespan | sum_of_costs | total_wait | total_move_rotate |"
    )
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|")

    for result in results:
        lines.append(
            f"| {result.case_name} | {result.planner_name} | "
            f"{result.enable_move_rotate} | {result.success} | "
            f"{result.planning_time_sec:.6f} | {result.makespan} | "
            f"{result.sum_of_costs} | {result.total_wait} | "
            f"{result.total_move_rotate} |"
        )

    filepath.write_text("\n".join(lines), encoding="utf-8")


def save_benchmark_spec(
    filepath: Path,
    cases: list[BenchmarkCaseSpec],
) -> None:
    """ベンチマーク仕様を保存する。

    Args:
        filepath: 保存先。
        cases: ケース一覧。
    """
    save_json(filepath, [asdict(case) for case in cases])


def load_benchmark_spec(
    filepath: Path,
) -> list[BenchmarkCaseSpec]:
    """ベンチマーク仕様を読み込む。

    JSON では tuple が list に、dict の int key が str に変換されるため、
    読込時に元の型へ正規化する。

    Args:
        filepath: 読込元。

    Returns:
        ケース一覧。
    """
    payload = json.loads(filepath.read_text(encoding="utf-8"))

    cases: list[BenchmarkCaseSpec] = []
    for item in payload:
        normalized_obstacles = [
            (int(obs[0]), int(obs[1]))
            for obs in item["obstacles"]
        ]

        normalized_starts = {
            int(robot_id): (
                int(state[0]),
                int(state[1]),
                int(state[2]),
            )
            for robot_id, state in item["starts"].items()
        }

        normalized_goals = {
            int(robot_id): (
                int(state[0]),
                int(state[1]),
                int(state[2]),
            )
            for robot_id, state in item["goals"].items()
        }

        cases.append(
            BenchmarkCaseSpec(
                case_id=item["case_id"],
                case_name=item["case_name"],
                purpose=item["purpose"],
                rows=int(item["rows"]),
                cols=int(item["cols"]),
                obstacles=normalized_obstacles,
                starts=normalized_starts,
                goals=normalized_goals,
                max_time=int(item["max_time"]),
                tier=item["tier"],
            )
        )

    return cases