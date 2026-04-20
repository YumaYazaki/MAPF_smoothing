from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from benchmark_common import (
    BenchmarkCaseSpec,
    BenchmarkResult,
    benchmark_result_to_flat_row,
    load_benchmark_spec,
    save_csv,
    save_json,
    save_markdown_summary,
)
from lacam_adapter import build_lacam_problem, run_single_case_lacam_result
from lacam_runner import LaCAMBackend, run_lacam
from orientation_lacam_backend_v2 import OrientationLaCAMBackendV2
from orientation_lacam_backend_v3 import OrientationLaCAMBackendV3


RESULT_DIR = Path("MAPF/results/phase2")
PHASE1_SPEC_PATH = Path("MAPF/results/phase1/benchmark_spec.json")

PLANNER_NAME = "OrientationLaCAMV3"
TIME_LIMIT_SEC = 30.0
DEBUG = False


def build_lacam_backend() -> LaCAMBackend:
    """LaCAM backend を構築する。"""
    return OrientationLaCAMBackendV3(
        max_high_level_expansions=20_000,
        max_depth=128,
        max_branch_children=8,
        max_single_agent_candidates=6,
        max_joint_successors=6,
        pair_priority_successors_per_pattern=2,
    )


def run_single_case(
    case: BenchmarkCaseSpec,
    enable_move_rotate: bool,
    planner_name: str = PLANNER_NAME,
    debug: bool = DEBUG,
    time_limit_sec: float | None = TIME_LIMIT_SEC,
) -> BenchmarkResult:
    """単一ケースを LaCAM backend で実行し、結果を返す。

    Args:
        case: ベンチマークケース。
        enable_move_rotate: move+rotate を許可するか。
        planner_name: プランナ名。
        debug: デバッグ出力有効化フラグ。
        time_limit_sec: 計算時間制限 [sec]。

    Returns:
        実験結果。
    """
    problem = build_lacam_problem(
        case=case,
        enable_move_rotate=enable_move_rotate,
    )

    backend = build_lacam_backend()
    lacam_solution = run_lacam(
        problem=problem,
        backend=backend,
        time_limit_sec=time_limit_sec,
        debug=debug,
        seed=42,
    )

    return run_single_case_lacam_result(
        case=case,
        enable_move_rotate=enable_move_rotate,
        planner_name=planner_name,
        lacam_solution=lacam_solution,
    )


def build_result_payload(
    results: list[BenchmarkResult],
) -> list[Dict[str, Any]]:
    """JSON 保存用 payload を構築する。

    Args:
        results: 実験結果一覧。

    Returns:
        JSON 保存用辞書一覧。
    """
    return [asdict(result) for result in results]


def build_csv_rows(
    results: list[BenchmarkResult],
) -> list[Dict[str, Any]]:
    """CSV 保存用行データを構築する。

    Args:
        results: 実験結果一覧。

    Returns:
        CSV 行データ一覧。
    """
    return [benchmark_result_to_flat_row(result) for result in results]


def main() -> None:
    """Phase 2 ベンチマークを実行する。"""
    RESULT_DIR.mkdir(parents=True, exist_ok=True)

    if not PHASE1_SPEC_PATH.exists():
        raise FileNotFoundError(
            "Phase 1 benchmark spec not found. "
            f"Expected: {PHASE1_SPEC_PATH}"
        )

    cases = load_benchmark_spec(PHASE1_SPEC_PATH)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    all_results: list[BenchmarkResult] = []

    print("=== Phase 2 benchmark (OrientationLaCAMV3) start ===")
    print(f"Loaded benchmark spec: {PHASE1_SPEC_PATH.resolve()}")

    for case in cases:
        for enable_move_rotate in (False, True):
            result = run_single_case(
                case=case,
                enable_move_rotate=enable_move_rotate,
                planner_name=PLANNER_NAME,
                debug=DEBUG,
                time_limit_sec=TIME_LIMIT_SEC,
            )
            all_results.append(result)

            print(
                f"[DONE] {case.case_name} | "
                f"move+rotate={enable_move_rotate} | "
                f"success={result.success} | "
                f"time={result.planning_time_sec:.6f}s | "
                f"makespan={result.makespan} | "
                f"cost={result.sum_of_costs} | "
                f"notes={result.notes}"
            )

    result_payload = build_result_payload(all_results)
    csv_rows = build_csv_rows(all_results)

    save_json(
        RESULT_DIR / "orientation_lacam_v3_latest.json",
        result_payload,
    )
    save_csv(
        RESULT_DIR / "orientation_lacam_v3_latest.csv",
        csv_rows,
    )
    save_markdown_summary(
        RESULT_DIR / "orientation_lacam_v3_latest.md",
        cases=cases,
        results=all_results,
    )

    save_json(
        RESULT_DIR / f"orientation_lacam_v3_{timestamp}.json",
        result_payload,
    )
    save_csv(
        RESULT_DIR / f"orientation_lacam_v3_{timestamp}.csv",
        csv_rows,
    )
    save_markdown_summary(
        RESULT_DIR / f"orientation_lacam_v3_{timestamp}.md",
        cases=cases,
        results=all_results,
    )

    print("=== Phase 2 benchmark (OrientationLaCAMV3) finished ===")
    print(f"Saved to: {RESULT_DIR.resolve()}")


if __name__ == "__main__":
    main()