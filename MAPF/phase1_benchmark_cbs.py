from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

from benchmark_common import (
    BenchmarkCaseSpec,
    BenchmarkResult,
    benchmark_result_to_flat_row,
    build_environment,
    parse_states,
    save_benchmark_spec,
    save_csv,
    save_json,
    save_markdown_summary,
    summarize_solution,
)
from cbs import cbs_plan


RESULT_DIR = Path("MAPF/results/phase1")

# CBS 高レベル探索の打ち切り条件
CBS_MAX_WALLCLOCK_SEC = 60.0
CBS_MAX_HIGH_LEVEL_EXPANSIONS = 10000


def get_phase1_benchmark_cases() -> list[BenchmarkCaseSpec]:
    """Phase 1 で固定するベンチマークケース一覧を返す。

    Returns:
        ベンチマークケース一覧。
    """
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
        BenchmarkCaseSpec(
            case_id="case_d",
            case_name="Case D: 3-agent single bottleneck",
            purpose="3台が単一ボトルネックを共有し、通過順序で品質差が出るケース",
            rows=5,
            cols=9,
            obstacles=[
                (0, 4),
                (1, 4),
                (3, 4),
                (4, 4),
            ],
            starts={
                0: (2, 1, 0),
                1: (1, 7, 180),
                2: (3, 7, 180),
            },
            goals={
                0: (2, 7, 0),
                1: (1, 1, 180),
                2: (3, 1, 180),
            },
            max_time=40,
            tier="Tier 4: hard coordination",
        ),
        BenchmarkCaseSpec(
            case_id="case_e",
            case_name="Case E: 4-agent cross intersection",
            purpose="4方向から中央へ同時進入する十字交差ケース",
            rows=7,
            cols=7,
            obstacles=[],
            starts={
                0: (3, 0, 0),
                1: (0, 3, 270),
                2: (3, 6, 180),
                3: (6, 3, 90),
            },
            goals={
                0: (3, 6, 0),
                1: (6, 3, 270),
                2: (3, 0, 180),
                3: (0, 3, 90),
            },
            max_time=50,
            tier="Tier 4: hard coordination",
        ),
        BenchmarkCaseSpec(
            case_id="case_f",
            case_name="Case F: pocket evacuation",
            purpose="退避ポケットを使うか待機するかで差が出るケース",
            rows=5,
            cols=9,
            obstacles=[
                (0, 4),
                (1, 4),
                (4, 4),
            ],
            starts={
                0: (2, 1, 0),
                1: (2, 7, 180),
                2: (3, 6, 180),
            },
            goals={
                0: (2, 7, 0),
                1: (2, 1, 180),
                2: (3, 1, 180),
            },
            max_time=40,
            tier="Tier 4: hard coordination",
        ),
        BenchmarkCaseSpec(
            case_id="case_g",
            case_name="Case G: double bottleneck",
            purpose="2つの狭窄部をまたぐため局所最適に落ちやすいケース",
            rows=5,
            cols=11,
            obstacles=[
                (0, 3),
                (1, 3),
                (3, 3),
                (4, 3),
                (0, 7),
                (1, 7),
                (3, 7),
                (4, 7),
            ],
            starts={
                0: (2, 1, 0),
                1: (1, 9, 180),
                2: (2, 9, 180),
                3: (3, 9, 180),
            },
            goals={
                0: (2, 9, 0),
                1: (1, 1, 180),
                2: (2, 1, 180),
                3: (3, 1, 180),
            },
            max_time=60,
            tier="Tier 4: hard coordination",
        ),
        BenchmarkCaseSpec(
            case_id="case_h",
            case_name="Case H: asymmetric orientation-critical",
            purpose="姿勢変更が本質的に効く非対称な強干渉ケース",
            rows=6,
            cols=8,
            obstacles=[
                (0, 3),
                (1, 3),
                (4, 3),
                (5, 3),
            ],
            starts={
                0: (2, 1, 0),
                1: (3, 6, 180),
                2: (2, 6, 180),
            },
            goals={
                0: (3, 6, 90),
                1: (2, 1, 270),
                2: (2, 2, 180),
            },
            max_time=50,
            tier="Tier 4: hard coordination",
        ),
    ]


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

    env = build_environment(
        case=case,
        enable_move_rotate=enable_move_rotate,
    )
    starts = parse_states(case.starts)
    goals = parse_states(case.goals)

    start_time = time.perf_counter()
    solution = cbs_plan(
        starts=starts,
        goals=goals,
        env=env,
        max_time=case.max_time,
        debug=debug,
        max_wallclock_sec=CBS_MAX_WALLCLOCK_SEC,
        max_high_level_expansions=CBS_MAX_HIGH_LEVEL_EXPANSIONS,
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
            notes=(
                "No solution found "
                f"(max_wallclock_sec={CBS_MAX_WALLCLOCK_SEC}, "
                f"max_high_level_expansions="
                f"{CBS_MAX_HIGH_LEVEL_EXPANSIONS})"
            ),
        )

    return summarize_solution(
        paths=solution,
        planning_time_sec=planning_time_sec,
        planner_name=planner_name,
        case=case,
        enable_move_rotate=enable_move_rotate,
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
    """Phase 1 実行とベースライン保存を行う。"""
    RESULT_DIR.mkdir(parents=True, exist_ok=True)

    cases = get_phase1_benchmark_cases()
    planner_name = "CBS+A*"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    all_results: list[BenchmarkResult] = []

    print("=== Phase 1 benchmark start ===")
    print(
        "CBS limits: "
        f"max_wallclock_sec={CBS_MAX_WALLCLOCK_SEC}, "
        f"max_high_level_expansions={CBS_MAX_HIGH_LEVEL_EXPANSIONS}"
    )

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
                f"cost={result.sum_of_costs} | "
                f"notes={result.notes}"
            )

    result_payload = build_result_payload(all_results)
    csv_rows = build_csv_rows(all_results)

    save_benchmark_spec(
        RESULT_DIR / "benchmark_spec.json",
        cases=cases,
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