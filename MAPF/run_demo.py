from __future__ import annotations

import time
from typing import Any, Dict, Optional, Set, Tuple

from cbs import cbs_plan
from environment import PlanningEnvironment
from mapf_types import ActionType, GridNode, OrientationMode, Path, RobotGeometrySpec
from visualize import VisualizationConfig, show_animation, plot_solution_timestep
from matplotlib import pyplot as plt
from visualize_envelope_debug import plot_path_transition_at_time, plot_pair_path_transitions_at_time
import matplotlib.pyplot as plt


# ============================================================
# Visualization flags
# ============================================================
ENABLE_VISUALIZATION = True
VISUALIZATION_INTERVAL_MS = 1000

# None の場合は全ケース可視化対象。
# 例: {"Case 03: two robot corridor", "Case C: coordination sensitive"}
# VISUALIZE_CASE_NAMES: Optional[Set[str]] = None
VISUALIZE_CASE_NAMES: Optional[Set[str]] = {"Case C: coordination sensitive"}



def should_visualize_case(case_name: str) -> bool:
    """ケースを可視化するか判定する。

    Args:
        case_name: ケース名。

    Returns:
        可視化する場合True。
    """
    if not ENABLE_VISUALIZATION:
        return False
    if VISUALIZE_CASE_NAMES is None:
        return True
    return case_name in VISUALIZE_CASE_NAMES


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
    """左右対称・前後非対称な初期ロボット形状を返す。

    Returns:
        ロボット形状仕様。
    """
    return RobotGeometrySpec(
        fine_scale=4,
        occupied_cells_at_zero_deg=frozenset(
            {
                # ムーバー本体
                (0, 0),
                # アーム
                (0, 1),
                (0, 2),
                (0, 3),
                # 搬送物
                (0, 4),
            }
        ),
    )


def build_demo_environment() -> PlanningEnvironment:
    """最小検証用環境を構築する。

    Returns:
        計画環境。
    """
    obstacles: set[tuple[int, int]] = set()

    env = PlanningEnvironment(
        rows=5,
        cols=7,
        obstacles=obstacles,
        modes=build_default_modes(),
        robot_geometry=build_default_robot_geometry(),
        enable_diagonal_move=False,
        enable_move_rotate=False,
        enable_reserved_regions=False,
    )
    return env


def print_solution(paths: Dict[int, Path]) -> None:
    """解を読みやすく表示する。

    Args:
        paths: ロボットID -> Path。
    """
    for robot_id, path in sorted(paths.items()):
        print(f"=== Robot {robot_id} ===")
        print(f"Cost: {path.cost}")
        print("States:")
        for state in path.states:
            print(
                f"  t={state.time:2d}, "
                f"pos=({state.node.row},{state.node.col}), "
                f"mode={state.mode.degree}"
            )
        print("Transitions:")
        for transition in path.transitions:
            print(
                f"  t={transition.time:2d}, "
                f"{transition.action_type.value:12s}, "
                f"({transition.from_node.row},{transition.from_node.col},"
                f"{transition.from_mode.degree})"
                f" -> "
                f"({transition.to_node.row},{transition.to_node.col},"
                f"{transition.to_mode.degree})"
            )
        print()


def print_joint_timeline(paths: Dict[int, Path]) -> None:
    """複数ロボットの状態を時系列で並べて表示する。

    Args:
        paths: ロボットID -> Path。
    """
    horizon = max(len(path.states) for path in paths.values())

    print("Joint timeline:")
    for time_idx in range(horizon):
        row_items = [f"t={time_idx:2d}"]
        for robot_id, path in sorted(paths.items()):
            if time_idx < len(path.states):
                state = path.states[time_idx]
            else:
                state = path.states[-1]

            row_items.append(
                f"R{robot_id}:({state.node.row},{state.node.col},"
                f"{state.mode.degree})"
            )
        print(" | ".join(row_items))
    print()


def count_actions(path: Path) -> Dict[str, int]:
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
    paths: Dict[int, Path],
    planning_time_sec: float,
) -> Dict[str, Any]:
    """解全体の評価指標を集計する。

    Args:
        paths: ロボットID -> Path。
        planning_time_sec: 計画時間 [sec]。

    Returns:
        評価指標辞書。
    """
    per_robot: Dict[int, Dict[str, Any]] = {}
    sum_of_costs = 0
    makespan = 0

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

        per_robot[robot_id] = {
            "cost": path.cost,
            "num_states": len(path.states),
            "num_transitions": len(path.transitions),
            "num_wait": action_counts["wait"],
            "num_move": action_counts["move"],
            "num_rotate": action_counts["rotate"],
            "num_move_rotate": action_counts["move_rotate"],
        }

    return {
        "planning_time_sec": planning_time_sec,
        "num_robots": len(paths),
        "makespan": makespan,
        "sum_of_costs": sum_of_costs,
        "total_wait": total_wait,
        "total_move": total_move,
        "total_rotate": total_rotate,
        "total_move_rotate": total_move_rotate,
        "per_robot": per_robot,
    }


def print_metrics(metrics: Dict[str, Any]) -> None:
    """評価指標を読みやすく表示する。

    Args:
        metrics: 評価指標辞書。
    """
    print("Metrics:")
    print(f"  planning_time_sec : {metrics['planning_time_sec']:.6f}")
    print(f"  num_robots        : {metrics['num_robots']}")
    print(f"  makespan          : {metrics['makespan']}")
    print(f"  sum_of_costs      : {metrics['sum_of_costs']}")
    print(f"  total_wait        : {metrics['total_wait']}")
    print(f"  total_move        : {metrics['total_move']}")
    print(f"  total_rotate      : {metrics['total_rotate']}")
    print(f"  total_move_rotate : {metrics['total_move_rotate']}")
    print("  per_robot:")
    for robot_id, robot_metrics in metrics["per_robot"].items():
        print(
            f"    R{robot_id}: "
            f"cost={robot_metrics['cost']}, "
            f"states={robot_metrics['num_states']}, "
            f"transitions={robot_metrics['num_transitions']}, "
            f"wait={robot_metrics['num_wait']}, "
            f"move={robot_metrics['num_move']}, "
            f"rotate={robot_metrics['num_rotate']}, "
            f"move_rotate={robot_metrics['num_move_rotate']}"
        )
    print()


def maybe_visualize(
    case_name: str,
    env: PlanningEnvironment,
    solution: Dict[int, Path],
) -> None:
    """必要に応じて解を可視化する。

    Args:
        case_name: ケース名。
        env: 計画環境。
        solution: ロボットID -> Path。
    """
    if not should_visualize_case(case_name):
        return

    print(f"[Visualization] Start: {case_name}")
    show_animation(
        env=env,
        paths=solution,
        config=VisualizationConfig(
            show_paths=True,
            show_labels=True,
            show_fine_cells=True,
            interval_ms=VISUALIZATION_INTERVAL_MS,
        ),
    )

    # fig, ax = plot_path_transition_at_time(
    #     env=env,
    #     paths=solution,
    #     robot_id=1,
    #     time_idx=2,
    # )
    # plt.show()

    for i in range(8):
        fig, ax = plot_pair_path_transitions_at_time(
            env=env,
            paths=solution,
            robot_i=0,
            robot_j=1,
            time_idx=i,
        )
        plt.show()




def solve_case(
    case_name: str,
    env: PlanningEnvironment,
    starts: Dict[int, Tuple[GridNode, OrientationMode]],
    goals: Dict[int, Tuple[GridNode, OrientationMode]],
    max_time: int,
    debug: bool = False,
) -> None:
    """1ケース分の計画を実行し、結果と計算時間を表示する。

    Args:
        case_name: ケース名。
        env: 計画環境。
        starts: ロボットID -> (始点ノード, 始点姿勢)。
        goals: ロボットID -> (目標ノード, 目標姿勢)。
        max_time: 低レベル探索最大時刻。
        debug: デバッグ出力有効化フラグ。
    """
    print(f"### {case_name} ###")

    start_time = time.perf_counter()
    solution = cbs_plan(
        starts=starts,
        goals=goals,
        env=env,
        max_time=max_time,
        debug=debug,
    )
    elapsed_sec = time.perf_counter() - start_time

    print(f"Elapsed time: {elapsed_sec:.6f} sec")

    if solution is None:
        print("No solution found.\n")
        return

    metrics = summarize_solution(
        paths=solution,
        planning_time_sec=elapsed_sec,
    )
    print_metrics(metrics)
    print_solution(solution)
    print_joint_timeline(solution)

    maybe_visualize(
        case_name=case_name,
        env=env,
        solution=solution,
    )


def run_case_01_rotation_only() -> None:
    """単一ロボットのその場回転を確認する。"""
    env = build_demo_environment()

    starts = {
        0: (GridNode(2, 2), OrientationMode(0)),
    }
    goals = {
        0: (GridNode(2, 2), OrientationMode(180)),
    }

    solve_case(
        case_name="Case 01: rotation only",
        env=env,
        starts=starts,
        goals=goals,
        max_time=10,
        debug=False,
    )


def run_case_02_move_rotate() -> None:
    """単一ロボットの move+rotate を確認する。"""
    env = build_demo_environment()

    starts = {
        0: (GridNode(3, 1), OrientationMode(0)),
    }
    goals = {
        0: (GridNode(2, 2), OrientationMode(90)),
    }

    solve_case(
        case_name="Case 02: move+rotate",
        env=env,
        starts=starts,
        goals=goals,
        max_time=10,
        debug=False,
    )


def run_case_03_two_robot_corridor() -> None:
    """2ロボットの対向通過を確認する。"""
    env = build_demo_environment()

    starts = {
        0: (GridNode(2, 1), OrientationMode(0)),
        1: (GridNode(2, 5), OrientationMode(180)),
    }
    goals = {
        0: (GridNode(2, 5), OrientationMode(0)),
        1: (GridNode(2, 1), OrientationMode(180)),
    }

    solve_case(
        case_name="Case 03: two robot corridor",
        env=env,
        starts=starts,
        goals=goals,
        max_time=20,
        debug=False,
    )


def run_case_04_two_robot_with_obstacle() -> None:
    """障害物ありの2ロボットケースを確認する。"""
    env = build_demo_environment()
    env.obstacles = {
        (1, 3),
        (2, 3),
        (3, 3),
    }
    env.clear_caches()

    starts = {
        0: (GridNode(0, 1), OrientationMode(0)),
        1: (GridNode(4, 5), OrientationMode(180)),
    }
    goals = {
        0: (GridNode(4, 5), OrientationMode(90)),
        1: (GridNode(0, 1), OrientationMode(270)),
    }

    solve_case(
        case_name="Case 04: two robot with obstacle wall",
        env=env,
        starts=starts,
        goals=goals,
        max_time=30,
        debug=False,
    )


def run_case_a_move_rotate_helpful() -> None:
    """Case A: move+rotate が明確に効く単一ロボットケース。"""
    env = build_demo_environment()
    env.rows = 5
    env.cols = 5
    env.obstacles = set()
    env.clear_caches()

    starts = {
        0: (GridNode(4, 1), OrientationMode(0)),
    }
    goals = {
        0: (GridNode(1, 4), OrientationMode(90)),
    }

    solve_case(
        case_name="Case A: move+rotate helpful",
        env=env,
        starts=starts,
        goals=goals,
        max_time=20,
        debug=False,
    )


def run_case_b_move_rotate_unhelpful() -> None:
    """Case B: move+rotate が効きにくい広い単純ケース。"""
    env = build_demo_environment()
    env.rows = 5
    env.cols = 9
    env.obstacles = set()
    env.clear_caches()

    starts = {
        0: (GridNode(2, 1), OrientationMode(0)),
        1: (GridNode(0, 7), OrientationMode(180)),
    }
    goals = {
        0: (GridNode(2, 7), OrientationMode(0)),
        1: (GridNode(0, 1), OrientationMode(180)),
    }

    solve_case(
        case_name="Case B: move+rotate likely unhelpful",
        env=env,
        starts=starts,
        goals=goals,
        max_time=25,
        debug=False,
    )


def run_case_c_coordination_sensitive() -> None:
    """Case C: 協調で move+rotate の価値が出る可能性があるケース。"""
    env = build_demo_environment()
    env.rows = 5
    env.cols = 7
    env.obstacles = {
        (0, 3),
        (1, 3),
        (3, 3),
        (4, 3),
    }
    env.clear_caches()

    starts = {
        0: (GridNode(2, 1), OrientationMode(0)),
        1: (GridNode(1, 5), OrientationMode(180)),
    }
    goals = {
        0: (GridNode(2, 5), OrientationMode(0)),
        1: (GridNode(3, 1), OrientationMode(180)),
    }

    solve_case(
        case_name="Case C: coordination sensitive",
        env=env,
        starts=starts,
        goals=goals,
        max_time=30,
        debug=False,
    )


if __name__ == "__main__":
    run_case_01_rotation_only()
    run_case_02_move_rotate()
    run_case_03_two_robot_corridor()
    run_case_04_two_robot_with_obstacle()
    run_case_a_move_rotate_helpful()
    run_case_b_move_rotate_unhelpful()
    run_case_c_coordination_sensitive()