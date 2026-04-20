from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from environment import PlanningEnvironment
from geometry import (
    grid_node_center_in_fine,
    rotate_relative_cell_discrete,
)
from mapf_types import FineCell, Path, TimedState


@dataclass(frozen=True)
class VisualizationConfig:
    """可視化設定。

    Attributes:
        show_paths: 背景にロボットの経路を表示するか。
        show_labels: ロボットIDラベルを表示するか。
        show_fine_cells: 占有セルを細グリッドで表示するか。
        interval_ms: アニメーションの更新間隔 [ms]。
        show_center_cross: 回転中心を十字で描くか。
        highlight_base_cell: ムーバー基準セルを強調するか。
        highlight_tip_cell: 先端セルを強調するか。
        show_reference_states: 初期姿勢・最終姿勢を薄く表示するか。
    """

    show_paths: bool = True
    show_labels: bool = True
    show_fine_cells: bool = True
    interval_ms: int = 800
    show_center_cross: bool = True
    highlight_base_cell: bool = True
    highlight_tip_cell: bool = True
    show_reference_states: bool = True


def get_state_at_time(
    path: Path,
    time_idx: int,
) -> TimedState:
    """指定時刻の状態を返す。

    Args:
        path: 対象経路。
        time_idx: 参照時刻。

    Returns:
        指定時刻状態。
    """
    if time_idx < len(path.states):
        return path.states[time_idx]
    return path.states[-1]


def get_horizon(
    paths: Dict[int, Path],
) -> int:
    """全ロボット経路の可視化ホライズンを返す。

    Args:
        paths: ロボットID -> Path。

    Returns:
        可視化ホライズン。
    """
    return max(len(path.states) for path in paths.values())


def robot_color(
    robot_id: int,
) -> str:
    """ロボットIDに応じた色を返す。

    Args:
        robot_id: ロボットID。

    Returns:
        matplotlibで使える色名。
    """
    palette = [
        "tab:blue",
        "tab:orange",
        "tab:green",
        "tab:red",
        "tab:purple",
        "tab:brown",
        "tab:pink",
        "tab:gray",
    ]
    return palette[robot_id % len(palette)]


def setup_axes(
    ax: Axes,
    env: PlanningEnvironment,
) -> None:
    """描画軸を初期化する。

    Args:
        ax: 描画軸。
        env: 計画環境。
    """
    ax.set_xlim(0, env.cols)
    ax.set_ylim(env.rows, 0)
    ax.set_aspect("equal")
    ax.set_xticks(range(env.cols + 1))
    ax.set_yticks(range(env.rows + 1))
    ax.grid(True, which="both", linewidth=0.8)
    ax.set_xlabel("col")
    ax.set_ylabel("row")


def draw_obstacles(
    ax: Axes,
    env: PlanningEnvironment,
) -> None:
    """障害物を描画する。

    Args:
        ax: 描画軸。
        env: 計画環境。
    """
    for row, col in env.obstacles:
        ax.add_patch(
            Rectangle(
                (col, row),
                1.0,
                1.0,
                facecolor="black",
                edgecolor="black",
                alpha=0.55,
                zorder=1,
            )
        )


def draw_path_lines(
    ax: Axes,
    paths: Dict[int, Path],
) -> None:
    """各ロボットの経路中心線を背景に描画する。

    Args:
        ax: 描画軸。
        paths: ロボットID -> Path。
    """
    for robot_id, path in sorted(paths.items()):
        xs = [state.node.col + 0.5 for state in path.states]
        ys = [state.node.row + 0.5 for state in path.states]
        ax.plot(
            xs,
            ys,
            linestyle="--",
            marker="o",
            linewidth=1.5,
            markersize=3,
            alpha=0.45,
            color=robot_color(robot_id),
            zorder=0,
        )


def draw_robot_center(
    ax: Axes,
    robot_id: int,
    state: TimedState,
    show_labels: bool,
    alpha: float = 1.0,
    label_suffix: str = "",
) -> None:
    """ロボット中心を描画する。

    Args:
        ax: 描画軸。
        robot_id: ロボットID。
        state: 現在状態。
        show_labels: ラベル表示フラグ。
        alpha: 透明度。
        label_suffix: ラベル末尾文字列。
    """
    x = state.node.col + 0.5
    y = state.node.row + 0.5
    color = robot_color(robot_id)

    ax.scatter(
        [x],
        [y],
        s=90,
        color=color,
        edgecolors="black",
        alpha=alpha,
        zorder=6,
    )

    if show_labels:
        ax.text(
            x + 0.02,
            y - 0.18,
            f"R{robot_id}{label_suffix}\n{state.mode.degree}°",
            ha="center",
            va="center",
            fontsize=8,
            color=color,
            alpha=alpha,
            zorder=7,
        )


def draw_center_cross(
    ax: Axes,
    state: TimedState,
    alpha: float = 1.0,
) -> None:
    """回転中心を十字で描画する。

    Args:
        ax: 描画軸。
        state: 現在状態。
        alpha: 透明度。
    """
    x = state.node.col + 0.5
    y = state.node.row + 0.5
    size = 0.08

    ax.add_line(
        Line2D(
            [x - size, x + size],
            [y, y],
            color="black",
            linewidth=1.2,
            alpha=alpha,
            zorder=8,
        )
    )
    ax.add_line(
        Line2D(
            [x, x],
            [y - size, y + size],
            color="black",
            linewidth=1.2,
            alpha=alpha,
            zorder=8,
        )
    )


def relative_base_cell() -> FineCell:
    """ムーバー基準セルの相対座標を返す。

    Returns:
        相対セル座標。
    """
    return (0, 0)


def relative_tip_cell(
    env: PlanningEnvironment,
) -> FineCell:
    """0度姿勢での先端セルの相対座標を返す。

    Args:
        env: 計画環境。

    Returns:
        相対先端セル座標。
    """
    occ = list(env.robot_geometry.occupied_cells_at_zero_deg)
    return max(occ, key=lambda rc: rc[1])


def absolute_relative_cell(
    env: PlanningEnvironment,
    state: TimedState,
    rel_cell: FineCell,
) -> FineCell:
    """相対セル座標を絶対細グリッド座標へ変換する。

    可視化上の基準セル・先端セルの位置は、solver が使っている
    離散回転規則と一致させる。

    Args:
        env: 計画環境。
        state: 状態。
        rel_cell: 相対セル座標。

    Returns:
        絶対細グリッド座標。
    """
    center_row, center_col = grid_node_center_in_fine(
        node=state.node,
        fine_scale=env.robot_geometry.fine_scale,
    )

    rot_row, rot_col = rotate_relative_cell_discrete(
        rel_row=rel_cell[0],
        rel_col=rel_cell[1],
        degree=state.mode.degree,
    )

    return (
        int(round(center_row + rot_row)),
        int(round(center_col + rot_col)),
    )

def draw_highlight_fine_cell(
    ax: Axes,
    env: PlanningEnvironment,
    fine_cell: FineCell,
    edgecolor: str,
    linewidth: float,
    linestyle: str,
    alpha: float,
    zorder: int,
) -> None:
    """細グリッドの1セルを強調描画する。

    Args:
        ax: 描画軸。
        env: 計画環境。
        fine_cell: 細グリッドセル座標。
        edgecolor: 枠色。
        linewidth: 線幅。
        linestyle: 線種。
        alpha: 透明度。
        zorder: 描画順。
    """
    fine_row, fine_col = fine_cell
    fine_scale = env.robot_geometry.fine_scale
    ax.add_patch(
        Rectangle(
            (fine_col / fine_scale, fine_row / fine_scale),
            1.0 / fine_scale,
            1.0 / fine_scale,
            fill=False,
            edgecolor=edgecolor,
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alpha,
            zorder=zorder,
        )
    )


def draw_robot_occupancy(
    ax: Axes,
    env: PlanningEnvironment,
    robot_id: int,
    state: TimedState,
    config: VisualizationConfig,
    alpha: float = 0.28,
    edge_alpha: float = 0.4,
) -> None:
    """ロボットの占有セルを描画する。

    Args:
        ax: 描画軸。
        env: 計画環境。
        robot_id: ロボットID。
        state: 現在状態。
        config: 可視化設定。
        alpha: 塗りつぶし透明度。
        edge_alpha: 強調枠の透明度。
    """
    color = robot_color(robot_id)
    fine_scale = env.robot_geometry.fine_scale
    occ = env.state_occupancy_key(state.node, state.mode)

    for fine_row, fine_col in occ:
        x = fine_col / fine_scale
        y = fine_row / fine_scale
        ax.add_patch(
            Rectangle(
                (x, y),
                1.0 / fine_scale,
                1.0 / fine_scale,
                facecolor=color,
                edgecolor=color,
                alpha=alpha,
                linewidth=0.5,
                zorder=2,
            )
        )

    if config.highlight_base_cell:
        base_cell = absolute_relative_cell(
            env=env,
            state=state,
            rel_cell=relative_base_cell(),
        )
        draw_highlight_fine_cell(
            ax=ax,
            env=env,
            fine_cell=base_cell,
            edgecolor="black",
            linewidth=1.3,
            linestyle="-",
            alpha=edge_alpha,
            zorder=5,
        )

    if config.highlight_tip_cell:
        tip_cell = absolute_relative_cell(
            env=env,
            state=state,
            rel_cell=relative_tip_cell(env),
        )
        draw_highlight_fine_cell(
            ax=ax,
            env=env,
            fine_cell=tip_cell,
            edgecolor="red",
            linewidth=1.2,
            linestyle="-",
            alpha=edge_alpha,
            zorder=5,
        )


def draw_reference_state(
    ax: Axes,
    env: PlanningEnvironment,
    robot_id: int,
    state: TimedState,
    label_suffix: str,
) -> None:
    """初期姿勢・最終姿勢の参考表示を描画する。

    Args:
        ax: 描画軸。
        env: 計画環境。
        robot_id: ロボットID。
        state: 参考状態。
        label_suffix: "S" または "G" など。
    """
    draw_robot_occupancy(
        ax=ax,
        env=env,
        robot_id=robot_id,
        state=state,
        config=VisualizationConfig(
            show_paths=False,
            show_labels=False,
            show_fine_cells=True,
            interval_ms=800,
            show_center_cross=False,
            highlight_base_cell=True,
            highlight_tip_cell=True,
            show_reference_states=False,
        ),
        alpha=0.08,
        edge_alpha=0.18,
    )

    draw_robot_center(
        ax=ax,
        robot_id=robot_id,
        state=state,
        show_labels=True,
        alpha=0.22,
        label_suffix=label_suffix,
    )


def draw_state_snapshot(
    ax: Axes,
    env: PlanningEnvironment,
    paths: Dict[int, Path],
    time_idx: int,
    config: VisualizationConfig,
) -> None:
    """指定時刻の状態を描画する。

    Args:
        ax: 描画軸。
        env: 計画環境。
        paths: ロボットID -> Path。
        time_idx: 表示時刻。
        config: 可視化設定。
    """
    ax.clear()
    setup_axes(ax, env)
    draw_obstacles(ax, env)

    if config.show_paths:
        draw_path_lines(ax, paths)

    if config.show_reference_states:
        for robot_id, path in sorted(paths.items()):
            start_state = path.states[0]
            goal_state = path.states[-1]
            draw_reference_state(
                ax=ax,
                env=env,
                robot_id=robot_id,
                state=start_state,
                label_suffix="S",
            )
            draw_reference_state(
                ax=ax,
                env=env,
                robot_id=robot_id,
                state=goal_state,
                label_suffix="G",
            )

    for robot_id, path in sorted(paths.items()):
        state = get_state_at_time(path, time_idx)

        if config.show_fine_cells:
            draw_robot_occupancy(
                ax=ax,
                env=env,
                robot_id=robot_id,
                state=state,
                config=config,
                alpha=0.28,
                edge_alpha=0.95,
            )

        draw_robot_center(
            ax=ax,
            robot_id=robot_id,
            state=state,
            show_labels=config.show_labels,
            alpha=1.0,
            label_suffix="",
        )

        if config.show_center_cross:
            draw_center_cross(ax=ax, state=state, alpha=1.0)

    ax.set_title(f"Time = {time_idx}")


def plot_solution_timestep(
    env: PlanningEnvironment,
    paths: Dict[int, Path],
    time_idx: int,
    config: Optional[VisualizationConfig] = None,
) -> tuple[Figure, Axes]:
    """指定時刻の静止画を生成する。

    Args:
        env: 計画環境。
        paths: ロボットID -> Path。
        time_idx: 表示時刻。
        config: 可視化設定。

    Returns:
        Figure と Axes。
    """
    if config is None:
        config = VisualizationConfig()

    fig, ax = plt.subplots(figsize=(7, 5))
    draw_state_snapshot(
        ax=ax,
        env=env,
        paths=paths,
        time_idx=time_idx,
        config=config,
    )
    fig.tight_layout()
    return fig, ax


def animate_solution(
    env: PlanningEnvironment,
    paths: Dict[int, Path],
    config: Optional[VisualizationConfig] = None,
) -> animation.FuncAnimation:
    """解の時系列アニメーションを生成する。

    Args:
        env: 計画環境。
        paths: ロボットID -> Path。
        config: 可視化設定。

    Returns:
        matplotlib の FuncAnimation。
    """
    if config is None:
        config = VisualizationConfig()

    horizon = get_horizon(paths)
    fig, ax = plt.subplots(figsize=(7, 5))

    def update(frame_idx: int) -> None:
        draw_state_snapshot(
            ax=ax,
            env=env,
            paths=paths,
            time_idx=frame_idx,
            config=config,
        )

    anim = animation.FuncAnimation(
        fig=fig,
        func=update,
        frames=horizon,
        interval=config.interval_ms,
        repeat=False,
    )
    fig.tight_layout()
    return anim


def show_animation(
    env: PlanningEnvironment,
    paths: Dict[int, Path],
    config: Optional[VisualizationConfig] = None,
) -> animation.FuncAnimation:
    """アニメーションを作って表示する。

    Args:
        env: 計画環境。
        paths: ロボットID -> Path。
        config: 可視化設定。

    Returns:
        生成したアニメーション。
    """
    anim = animate_solution(
        env=env,
        paths=paths,
        config=config,
    )
    plt.show()
    return anim