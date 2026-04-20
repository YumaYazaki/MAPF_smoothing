from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle

from environment import PlanningEnvironment
from mapf_types import Path, TimedState, Transition


@dataclass(frozen=True)
class EnvelopeDebugConfig:
    """envelope デバッグ表示設定。

    Attributes:
        show_start_state: 始点 occupancy を表示するか。
        show_goal_state: 終点 occupancy を表示するか。
        show_center_markers: 始点・終点中心を表示するか。
        show_labels: ラベルを表示するか。
    """

    show_start_state: bool = True
    show_goal_state: bool = True
    show_center_markers: bool = True
    show_labels: bool = True


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


def draw_fine_cells(
    ax: Axes,
    env: PlanningEnvironment,
    fine_cells: set[tuple[int, int]] | frozenset[tuple[int, int]],
    facecolor: str,
    edgecolor: str,
    alpha: float,
    linewidth: float,
    zorder: int,
) -> None:
    """fine cell 集合を描画する。

    Args:
        ax: 描画軸。
        env: 計画環境。
        fine_cells: 描画対象 fine cell 集合。
        facecolor: 塗り色。
        edgecolor: 枠色。
        alpha: 透明度。
        linewidth: 線幅。
        zorder: 描画順。
    """
    fine_scale = env.robot_geometry.fine_scale

    for fine_row, fine_col in fine_cells:
        ax.add_patch(
            Rectangle(
                (fine_col / fine_scale, fine_row / fine_scale),
                1.0 / fine_scale,
                1.0 / fine_scale,
                facecolor=facecolor,
                edgecolor=edgecolor,
                alpha=alpha,
                linewidth=linewidth,
                zorder=zorder,
            )
        )


def draw_state_reference(
    ax: Axes,
    env: PlanningEnvironment,
    state: TimedState,
    color: str,
    label: Optional[str],
    alpha_cells: float,
    alpha_marker: float,
    linestyle: str,
) -> None:
    """状態 occupancy と中心を参考表示する。

    Args:
        ax: 描画軸。
        env: 計画環境。
        state: 対象状態。
        color: 色。
        label: ラベル。
        alpha_cells: occupancy 透明度。
        alpha_marker: 中心マーカー透明度。
        linestyle: 中心ラベル用説明文字列。
    """
    occ = env.state_occupancy_key(state.node, state.mode)

    draw_fine_cells(
        ax=ax,
        env=env,
        fine_cells=occ,
        facecolor=color,
        edgecolor=color,
        alpha=alpha_cells,
        linewidth=0.4,
        zorder=3,
    )

    x = state.node.col + 0.5
    y = state.node.row + 0.5
    ax.scatter(
        [x],
        [y],
        s=80,
        color=color,
        edgecolors="black",
        alpha=alpha_marker,
        zorder=6,
    )

    if label is not None:
        ax.text(
            x,
            y - 0.18,
            label,
            ha="center",
            va="center",
            fontsize=8,
            color=color,
            alpha=alpha_marker,
            zorder=7,
        )


def plot_transition_envelope(
    env: PlanningEnvironment,
    transition: Transition,
    robot_id: int = 0,
    config: Optional[EnvelopeDebugConfig] = None,
    title: Optional[str] = None,
) -> tuple[Figure, Axes]:
    """1つの遷移について envelope を可視化する。

    表示内容:
    - 障害物
    - 始点 occupancy
    - 終点 occupancy
    - transition envelope

    Args:
        env: 計画環境。
        transition: 可視化対象遷移。
        robot_id: 色決め用ロボットID。
        config: 表示設定。
        title: タイトル。

    Returns:
        Figure と Axes。
    """
    if config is None:
        config = EnvelopeDebugConfig()

    fig, ax = plt.subplots(figsize=(7, 5))
    setup_axes(ax, env)
    draw_obstacles(ax, env)

    color = robot_color(robot_id)
    envelope = env.transition_envelope(
        from_node=transition.from_node,
        from_mode=transition.from_mode,
        to_node=transition.to_node,
        to_mode=transition.to_mode,
    )

    draw_fine_cells(
        ax=ax,
        env=env,
        fine_cells=envelope,
        facecolor=color,
        edgecolor=color,
        alpha=0.18,
        linewidth=0.4,
        zorder=2,
    )

    if config.show_start_state:
        start_state = TimedState(
            node=transition.from_node,
            mode=transition.from_mode,
            time=transition.time,
        )
        draw_state_reference(
            ax=ax,
            env=env,
            state=start_state,
            color=color,
            label=f"start\n{transition.from_mode.degree}°"
            if config.show_labels
            else None,
            alpha_cells=0.30,
            alpha_marker=0.90 if config.show_center_markers else 0.0,
            linestyle="--",
        )

    if config.show_goal_state:
        goal_state = TimedState(
            node=transition.to_node,
            mode=transition.to_mode,
            time=transition.time + 1,
        )
        draw_state_reference(
            ax=ax,
            env=env,
            state=goal_state,
            color=color,
            label=f"goal\n{transition.to_mode.degree}°"
            if config.show_labels
            else None,
            alpha_cells=0.12,
            alpha_marker=0.55 if config.show_center_markers else 0.0,
            linestyle="-",
        )

    if title is None:
        title = (
            f"Transition envelope: "
            f"({transition.from_node.row},{transition.from_node.col},"
            f"{transition.from_mode.degree}) -> "
            f"({transition.to_node.row},{transition.to_node.col},"
            f"{transition.to_mode.degree})"
        )
    ax.set_title(title)
    fig.tight_layout()
    return fig, ax


def plot_pair_transition_envelopes(
    env: PlanningEnvironment,
    transition_i: Transition,
    transition_j: Transition,
    robot_i: int = 0,
    robot_j: int = 1,
    title: Optional[str] = None,
) -> tuple[Figure, Axes]:
    """2つの遷移の envelope と重なりを可視化する。

    表示内容:
    - ロボットi の envelope
    - ロボットj の envelope
    - overlap セル

    Args:
        env: 計画環境。
        transition_i: ロボットiの遷移。
        transition_j: ロボットjの遷移。
        robot_i: ロボットi ID。
        robot_j: ロボットj ID。
        title: タイトル。

    Returns:
        Figure と Axes。
    """
    fig, ax = plt.subplots(figsize=(7, 5))
    setup_axes(ax, env)
    draw_obstacles(ax, env)

    color_i = robot_color(robot_i)
    color_j = robot_color(robot_j)

    env_i = env.transition_envelope(
        from_node=transition_i.from_node,
        from_mode=transition_i.from_mode,
        to_node=transition_i.to_node,
        to_mode=transition_i.to_mode,
    )
    env_j = env.transition_envelope(
        from_node=transition_j.from_node,
        from_mode=transition_j.from_mode,
        to_node=transition_j.to_node,
        to_mode=transition_j.to_mode,
    )
    overlap = env_i & env_j

    draw_fine_cells(
        ax=ax,
        env=env,
        fine_cells=env_i,
        facecolor=color_i,
        edgecolor=color_i,
        alpha=0.18,
        linewidth=0.4,
        zorder=2,
    )
    draw_fine_cells(
        ax=ax,
        env=env,
        fine_cells=env_j,
        facecolor=color_j,
        edgecolor=color_j,
        alpha=0.18,
        linewidth=0.4,
        zorder=2,
    )
    draw_fine_cells(
        ax=ax,
        env=env,
        fine_cells=overlap,
        facecolor="red",
        edgecolor="red",
        alpha=0.45,
        linewidth=0.5,
        zorder=5,
    )

    draw_state_reference(
        ax=ax,
        env=env,
        state=TimedState(
            node=transition_i.from_node,
            mode=transition_i.from_mode,
            time=transition_i.time,
        ),
        color=color_i,
        label=f"R{robot_i} start",
        alpha_cells=0.28,
        alpha_marker=0.85,
        linestyle="--",
    )
    draw_state_reference(
        ax=ax,
        env=env,
        state=TimedState(
            node=transition_i.to_node,
            mode=transition_i.to_mode,
            time=transition_i.time + 1,
        ),
        color=color_i,
        label=f"R{robot_i} goal",
        alpha_cells=0.10,
        alpha_marker=0.45,
        linestyle="-",
    )

    draw_state_reference(
        ax=ax,
        env=env,
        state=TimedState(
            node=transition_j.from_node,
            mode=transition_j.from_mode,
            time=transition_j.time,
        ),
        color=color_j,
        label=f"R{robot_j} start",
        alpha_cells=0.28,
        alpha_marker=0.85,
        linestyle="--",
    )
    draw_state_reference(
        ax=ax,
        env=env,
        state=TimedState(
            node=transition_j.to_node,
            mode=transition_j.to_mode,
            time=transition_j.time + 1,
        ),
        color=color_j,
        label=f"R{robot_j} goal",
        alpha_cells=0.10,
        alpha_marker=0.45,
        linestyle="-",
    )

    if title is None:
        title = "Pair transition envelopes"
    ax.set_title(title)
    fig.tight_layout()
    return fig, ax


def get_transition_at_time_or_wait(
    path: Path,
    time_idx: int,
) -> Transition:
    """指定時刻の遷移を返す。

    終端後はその場WAIT遷移を返す。

    Args:
        path: 対象経路。
        time_idx: 参照時刻。

    Returns:
        遷移。
    """
    if time_idx < len(path.transitions):
        return path.transitions[time_idx]

    terminal_state = path.states[-1]
    return Transition(
        from_node=terminal_state.node,
        from_mode=terminal_state.mode,
        to_node=terminal_state.node,
        to_mode=terminal_state.mode,
        time=time_idx,
        action_type=path.transitions[-1].action_type
        if path.transitions
        else None,  # type: ignore[arg-type]
    )


def plot_path_transition_at_time(
    env: PlanningEnvironment,
    paths: Dict[int, Path],
    robot_id: int,
    time_idx: int,
) -> tuple[Figure, Axes]:
    """特定ロボットの特定時刻遷移 envelope を可視化する。

    Args:
        env: 計画環境。
        paths: ロボットID -> Path。
        robot_id: 対象ロボットID。
        time_idx: 対象時刻。

    Returns:
        Figure と Axes。
    """
    transition = get_transition_at_time_or_wait(paths[robot_id], time_idx)
    return plot_transition_envelope(
        env=env,
        transition=transition,
        robot_id=robot_id,
        title=f"R{robot_id} transition envelope at t={time_idx}",
    )


def plot_pair_path_transitions_at_time(
    env: PlanningEnvironment,
    paths: Dict[int, Path],
    robot_i: int,
    robot_j: int,
    time_idx: int,
) -> tuple[Figure, Axes]:
    """2ロボットの同時遷移 envelope を可視化する。

    Args:
        env: 計画環境。
        paths: ロボットID -> Path。
        robot_i: ロボットi ID。
        robot_j: ロボットj ID。
        time_idx: 対象時刻。

    Returns:
        Figure と Axes。
    """
    transition_i = get_transition_at_time_or_wait(paths[robot_i], time_idx)
    transition_j = get_transition_at_time_or_wait(paths[robot_j], time_idx)

    return plot_pair_transition_envelopes(
        env=env,
        transition_i=transition_i,
        transition_j=transition_j,
        robot_i=robot_i,
        robot_j=robot_j,
        title=f"Pair transition envelopes at t={time_idx}",
    )