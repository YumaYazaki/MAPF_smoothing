from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle

from environment import PlanningEnvironment
from geometry import rotate_relative_cell_discrete
from mapf_types import (
    FineCell,
    GridNode,
    OrientationMode,
    Primitive,
    RobotGeometrySpec,
)


@dataclass(frozen=True)
class PrimitiveSpec:
    """表示用 primitive 情報。

    Attributes:
        primitive: primitive 本体。
        name: 表示名。
    """

    primitive: Primitive
    name: str


def build_default_modes() -> list[OrientationMode]:
    """初期PoC用の4姿勢モードを返す。"""
    return [
        OrientationMode(0),
        OrientationMode(90),
        OrientationMode(180),
        OrientationMode(270),
    ]


def build_default_robot_geometry() -> RobotGeometrySpec:
    """左右対称・前後非対称な初期ロボット形状を返す。"""
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


def build_test_environment() -> PlanningEnvironment:
    """primitive envelope テスト用の広め環境を構築する。"""
    return PlanningEnvironment(
        rows=7,
        cols=7,
        obstacles=set(),
        modes=build_default_modes(),
        robot_geometry=build_default_robot_geometry(),
        enable_diagonal_move=False,
        enable_move_rotate=True,
        enable_reserved_regions=False,
    )


def enumerate_primitives() -> list[PrimitiveSpec]:
    """表示対象 primitive 一覧を返す。"""
    primitives: list[PrimitiveSpec] = [
        PrimitiveSpec(Primitive(0, 0, 0), "WAIT"),
        PrimitiveSpec(Primitive(0, 0, 90), "ROTATE +90"),
        PrimitiveSpec(Primitive(0, 0, -90), "ROTATE -90"),
        PrimitiveSpec(Primitive(-1, 0, 0), "MOVE UP"),
        PrimitiveSpec(Primitive(1, 0, 0), "MOVE DOWN"),
        PrimitiveSpec(Primitive(0, -1, 0), "MOVE LEFT"),
        PrimitiveSpec(Primitive(0, 1, 0), "MOVE RIGHT"),
        PrimitiveSpec(Primitive(-1, 0, 90), "MOVE UP + ROTATE +90"),
        PrimitiveSpec(Primitive(-1, 0, -90), "MOVE UP + ROTATE -90"),
        PrimitiveSpec(Primitive(1, 0, 90), "MOVE DOWN + ROTATE +90"),
        PrimitiveSpec(Primitive(1, 0, -90), "MOVE DOWN + ROTATE -90"),
        PrimitiveSpec(Primitive(0, -1, 90), "MOVE LEFT + ROTATE +90"),
        PrimitiveSpec(Primitive(0, -1, -90), "MOVE LEFT + ROTATE -90"),
        PrimitiveSpec(Primitive(0, 1, 90), "MOVE RIGHT + ROTATE +90"),
        PrimitiveSpec(Primitive(0, 1, -90), "MOVE RIGHT + ROTATE -90"),
    ]
    return primitives


def setup_axes(
    ax: Axes,
    env: PlanningEnvironment,
) -> None:
    """描画軸を初期化する。"""
    ax.set_xlim(0, env.cols)
    ax.set_ylim(env.rows, 0)
    ax.set_aspect("equal")
    ax.set_xticks(range(env.cols + 1))
    ax.set_yticks(range(env.rows + 1))
    ax.grid(True, which="both", linewidth=0.6)
    ax.set_xlabel("col")
    ax.set_ylabel("row")


def draw_fine_cells(
    ax: Axes,
    env: PlanningEnvironment,
    fine_cells: set[FineCell] | frozenset[FineCell],
    facecolor: str,
    edgecolor: str,
    alpha: float,
    linewidth: float,
    zorder: int,
) -> None:
    """fine cell 集合を描画する。"""
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


def local_delta_to_world(
    primitive: Primitive,
    start_mode: OrientationMode,
) -> tuple[int, int]:
    """primitive のローカル並進をワールド並進へ変換する。"""
    world_dr, world_dc = rotate_relative_cell_discrete(
        rel_row=primitive.dr,
        rel_col=primitive.dc,
        degree=start_mode.degree,
    )
    return world_dr, world_dc


def end_state_from_primitive(
    start_node: GridNode,
    start_mode: OrientationMode,
    primitive: Primitive,
) -> tuple[GridNode, OrientationMode]:
    """開始状態と primitive から終端状態を生成する。"""
    world_dr, world_dc = local_delta_to_world(
        primitive=primitive,
        start_mode=start_mode,
    )
    end_node = GridNode(
        row=start_node.row + world_dr,
        col=start_node.col + world_dc,
    )
    end_mode = OrientationMode(
        (start_mode.degree + primitive.dtheta) % 360
    )
    return end_node, end_mode


def draw_state_marker(
    ax: Axes,
    node: GridNode,
    mode: OrientationMode,
    color: str,
    label: str,
    alpha: float,
) -> None:
    """状態中心を描画する。"""
    x = node.col + 0.5
    y = node.row + 0.5
    ax.scatter(
        [x],
        [y],
        s=70,
        color=color,
        edgecolors="black",
        alpha=alpha,
        zorder=6,
    )
    ax.text(
        x,
        y - 0.18,
        f"{label}\n{mode.degree}°",
        ha="center",
        va="center",
        fontsize=8,
        color=color,
        alpha=alpha,
        zorder=7,
    )


def plot_single_primitive_for_mode(
    env: PlanningEnvironment,
    primitive_spec: PrimitiveSpec,
    start_mode: OrientationMode,
    start_node: GridNode,
    ax: Axes,
) -> None:
    """1 primitive を1開始姿勢で可視化する。"""
    primitive = primitive_spec.primitive
    goal_node, goal_mode = end_state_from_primitive(
        start_node=start_node,
        start_mode=start_mode,
        primitive=primitive,
    )

    start_occ = env.state_occupancy_key(start_node, start_mode)
    goal_occ = env.state_occupancy_key(goal_node, goal_mode)
    envelope = env.transition_envelope(
        from_node=start_node,
        from_mode=start_mode,
        to_node=goal_node,
        to_mode=goal_mode,
    )

    setup_axes(ax, env)

    draw_fine_cells(
        ax=ax,
        env=env,
        fine_cells=envelope,
        facecolor="tab:blue",
        edgecolor="tab:blue",
        alpha=0.18,
        linewidth=0.4,
        zorder=2,
    )
    draw_fine_cells(
        ax=ax,
        env=env,
        fine_cells=start_occ,
        facecolor="tab:orange",
        edgecolor="tab:orange",
        alpha=0.28,
        linewidth=0.5,
        zorder=3,
    )
    draw_fine_cells(
        ax=ax,
        env=env,
        fine_cells=goal_occ,
        facecolor="tab:green",
        edgecolor="tab:green",
        alpha=0.18,
        linewidth=0.5,
        zorder=3,
    )

    draw_state_marker(
        ax=ax,
        node=start_node,
        mode=start_mode,
        color="tab:orange",
        label="start",
        alpha=0.9,
    )
    draw_state_marker(
        ax=ax,
        node=goal_node,
        mode=goal_mode,
        color="tab:green",
        label="goal",
        alpha=0.6,
    )

    ax.set_title(f"{primitive_spec.name}\nstart={start_mode.degree}°")


def plot_all_primitives_all_modes(
    env: PlanningEnvironment,
    primitive_specs: list[PrimitiveSpec],
) -> None:
    """すべての primitive を全開始姿勢で可視化する。

    primitive ごとに 2x2 の subplot を1枚出す。
    """
    start_node = GridNode(row=3, col=3)
    modes = build_default_modes()

    for primitive_spec in primitive_specs:
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(
            f"Primitive envelope debug: {primitive_spec.name}",
            fontsize=14,
        )

        for ax, mode in zip(axes.ravel(), modes):
            plot_single_primitive_for_mode(
                env=env,
                primitive_spec=primitive_spec,
                start_mode=mode,
                start_node=start_node,
                ax=ax,
            )

        fig.tight_layout()
        plt.show()


def plot_all_primitives_canonical(
    env: PlanningEnvironment,
    primitive_specs: list[PrimitiveSpec],
) -> None:
    """すべての primitive を canonical な開始姿勢 0° で一覧表示する。

    一覧性を重視して 3 列で並べる。
    """
    start_node = GridNode(row=3, col=3)
    start_mode = OrientationMode(0)

    n = len(primitive_specs)
    ncols = 3
    nrows = (n + ncols - 1) // ncols

    fig, axes = plt.subplots(
        nrows,
        ncols,
        figsize=(5 * ncols, 4 * nrows),
    )
    axes_list = axes.ravel() if hasattr(axes, "ravel") else [axes]

    for ax, primitive_spec in zip(axes_list, primitive_specs):
        plot_single_primitive_for_mode(
            env=env,
            primitive_spec=primitive_spec,
            start_mode=start_mode,
            start_node=start_node,
            ax=ax,
        )

    for ax in axes_list[len(primitive_specs):]:
        ax.axis("off")

    fig.suptitle("All primitive envelopes (canonical start=0°)", fontsize=14)
    fig.tight_layout()
    plt.show()


def main() -> None:
    """エントリポイント。"""
    env = build_test_environment()
    primitive_specs = enumerate_primitives()

    # まず canonical 一覧で全 primitive をざっと確認
    plot_all_primitives_canonical(
        env=env,
        primitive_specs=primitive_specs,
    )

    # 次に primitive ごとに 4 姿勢すべて確認
    plot_all_primitives_all_modes(
        env=env,
        primitive_specs=primitive_specs,
    )


if __name__ == "__main__":
    main()