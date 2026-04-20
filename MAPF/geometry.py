from __future__ import annotations

import math
from dataclasses import dataclass
from typing import FrozenSet

from mapf_types import FineCell, GridNode, OrientationMode, Primitive, RobotGeometrySpec


@dataclass(frozen=True)
class FinePose:
    """細グリッド上の姿勢付き基準位置。

    Attributes:
        row_center: 基準点の細グリッド行座標。
        col_center: 基準点の細グリッド列座標。
        angle_deg: 連続角度 [deg]。
    """

    row_center: float
    col_center: float
    angle_deg: float


def grid_node_center_in_fine(
    node: GridNode,
    fine_scale: int,
) -> tuple[float, float]:
    """MAPFノード中心を細グリッド座標へ変換する。"""
    row_center = node.row * fine_scale + fine_scale / 2.0
    col_center = node.col * fine_scale + fine_scale / 2.0
    return row_center, col_center


def rotate_relative_cell_discrete(
    rel_row: int,
    rel_col: int,
    degree: int,
) -> FineCell:
    """相対セル座標を90度単位で厳密回転する。"""
    deg = degree % 360
    if deg == 0:
        return rel_row, rel_col
    if deg == 90:
        return -rel_col, rel_row
    if deg == 180:
        return -rel_row, -rel_col
    if deg == 270:
        return rel_col, -rel_row
    raise ValueError(f"Unsupported rotation degree: {degree}")


def rotate_point(
    rel_row: float,
    rel_col: float,
    angle_deg: float,
) -> tuple[float, float]:
    """相対座標点を任意角度だけ回転する。

    この系では row は下向きが正なので、離散回転
    rotate_relative_cell_discrete() と整合するように定義する。

    具体的には +90° で
        (rel_row, rel_col) -> (-rel_col, rel_row)
    となるようにする。

    Args:
        rel_row: 相対行座標。
        rel_col: 相対列座標。
        angle_deg: 回転角度 [deg]。

    Returns:
        回転後の相対座標。
    """
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    rot_row = cos_t * rel_row - sin_t * rel_col
    rot_col = sin_t * rel_row + cos_t * rel_col
    return rot_row, rot_col


def shortest_angle_delta_deg(
    from_deg: float,
    to_deg: float,
) -> float:
    """最短回転方向での角度差を返す。"""
    return (to_deg - from_deg + 180.0) % 360.0 - 180.0


def interpolate_angle_deg(
    from_deg: float,
    to_deg: float,
    alpha: float,
) -> float:
    """最短回転方向で角度補間する。"""
    delta = shortest_angle_delta_deg(from_deg, to_deg)
    return from_deg + alpha * delta


def state_occupancy(
    node: GridNode,
    mode: OrientationMode,
    geometry: RobotGeometrySpec,
) -> FrozenSet[FineCell]:
    """離散姿勢での厳密な占有セル集合を返す。"""
    row_center, col_center = grid_node_center_in_fine(
        node=node,
        fine_scale=geometry.fine_scale,
    )

    cells: set[FineCell] = set()
    for rel_row, rel_col in geometry.occupied_cells_at_zero_deg:
        rot_row, rot_col = rotate_relative_cell_discrete(
            rel_row=rel_row,
            rel_col=rel_col,
            degree=mode.degree,
        )
        cells.add(
            (
                int(round(row_center + rot_row)),
                int(round(col_center + rot_col)),
            )
        )

    return frozenset(cells)


def subcell_offsets(
    samples_per_axis: int,
) -> list[float]:
    """1セル内部のサンプル点オフセットを返す。"""
    if samples_per_axis < 1:
        raise ValueError("samples_per_axis must be >= 1")

    if samples_per_axis == 1:
        return [0.0]

    step = 1.0 / samples_per_axis
    start = -0.5 + step / 2.0
    return [start + idx * step for idx in range(samples_per_axis)]


def conservative_cell_index(
    value: float,
) -> int:
    """連続座標を含む fine cell index へ変換する。"""
    return int(math.floor(value + 0.5))


def occupancy_at_pose_sampled(
    pose: FinePose,
    geometry: RobotGeometrySpec,
    samples_per_axis: int = 3,
) -> FrozenSet[FineCell]:
    """連続姿勢での occupancy をサンプルベースで近似する。

    これは envelope 事前計算用であり、runtime の state 表示には使わない。
    """
    cells: set[FineCell] = set()
    offsets = subcell_offsets(samples_per_axis)

    for rel_row, rel_col in geometry.occupied_cells_at_zero_deg:
        for off_row in offsets:
            for off_col in offsets:
                sample_row = rel_row + off_row
                sample_col = rel_col + off_col
                rot_row, rot_col = rotate_point(
                    rel_row=sample_row,
                    rel_col=sample_col,
                    angle_deg=pose.angle_deg,
                )
                abs_row = pose.row_center + rot_row
                abs_col = pose.col_center + rot_col
                cells.add(
                    (
                        conservative_cell_index(abs_row),
                        conservative_cell_index(abs_col),
                    )
                )

    return frozenset(cells)


def interpolate_pose(
    primitive: Primitive,
    geometry: RobotGeometrySpec,
    alpha: float,
) -> FinePose:
    """canonical primitive に対する補間姿勢を返す。

    始点は基準点原点・角度0度とする。
    """
    fine_scale = geometry.fine_scale
    row_center = alpha * primitive.dr * fine_scale
    col_center = alpha * primitive.dc * fine_scale
    angle_deg = interpolate_angle_deg(
        from_deg=0.0,
        to_deg=float(primitive.dtheta),
        alpha=alpha,
    )
    return FinePose(
        row_center=row_center,
        col_center=col_center,
        angle_deg=angle_deg,
    )


def sample_alphas(
    num_samples: int,
) -> list[float]:
    """[0, 1] 区間のサンプル点列を返す。"""
    if num_samples < 2:
        raise ValueError("num_samples must be >= 2")
    return [idx / (num_samples - 1) for idx in range(num_samples)]


def generate_relative_envelope(
    primitive: Primitive,
    geometry: RobotGeometrySpec,
    num_samples: int,
    subcell_samples: int,
) -> FrozenSet[FineCell]:
    """primitive の relative swept envelope を生成する。"""
    cells: set[FineCell] = set()

    for alpha in sample_alphas(num_samples):
        pose = interpolate_pose(
            primitive=primitive,
            geometry=geometry,
            alpha=alpha,
        )
        occ = occupancy_at_pose_sampled(
            pose=pose,
            geometry=geometry,
            samples_per_axis=subcell_samples,
        )
        cells.update(occ)

    return frozenset(cells)


def transform_relative_cells(
    relative_cells: FrozenSet[FineCell],
    node: GridNode,
    mode: OrientationMode,
    geometry: RobotGeometrySpec,
) -> FrozenSet[FineCell]:
    """relative fine cell 集合を絶対 fine cell 集合へ変換する。"""
    row_center, col_center = grid_node_center_in_fine(
        node=node,
        fine_scale=geometry.fine_scale,
    )

    cells: set[FineCell] = set()
    for rel_row, rel_col in relative_cells:
        rot_row, rot_col = rotate_relative_cell_discrete(
            rel_row=rel_row,
            rel_col=rel_col,
            degree=mode.degree,
        )
        cells.add(
            (
                int(round(row_center + rot_row)),
                int(round(col_center + rot_col)),
            )
        )

    return frozenset(cells)