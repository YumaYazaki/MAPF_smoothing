from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, Tuple

from geometry import (
    generate_relative_envelope,
    state_occupancy,
    transform_relative_cells,
    rotate_relative_cell_discrete, 
)
from mapf_types import (
    Action,
    ActionType,
    FineCell,
    GridNode,
    OrientationMode,
    Primitive,
    RobotGeometrySpec,
)


StateCacheKey = Tuple[GridNode, OrientationMode]
TransitionKey = Tuple[GridNode, OrientationMode, GridNode, OrientationMode]


@dataclass
class PlanningEnvironment:
    """姿勢付きMAPF用の計画環境。

    Attributes:
        rows: MAPFグリッド行数。
        cols: MAPFグリッド列数。
        obstacles: 静的障害物セル集合。
        modes: 利用する姿勢モード集合。
        robot_geometry: ロボット形状仕様。
        enable_diagonal_move: 斜め移動を許可するか。
        enable_move_rotate: move+rotate を許可するか。
        enable_reserved_regions: 予約領域を有効にするか。
        reserved_region_map: 細グリッドセル -> 領域ID の写像。
    """

    rows: int
    cols: int
    obstacles: set[tuple[int, int]]
    modes: list[OrientationMode]
    robot_geometry: RobotGeometrySpec
    enable_diagonal_move: bool = False
    enable_move_rotate: bool = True
    enable_reserved_regions: bool = False
    reserved_region_map: Dict[FineCell, int] = field(default_factory=dict)

    _state_cache: Dict[StateCacheKey, FrozenSet[FineCell]] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    _transition_envelope_cache: Dict[TransitionKey, FrozenSet[FineCell]] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    _relative_primitive_envelope_cache: Dict[Primitive, FrozenSet[FineCell]] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    _region_cache: Dict[TransitionKey, FrozenSet[int]] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    _obstacle_fine_cells_cache: FrozenSet[FineCell] = field(
        default_factory=frozenset,
        init=False,
        repr=False,
    )

    def __post_init__(self) -> None:
        self._rebuild_obstacle_fine_cells()

    def clear_caches(self) -> None:
        """内部キャッシュを消去する。"""
        self._state_cache.clear()
        self._transition_envelope_cache.clear()
        self._relative_primitive_envelope_cache.clear()
        self._region_cache.clear()
        self._rebuild_obstacle_fine_cells()

    def _rebuild_obstacle_fine_cells(self) -> None:
        """障害物セルの細グリッド表現を再構築する。"""
        fine_scale = self.robot_geometry.fine_scale
        obstacle_fine_cells: set[FineCell] = set()

        for row, col in self.obstacles:
            for dr in range(fine_scale):
                for dc in range(fine_scale):
                    obstacle_fine_cells.add(
                        (
                            row * fine_scale + dr,
                            col * fine_scale + dc,
                        )
                    )

        self._obstacle_fine_cells_cache = frozenset(obstacle_fine_cells)

    def get_obstacle_fine_cells(self) -> FrozenSet[FineCell]:
        """障害物の細グリッド占有集合を返す。"""
        return self._obstacle_fine_cells_cache

    def in_bounds(self, node: GridNode) -> bool:
        """ノードがMAPFグリッド範囲内か判定する。"""
        return 0 <= node.row < self.rows and 0 <= node.col < self.cols

    def is_static_cell_free(self, node: GridNode) -> bool:
        """ノード位置が静的障害物でないか判定する。"""
        return (node.row, node.col) not in self.obstacles

    def state_occupancy_key(
        self,
        node: GridNode,
        mode: OrientationMode,
    ) -> FrozenSet[FineCell]:
        """状態 occupancy を返す。"""
        key = (node, mode)
        cached = self._state_cache.get(key)
        if cached is not None:
            return cached

        occ = state_occupancy(
            node=node,
            mode=mode,
            geometry=self.robot_geometry,
        )
        self._state_cache[key] = occ
        return occ

    def infer_primitive(
        self,
        from_node: GridNode,
        from_mode: OrientationMode,
        to_node: GridNode,
        to_mode: OrientationMode,
    ) -> Primitive:
        """遷移から primitive を同定する。

        primitive の並進成分は、開始姿勢基準のローカル座標系で表す。

        Args:
            from_node: 始点ノード。
            from_mode: 始点姿勢。
            to_node: 終点ノード。
            to_mode: 終点姿勢。

        Returns:
            ローカル座標系で表現された primitive。
        """
        world_dr = to_node.row - from_node.row
        world_dc = to_node.col - from_node.col

        # ワールド差分を、始点姿勢基準のローカル差分へ変換
        local_dr, local_dc = rotate_relative_cell_discrete(
            rel_row=world_dr,
            rel_col=world_dc,
            degree=(-from_mode.degree) % 360,
        )

        dtheta = (to_mode.degree - from_mode.degree + 180) % 360 - 180

        if dtheta not in (0, 90, -90):
            raise ValueError(
                f"Unsupported primitive rotation delta: {dtheta}"
            )

        return Primitive(
            dr=local_dr,
            dc=local_dc,
            dtheta=dtheta,
        )

    def _default_num_samples_for_primitive(
        self,
        primitive: Primitive,
    ) -> int:
        """primitive に応じた envelope 生成サンプル数を返す。"""
        is_translation = primitive.dr != 0 or primitive.dc != 0
        is_rotation = primitive.dtheta != 0

        if not is_translation and not is_rotation:
            return 2
        if is_translation and not is_rotation:
            return 11
        if not is_translation and is_rotation:
            return 41
        return 65

    def _default_subcell_samples_for_primitive(
        self,
        primitive: Primitive,
    ) -> int:
        """primitive に応じたセル内サンプル数を返す。"""
        is_rotation = primitive.dtheta != 0
        return 5 if is_rotation else 3

    def relative_primitive_envelope(
        self,
        primitive: Primitive,
    ) -> FrozenSet[FineCell]:
        """primitive の relative swept envelope を返す。"""
        cached = self._relative_primitive_envelope_cache.get(primitive)
        if cached is not None:
            return cached

        env = generate_relative_envelope(
            primitive=primitive,
            geometry=self.robot_geometry,
            num_samples=self._default_num_samples_for_primitive(primitive),
            subcell_samples=self._default_subcell_samples_for_primitive(
                primitive
            ),
        )
        self._relative_primitive_envelope_cache[primitive] = env
        return env

    def transition_envelope(
        self,
        from_node: GridNode,
        from_mode: OrientationMode,
        to_node: GridNode,
        to_mode: OrientationMode,
    ) -> FrozenSet[FineCell]:
        """遷移 envelope を返す。"""
        key = (from_node, from_mode, to_node, to_mode)
        cached = self._transition_envelope_cache.get(key)
        if cached is not None:
            return cached

        primitive = self.infer_primitive(
            from_node=from_node,
            from_mode=from_mode,
            to_node=to_node,
            to_mode=to_mode,
        )
        relative_env = self.relative_primitive_envelope(primitive)
        absolute_env = transform_relative_cells(
            relative_cells=relative_env,
            node=from_node,
            mode=from_mode,
            geometry=self.robot_geometry,
        )
        self._transition_envelope_cache[key] = absolute_env
        return absolute_env

    def sweep_occupancy_key(
        self,
        from_node: GridNode,
        from_mode: OrientationMode,
        to_node: GridNode,
        to_mode: OrientationMode,
    ) -> FrozenSet[FineCell]:
        """後方互換用。transition envelope を返す。"""
        return self.transition_envelope(
            from_node=from_node,
            from_mode=from_mode,
            to_node=to_node,
            to_mode=to_mode,
        )

    def is_state_valid(
        self,
        node: GridNode,
        mode: OrientationMode,
    ) -> bool:
        """状態が有効か判定する。"""
        if not self.in_bounds(node):
            return False
        if not self.is_static_cell_free(node):
            return False

        occ = self.state_occupancy_key(node, mode)
        if not self._all_fine_cells_in_workspace(occ):
            return False
        if self._intersects_obstacle_fine_cells(occ):
            return False
        return True

    def is_transition_valid(
        self,
        from_node: GridNode,
        from_mode: OrientationMode,
        to_node: GridNode,
        to_mode: OrientationMode,
    ) -> bool:
        """遷移が有効か判定する。"""
        if not self.is_state_valid(from_node, from_mode):
            return False
        if not self.is_state_valid(to_node, to_mode):
            return False

        env = self.transition_envelope(
            from_node=from_node,
            from_mode=from_mode,
            to_node=to_node,
            to_mode=to_mode,
        )
        if not self._all_fine_cells_in_workspace(env):
            return False
        if self._intersects_obstacle_fine_cells(env):
            return False
        return True

    def reserved_regions_for_transition(
        self,
        from_node: GridNode,
        from_mode: OrientationMode,
        to_node: GridNode,
        to_mode: OrientationMode,
    ) -> FrozenSet[int]:
        """遷移中に占有する予約領域ID集合を返す。"""
        if not self.enable_reserved_regions:
            return frozenset()

        key = (from_node, from_mode, to_node, to_mode)
        cached = self._region_cache.get(key)
        if cached is not None:
            return cached

        env_cells = self.transition_envelope(
            from_node=from_node,
            from_mode=from_mode,
            to_node=to_node,
            to_mode=to_mode,
        )
        region_ids = {
            self.reserved_region_map[cell]
            for cell in env_cells
            if cell in self.reserved_region_map
        }
        result = frozenset(region_ids)
        self._region_cache[key] = result
        return result

    def generate_actions(
        self,
        node: GridNode,
        mode: OrientationMode,
    ) -> list[Action]:
        """状態から許容される1ステップ遷移候補を返す。"""
        actions: list[Action] = []

        if self.is_transition_valid(node, mode, node, mode):
            actions.append(
                Action(
                    action_type=ActionType.WAIT,
                    next_node=node,
                    next_mode=mode,
                )
            )

        for next_mode in self._next_rotation_modes(mode):
            if self.is_transition_valid(node, mode, node, next_mode):
                actions.append(
                    Action(
                        action_type=ActionType.ROTATE,
                        next_node=node,
                        next_mode=next_mode,
                    )
                )

        for next_node in self._neighbor_nodes(node):
            if self.is_transition_valid(node, mode, next_node, mode):
                actions.append(
                    Action(
                        action_type=ActionType.MOVE,
                        next_node=next_node,
                        next_mode=mode,
                    )
                )

            if self.enable_move_rotate:
                for next_mode in self._next_rotation_modes(mode):
                    if self.is_transition_valid(node, mode, next_node, next_mode):
                        actions.append(
                            Action(
                                action_type=ActionType.MOVE_ROTATE,
                                next_node=next_node,
                                next_mode=next_mode,
                            )
                        )

        return actions

    def _neighbor_nodes(
        self,
        node: GridNode,
    ) -> list[GridNode]:
        """隣接ノードを返す。"""
        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.enable_diagonal_move:
            offsets.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])

        neighbors: list[GridNode] = []
        for dr, dc in offsets:
            nxt = GridNode(row=node.row + dr, col=node.col + dc)
            if self.in_bounds(nxt) and self.is_static_cell_free(nxt):
                neighbors.append(nxt)
        return neighbors

    def _next_rotation_modes(
        self,
        mode: OrientationMode,
    ) -> list[OrientationMode]:
        """1ステップで許容する回転先姿勢を返す。"""
        mode_set = {m.degree for m in self.modes}
        next_modes: list[OrientationMode] = []

        for delta in (-90, 90):
            new_deg = (mode.degree + delta) % 360
            if new_deg in mode_set:
                next_modes.append(OrientationMode(new_deg))

        return next_modes

    def _all_fine_cells_in_workspace(
        self,
        cells: FrozenSet[FineCell],
    ) -> bool:
        """細グリッドセル集合がワークスペース内に収まるか判定する。"""
        max_row = self.rows * self.robot_geometry.fine_scale
        max_col = self.cols * self.robot_geometry.fine_scale

        for row, col in cells:
            if not (0 <= row < max_row and 0 <= col < max_col):
                return False
        return True

    def _intersects_obstacle_fine_cells(
        self,
        cells: FrozenSet[FineCell],
    ) -> bool:
        """占有セル集合が障害物細グリッドと交差するか判定する。"""
        return bool(cells & self._obstacle_fine_cells_cache)