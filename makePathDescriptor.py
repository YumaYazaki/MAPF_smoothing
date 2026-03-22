from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Literal, Optional, Tuple, Union
import math
import numpy as np


Vector2 = Tuple[float, float]
GeometryType = Literal["line", "quintic_corner"]
SegmentType = Literal[
    "straight_like",
    "turning",
    "coordination",
    "positioning",
    "generic",
]

# 区間evaluatorが返す共通の出力を定義。line/quintic両方で同じ形式で返す。
@dataclass(frozen=True)
class GeometryAtS:
    """Geometry evaluated at arc-length parameter s.

    Attributes:
        s_local: Local arc length inside the segment.
        position: Position p(s).
        tangent: Unit tangent dr/ds.
        curvature: Curvature kappa(s).
    """

    s_local: float
    position: Vector2
    tangent: Vector2
    curvature: float


# line segment evaluator
# 直線区間は解析的に求まるので、内部パラメータは不要でそのまま出力してよい。
@dataclass(frozen=True)
class LineSegmentEvaluator:
    """Evaluator for one straight line segment."""

    p_start: Vector2
    p_end: Vector2
    length: float
    tangent: Vector2

    @staticmethod
    def build(p_start: Vector2, p_end: Vector2) -> "LineSegmentEvaluator":
        """Build line evaluator from endpoints."""
        dx = p_end[0] - p_start[0]
        dy = p_end[1] - p_start[1]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            raise ValueError("Line segment length must be positive.")
        tangent = (dx / length, dy / length)
        return LineSegmentEvaluator(
            p_start=p_start,
            p_end=p_end,
            length=length,
            tangent=tangent,
        )

    def evaluate_at_s_local(self, s_local: float) -> GeometryAtS:
        """Evaluate line geometry at local arc length."""
        s = _clamp(s_local, 0.0, self.length)
        position = (
            self.p_start[0] + s * self.tangent[0],
            self.p_start[1] + s * self.tangent[1],
        )
        return GeometryAtS(
            s_local=s,
            position=position,
            tangent=self.tangent,
            curvature=0.0,
        )

# quinitic corner evaluator
## quintic curveの定義
@dataclass(frozen=True)
class QuinticPolynomial1D:
    """Represents a 1D quintic polynomial."""

    a0: float
    a1: float
    a2: float
    a3: float
    a4: float
    a5: float

    def value(self, u: float) -> float:
        return (
            self.a0
            + self.a1 * u
            + self.a2 * u**2
            + self.a3 * u**3
            + self.a4 * u**4
            + self.a5 * u**5
        )

    def first_derivative(self, u: float) -> float:
        return (
            self.a1
            + 2.0 * self.a2 * u
            + 3.0 * self.a3 * u**2
            + 4.0 * self.a4 * u**3
            + 5.0 * self.a5 * u**4
        )

    def second_derivative(self, u: float) -> float:
        return (
            2.0 * self.a2
            + 6.0 * self.a3 * u
            + 12.0 * self.a4 * u**2
            + 20.0 * self.a5 * u**3
        )


@dataclass(frozen=True)
class QuinticCornerCurve:
    """Local quintic corner smoothing curve in 2D."""

    corner: Vector2
    e_in: Vector2
    e_out: Vector2
    x_poly: QuinticPolynomial1D
    y_poly: QuinticPolynomial1D
    q_minus: Vector2
    q_plus: Vector2

    def point(self, u: float) -> Vector2:
        x_local = self.x_poly.value(u)
        y_local = self.y_poly.value(u)
        return _add(
            self.corner,
            _add(_scale(self.e_in, x_local), _scale(self.e_out, y_local)),
        )

    def first_derivative(self, u: float) -> Vector2:
        dx = self.x_poly.first_derivative(u)
        dy = self.y_poly.first_derivative(u)
        return _add(_scale(self.e_in, dx), _scale(self.e_out, dy))

    def second_derivative(self, u: float) -> Vector2:
        ddx = self.x_poly.second_derivative(u)
        ddy = self.y_poly.second_derivative(u)
        return _add(_scale(self.e_in, ddx), _scale(self.e_out, ddy))

    def speed_norm(self, u: float) -> float:
        dx, dy = self.first_derivative(u)
        return math.hypot(dx, dy)

    def curvature(self, u: float, eps: float = 1e-12) -> float:
        dx, dy = self.first_derivative(u)
        ddx, ddy = self.second_derivative(u)
        numerator = dx * ddy - dy * ddx
        denominator = (dx * dx + dy * dy) ** 1.5
        if denominator < eps:
            return 0.0
        return numerator / denominator
    

@dataclass(frozen=True)
class QuinticTurnParameters:
    """Parameters for quintic local smoothing at a corner.

    Attributes:
        d: Connection length from the corner along incoming/outgoing segments.
        lam: Tangent magnitude parameter of the quintic corner curve.
    """

    d: float
    lam: float
    
## u(s)evaluatorを内包した、segment evaluator
@dataclass(frozen=True)
class QuinticCornerSegmentEvaluator:
    """Evaluator for one quintic corner segment.

    The evaluator uses:
        - analytic quintic geometry in u
        - monotone s(u) table
        - hybrid Newton + bisection for u(s)
    """

    curve: QuinticCornerCurve
    length: float
    table_u: np.ndarray
    table_s: np.ndarray
    quadrature_order: int = 16

    def evaluate_at_s_local(
        self,
        s_local: float,
        tol_s: float = 1e-10,
        tol_u: float = 1e-12,
        max_iter: int = 30,
    ) -> GeometryAtS:
        """Evaluate quintic corner geometry at local arc length."""
        u_value = self.evaluate_u_from_s(
            s_local=s_local,
            tol_s=tol_s,
            tol_u=tol_u,
            max_iter=max_iter,
        )

        position = self.curve.point(u_value)
        du1 = self.curve.first_derivative(u_value)
        speed = math.hypot(du1[0], du1[1])

        if speed > 0.0:
            tangent = (du1[0] / speed, du1[1] / speed)
        else:
            tangent = (0.0, 0.0)

        curvature = self.curve.curvature(u_value)

        return GeometryAtS(
            s_local=_clamp(s_local, 0.0, self.length),
            position=position,
            tangent=tangent,
            curvature=curvature,
        )

    def evaluate_u_from_s(
        self,
        s_local: float,
        tol_s: float = 1e-10,
        tol_u: float = 1e-12,
        max_iter: int = 30,
    ) -> float:
        """Evaluate u(s) by hybrid Newton + bisection."""
        if s_local <= 0.0:
            return 0.0
        if s_local >= self.length:
            return 1.0

        i_left, i_right = bracket_from_table(self.table_s, s_local)

        u_left = float(self.table_u[i_left])
        u_right = float(self.table_u[i_right])
        s_left = float(self.table_s[i_left])
        s_right = float(self.table_s[i_right])

        if s_right > s_left:
            alpha = (s_local - s_left) / (s_right - s_left)
        else:
            alpha = 0.0
        u = u_left + alpha * (u_right - u_left)

        for _ in range(max_iter):
            s_u = self.arc_length_u(u)
            f_u = s_u - s_local

            if abs(f_u) <= tol_s:
                return _clamp(u, 0.0, 1.0)

            speed = self.curve.speed_norm(u)

            u_candidate = 0.5 * (u_left + u_right)
            if speed > 0.0:
                u_newton = u - f_u / speed
                if u_left < u_newton < u_right:
                    u_candidate = u_newton

            s_candidate = self.arc_length_u(u_candidate)
            f_candidate = s_candidate - s_local

            if f_candidate > 0.0:
                u_right = u_candidate
            else:
                u_left = u_candidate

            u = u_candidate

            if abs(u_right - u_left) <= tol_u:
                return _clamp(0.5 * (u_left + u_right), 0.0, 1.0)

        return _clamp(0.5 * (u_left + u_right), 0.0, 1.0)

    def arc_length_u(self, u: float) -> float:
        """Evaluate s(u) by Gauss-Legendre quadrature."""
        if u <= 0.0:
            return 0.0
        if u >= 1.0:
            u = 1.0

        nodes, weights = np.polynomial.legendre.leggauss(self.quadrature_order)
        midpoint = 0.5 * u
        half = 0.5 * u

        total = 0.0
        for node, weight in zip(nodes, weights):
            xi = midpoint + half * float(node)
            total += float(weight) * self.curve.speed_norm(xi)

        return half * total
    
# segment wrapper
# line/quintic両方をSegmentEvaluatorでラップして、同じインターフェースで扱えるようにする。
SegmentEvaluatorType = Union[LineSegmentEvaluator, QuinticCornerSegmentEvaluator]


@dataclass(frozen=True)
class PathSegment:
    """One segment in the whole path descriptor."""

    segment_id: int
    geometry_type: GeometryType
    segment_type: SegmentType
    s_start: float
    s_end: float
    evaluator: SegmentEvaluatorType

    @property
    def length(self) -> float:
        """Segment length."""
        return self.s_end - self.s_start

    def evaluate_at_s_global(self, s_global: float) -> GeometryAtS:
        """Evaluate geometry at global arc length."""
        s_local = s_global - self.s_start
        return self.evaluator.evaluate_at_s_local(s_local)
    
# PathDescriptor本体
@dataclass(frozen=True)
class PathHeader:
    """Whole-path metadata."""

    path_id: str
    version: str
    frame_id: str
    total_length: float


@dataclass(frozen=True)
class PathDescriptor:
    """Whole path descriptor with segment evaluators."""

    header: PathHeader
    segments: List[PathSegment] = field(default_factory=list)

    def find_segment_index(self, s_global: float) -> int:
        """Find the segment containing s_global."""
        if s_global <= 0.0:
            return 0
        if s_global >= self.header.total_length:
            return len(self.segments) - 1

        for idx, segment in enumerate(self.segments):
            if segment.s_start <= s_global <= segment.s_end:
                return idx

        # Fallback for numerical boundary issues
        return len(self.segments) - 1

    def evaluate_at_s(self, s_global: float) -> GeometryAtS:
        """Evaluate whole-path geometry at global arc length."""
        s_clamped = _clamp(s_global, 0.0, self.header.total_length)
        segment_idx = self.find_segment_index(s_clamped)
        return self.segments[segment_idx].evaluate_at_s_global(s_clamped)

# quintic evalueatorの構築関数
def build_quintic_corner_segment_evaluator(
    curve: QuinticCornerCurve,
    num_table_points: int = 201,
    quadrature_order: int = 16,
) -> QuinticCornerSegmentEvaluator:
    """Build quintic corner evaluator with s(u) table."""
    if num_table_points < 2:
        raise ValueError("num_table_points must be at least 2.")

    table_u = np.linspace(0.0, 1.0, num_table_points)
    table_s = np.zeros(num_table_points, dtype=float)

    temp_eval = QuinticCornerSegmentEvaluator(
        curve=curve,
        length=0.0,
        table_u=table_u,
        table_s=table_s,
        quadrature_order=quadrature_order,
    )

    for idx in range(1, num_table_points):
        table_s[idx] = temp_eval.arc_length_u(float(table_u[idx]))

    length = float(table_s[-1])

    if not np.all(np.diff(table_u) > 0.0):
        raise ValueError("table_u must be strictly increasing.")
    if not np.all(np.diff(table_s) > 0.0):
        raise ValueError("table_s must be strictly increasing.")

    return QuinticCornerSegmentEvaluator(
        curve=curve,
        length=length,
        table_u=table_u,
        table_s=table_s,
        quadrature_order=quadrature_order,
    )

# 手でパスを作成する例
def build_example_path_descriptor() -> PathDescriptor:
    """Build a simple example path with line + quintic corner + line."""
    p_prev = (-2.0, 0.0)
    p_corner = (0.0, 0.0)
    p_next = (0.0, 2.0)

    d = 0.8
    lam = 1.2

    curve = build_quintic_corner_curve(
        p_prev=p_prev,
        p_corner=p_corner,
        p_next=p_next,
        params=QuinticTurnParameters(d=d, lam=lam),
    )

    line_before = LineSegmentEvaluator.build(
        p_start=p_prev,
        p_end=curve.q_minus,
    )
    corner_eval = build_quintic_corner_segment_evaluator(
        curve=curve,
        num_table_points=301,
        quadrature_order=20,
    )
    line_after = LineSegmentEvaluator.build(
        p_start=curve.q_plus,
        p_end=p_next,
    )

    s0 = 0.0
    s1 = s0 + line_before.length
    s2 = s1 + corner_eval.length
    s3 = s2 + line_after.length

    segments = [
        PathSegment(
            segment_id=0,
            geometry_type="line",
            segment_type="straight_like",
            s_start=s0,
            s_end=s1,
            evaluator=line_before,
        ),
        PathSegment(
            segment_id=1,
            geometry_type="quintic_corner",
            segment_type="turning",
            s_start=s1,
            s_end=s2,
            evaluator=corner_eval,
        ),
        PathSegment(
            segment_id=2,
            geometry_type="line",
            segment_type="straight_like",
            s_start=s2,
            s_end=s3,
            evaluator=line_after,
        ),
    ]

    return PathDescriptor(
        header=PathHeader(
            path_id="example_path_001",
            version="1.0",
            frame_id="world",
            total_length=s3,
        ),
        segments=segments,
    )


# Utility functions
def bracket_from_table(table_s: np.ndarray, s_query: float) -> Tuple[int, int]:
    """Find bracket indices such that table_s[i] <= s_query <= table_s[i+1]."""
    if s_query <= float(table_s[0]):
        return 0, 1
    if s_query >= float(table_s[-1]):
        return len(table_s) - 2, len(table_s) - 1

    right = int(np.searchsorted(table_s, s_query, side="right"))
    left = right - 1
    return left, right


def _add(a: Vector2, b: Vector2) -> Vector2:
    return (a[0] + b[0], a[1] + b[1])


def _sub(a: Vector2, b: Vector2) -> Vector2:
    return (a[0] - b[0], a[1] - b[1])


def _scale(v: Vector2, s: float) -> Vector2:
    return (v[0] * s, v[1] * s)


def _dot(a: Vector2, b: Vector2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _norm(v: Vector2) -> float:
    return math.hypot(v[0], v[1])


def _normalize(v: Vector2) -> Vector2:
    n = _norm(v)
    if n == 0.0:
        raise ValueError("Cannot normalize a zero vector.")
    return (v[0] / n, v[1] / n)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _build_local_x_polynomial(d: float, lam: float) -> QuinticPolynomial1D:
    return QuinticPolynomial1D(
        a0=-d,
        a1=lam,
        a2=0.0,
        a3=10.0 * d - 6.0 * lam,
        a4=-15.0 * d + 8.0 * lam,
        a5=6.0 * d - 3.0 * lam,
    )


def _build_local_y_polynomial(d: float, lam: float) -> QuinticPolynomial1D:
    return QuinticPolynomial1D(
        a0=0.0,
        a1=0.0,
        a2=0.0,
        a3=10.0 * d - 4.0 * lam,
        a4=-15.0 * d + 7.0 * lam,
        a5=6.0 * d - 3.0 * lam,
    )


def build_quintic_corner_curve(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    params: QuinticTurnParameters,
) -> QuinticCornerCurve:
    """Build a C2 quintic local smoothing curve for one corner."""
    d = params.d
    lam = params.lam

    incoming = _sub(p_corner, p_prev)
    outgoing = _sub(p_next, p_corner)

    len_in = _norm(incoming)
    len_out = _norm(outgoing)

    if d <= 0.0 or lam <= 0.0:
        raise ValueError("d and lam must be positive.")
    if len_in <= 0.0 or len_out <= 0.0:
        raise ValueError("Incoming/outgoing segment lengths must be positive.")
    if d >= len_in or d >= len_out:
        raise ValueError("d must be smaller than both adjacent segment lengths.")

    e_in = _normalize(incoming)
    e_out = _normalize(outgoing)

    q_minus = _sub(p_corner, _scale(e_in, d))
    q_plus = _add(p_corner, _scale(e_out, d))

    return QuinticCornerCurve(
        corner=p_corner,
        e_in=e_in,
        e_out=e_out,
        x_poly=_build_local_x_polynomial(d, lam),
        y_poly=_build_local_y_polynomial(d, lam),
        q_minus=q_minus,
        q_plus=q_plus,
    )


# path descriptorの呼出し
if __name__ == "__main__":
    descriptor = build_example_path_descriptor()

    print(f"Total path length = {descriptor.header.total_length:.12f}")

    query_s_values = np.linspace(0.0, descriptor.header.total_length, 7)
    for s_query in query_s_values:
        geom = descriptor.evaluate_at_s(float(s_query))
        print(
            f"s={s_query:.12f}, "
            f"pos=({geom.position[0]:.12f}, {geom.position[1]:.12f}), "
            f"tan=({geom.tangent[0]:.12f}, {geom.tangent[1]:.12f}), "
            f"kappa={geom.curvature:.12f}"
        )