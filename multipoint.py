from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import math
import matplotlib.pyplot as plt


Vector2 = Tuple[float, float]


@dataclass(frozen=True)
class QuinticTurnParameters:
    """Parameters for one quintic local corner smoothing.

    Attributes:
        d: Connection length from the corner along incoming/outgoing segments.
        lam: Tangent magnitude parameter for the quintic curve.
    """

    d: float
    lam: float


@dataclass(frozen=True)
class GlobalSmoothingConfig:
    """Configuration for smoothing an entire MAPF polyline.

    Attributes:
        d_default: Default desired connection length.
        rho: Dimensionless tangent scale where lam = rho * d.
        corner_angle_threshold_deg: Threshold below which a point is treated
            as nearly straight and not smoothed.
        segment_margin_ratio: Safety margin to avoid overlap between adjacent
            smoothing intervals on the same segment.
        min_d: Minimum valid smoothing length. Corners with smaller effective
            d are skipped.
        curve_samples_per_corner: Number of samples per quintic curve.
    """

    d_default: float
    rho: float = 1.0
    corner_angle_threshold_deg: float = 5.0
    segment_margin_ratio: float = 0.98
    min_d: float = 1e-4
    curve_samples_per_corner: int = 80


@dataclass(frozen=True)
class QuinticPolynomial1D:
    """Represents a 1D quintic polynomial.

    The polynomial is:
        p(u) = a0 + a1*u + a2*u^2 + a3*u^3 + a4*u^4 + a5*u^5
    for u in [0, 1].
    """

    a0: float
    a1: float
    a2: float
    a3: float
    a4: float
    a5: float

    def value(self, u: float) -> float:
        """Evaluate the polynomial at u.

        Args:
            u: Curve parameter.

        Returns:
            Polynomial value.
        """
        return (
            self.a0
            + self.a1 * u
            + self.a2 * u**2
            + self.a3 * u**3
            + self.a4 * u**4
            + self.a5 * u**5
        )

    def first_derivative(self, u: float) -> float:
        """Evaluate the first derivative at u.

        Args:
            u: Curve parameter.

        Returns:
            First derivative value.
        """
        return (
            self.a1
            + 2.0 * self.a2 * u
            + 3.0 * self.a3 * u**2
            + 4.0 * self.a4 * u**3
            + 5.0 * self.a5 * u**4
        )

    def second_derivative(self, u: float) -> float:
        """Evaluate the second derivative at u.

        Args:
            u: Curve parameter.

        Returns:
            Second derivative value.
        """
        return (
            2.0 * self.a2
            + 6.0 * self.a3 * u
            + 12.0 * self.a4 * u**2
            + 20.0 * self.a5 * u**3
        )


@dataclass(frozen=True)
class QuinticCornerCurve:
    """Local quintic corner smoothing curve in 2D.

    The global curve is defined as:
        p(u) = corner + x(u) * e_in + y(u) * e_out
    """

    corner: Vector2
    e_in: Vector2
    e_out: Vector2
    x_poly: QuinticPolynomial1D
    y_poly: QuinticPolynomial1D
    q_minus: Vector2
    q_plus: Vector2

    def point(self, u: float) -> Vector2:
        """Evaluate the curve position.

        Args:
            u: Curve parameter in [0, 1].

        Returns:
            2D point on the curve.
        """
        x_local = self.x_poly.value(u)
        y_local = self.y_poly.value(u)
        return _add(
            self.corner,
            _add(_scale(self.e_in, x_local), _scale(self.e_out, y_local)),
        )

    def sample(self, num_points: int) -> List[Vector2]:
        """Sample points on the curve.

        Args:
            num_points: Number of samples including endpoints.

        Returns:
            Sampled points.

        Raises:
            ValueError: If num_points < 2.
        """
        if num_points < 2:
            raise ValueError("num_points must be at least 2.")
        return [self.point(i / (num_points - 1)) for i in range(num_points)]


@dataclass(frozen=True)
class CornerSmoothingResult:
    """Stores one corner smoothing result."""

    corner_index: int
    curve: QuinticCornerCurve
    d: float
    lam: float


@dataclass(frozen=True)
class SmoothedPolylineResult:
    """Stores the result of smoothing a whole MAPF polyline.

    Attributes:
        original_points: Original MAPF points.
        smoothed_points: Sampled smoothed polyline.
        corner_results: Per-corner smoothing results.
    """

    original_points: List[Vector2]
    smoothed_points: List[Vector2]
    corner_results: List[CornerSmoothingResult]


def smooth_mapf_polyline(
    points: Sequence[Vector2],
    config: GlobalSmoothingConfig,
) -> SmoothedPolylineResult:
    """Apply quintic local smoothing sequentially to a whole MAPF polyline.

    This function:
        1. Detects valid corners in the polyline.
        2. Assigns candidate smoothing lengths d for each corner.
        3. Adjusts adjacent d values so they do not overlap on shared segments.
        4. Replaces each corner with a quintic C2 local curve.
        5. Returns the final sampled smoothed polyline.

    Args:
        points: MAPF polyline points.
        config: Global smoothing configuration.

    Returns:
        Smoothed polyline result.

    Raises:
        ValueError: If the input polyline is invalid.
    """
    if len(points) < 3:
        raise ValueError("At least 3 points are required.")

    point_list = list(points)
    num_points = len(point_list)

    segment_lengths = _compute_segment_lengths(point_list)
    corner_flags = _detect_corners(point_list, config.corner_angle_threshold_deg)

    d_candidates = _compute_initial_corner_ds(
        points=point_list,
        segment_lengths=segment_lengths,
        corner_flags=corner_flags,
        d_default=config.d_default,
    )

    d_adjusted = _adjust_corner_ds_to_avoid_overlap(
        d_candidates=d_candidates,
        segment_lengths=segment_lengths,
        corner_flags=corner_flags,
        margin_ratio=config.segment_margin_ratio,
        min_d=config.min_d,
    )

    corner_results: List[CornerSmoothingResult] = []
    for idx in range(1, num_points - 1):
        d_eff = d_adjusted[idx]
        if d_eff is None or d_eff < config.min_d:
            continue

        lam = config.rho * d_eff
        params = QuinticTurnParameters(d=d_eff, lam=lam)
        curve = build_quintic_corner_curve(
            p_prev=point_list[idx - 1],
            p_corner=point_list[idx],
            p_next=point_list[idx + 1],
            params=params,
        )
        corner_results.append(
            CornerSmoothingResult(
                corner_index=idx,
                curve=curve,
                d=d_eff,
                lam=lam,
            )
        )

    smoothed_points = _assemble_smoothed_polyline(
        original_points=point_list,
        corner_results=corner_results,
        curve_samples_per_corner=config.curve_samples_per_corner,
    )

    return SmoothedPolylineResult(
        original_points=point_list,
        smoothed_points=smoothed_points,
        corner_results=corner_results,
    )


def plot_smoothed_polyline(result: SmoothedPolylineResult) -> None:
    """Plot original MAPF polyline and smoothed polyline.

    Args:
        result: Smoothed polyline result.
    """
    original_x = [p[0] for p in result.original_points]
    original_y = [p[1] for p in result.original_points]
    smooth_x = [p[0] for p in result.smoothed_points]
    smooth_y = [p[1] for p in result.smoothed_points]

    plt.figure(figsize=(8, 8))
    plt.plot(smooth_x, smooth_y, label="Smoothed polyline")
    plt.plot(original_x, original_y, "--", color="black", label="Original MAPF")
    plt.scatter(original_x, original_y, color="black", s=25, label="MAPF points")

    for corner_result in result.corner_results:
        q_minus = corner_result.curve.q_minus
        q_plus = corner_result.curve.q_plus
        plt.scatter(
            [q_minus[0], q_plus[0]],
            [q_minus[1], q_plus[1]],
            marker="x",
            s=50,
        )

    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Whole MAPF polyline smoothing by local quintic curves")
    plt.legend()
    plt.show()


def build_quintic_corner_curve(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    params: QuinticTurnParameters,
) -> QuinticCornerCurve:
    """Build a C2 quintic local smoothing curve for one corner.

    Args:
        p_prev: Previous point.
        p_corner: Corner point.
        p_next: Next point.
        params: Quintic smoothing parameters.

    Returns:
        Quintic corner curve.

    Raises:
        ValueError: If the corner or parameters are invalid.
    """
    d = params.d
    lam = params.lam

    if d <= 0.0:
        raise ValueError("d must be positive.")
    if lam <= 0.0:
        raise ValueError("lam must be positive.")

    incoming = _sub(p_corner, p_prev)
    outgoing = _sub(p_next, p_corner)

    len_in = _norm(incoming)
    len_out = _norm(outgoing)

    if len_in <= 0.0 or len_out <= 0.0:
        raise ValueError("Incoming/outgoing segment length must be positive.")
    if d >= len_in or d >= len_out:
        raise ValueError("d must be smaller than both adjacent segment lengths.")

    e_in = _normalize(incoming)
    e_out = _normalize(outgoing)

    if abs(_dot(e_in, e_out)) > 1.0 - 1e-9:
        raise ValueError("The corner is degenerate or nearly straight.")

    q_minus = _sub(p_corner, _scale(e_in, d))
    q_plus = _add(p_corner, _scale(e_out, d))

    x_poly = _build_local_x_polynomial(d=d, lam=lam)
    y_poly = _build_local_y_polynomial(d=d, lam=lam)

    return QuinticCornerCurve(
        corner=p_corner,
        e_in=e_in,
        e_out=e_out,
        x_poly=x_poly,
        y_poly=y_poly,
        q_minus=q_minus,
        q_plus=q_plus,
    )


def _assemble_smoothed_polyline(
    original_points: Sequence[Vector2],
    corner_results: Sequence[CornerSmoothingResult],
    curve_samples_per_corner: int,
) -> List[Vector2]:
    """Assemble the whole sampled smoothed polyline.

    Args:
        original_points: Original MAPF points.
        corner_results: Per-corner smoothing results.
        curve_samples_per_corner: Samples per corner curve.

    Returns:
        Sampled smoothed polyline.
    """
    corner_map = {result.corner_index: result for result in corner_results}
    num_points = len(original_points)

    path: List[Vector2] = [original_points[0]]

    for seg_idx in range(num_points - 1):
        p_start = original_points[seg_idx]
        p_end = original_points[seg_idx + 1]

        left_corner = corner_map.get(seg_idx)
        right_corner = corner_map.get(seg_idx + 1)

        segment_start = left_corner.curve.q_plus if left_corner else p_start
        segment_end = right_corner.curve.q_minus if right_corner else p_end

        if not _points_close(path[-1], segment_start):
            path.append(segment_start)

        if not _points_close(segment_start, segment_end):
            path.append(segment_end)

        if right_corner:
            curve_points = right_corner.curve.sample(curve_samples_per_corner)
            for point in curve_points[1:]:
                if not _points_close(path[-1], point):
                    path.append(point)

    return path


def _compute_segment_lengths(points: Sequence[Vector2]) -> List[float]:
    """Compute lengths of all consecutive segments.

    Args:
        points: Polyline points.

    Returns:
        Segment lengths.
    """
    return [_distance(points[i], points[i + 1]) for i in range(len(points) - 1)]


def _detect_corners(
    points: Sequence[Vector2],
    corner_angle_threshold_deg: float,
) -> List[bool]:
    """Detect whether each internal point should be treated as a corner.

    Args:
        points: Polyline points.
        corner_angle_threshold_deg: Threshold below which the point is treated
            as nearly straight.

    Returns:
        Boolean flags for all point indices.
    """
    flags = [False] * len(points)
    threshold_rad = math.radians(corner_angle_threshold_deg)

    for idx in range(1, len(points) - 1):
        v_in = _sub(points[idx], points[idx - 1])
        v_out = _sub(points[idx + 1], points[idx])

        if _norm(v_in) == 0.0 or _norm(v_out) == 0.0:
            continue

        e_in = _normalize(v_in)
        e_out = _normalize(v_out)
        cos_theta = max(-1.0, min(1.0, _dot(e_in, e_out)))
        theta = math.acos(cos_theta)

        if theta >= threshold_rad and theta <= math.pi - threshold_rad:
            flags[idx] = True

    return flags


def _compute_initial_corner_ds(
    points: Sequence[Vector2],
    segment_lengths: Sequence[float],
    corner_flags: Sequence[bool],
    d_default: float,
) -> List[Optional[float]]:
    """Compute initial candidate d for each corner.

    Args:
        points: Polyline points.
        segment_lengths: Consecutive segment lengths.
        corner_flags: Corner flags.
        d_default: Desired base d.

    Returns:
        Candidate d values per point index, or None.
    """
    d_values: List[Optional[float]] = [None] * len(points)

    for idx in range(1, len(points) - 1):
        if not corner_flags[idx]:
            continue

        len_in = segment_lengths[idx - 1]
        len_out = segment_lengths[idx]
        d_values[idx] = min(d_default, 0.5 * len_in, 0.5 * len_out)

    return d_values


def _adjust_corner_ds_to_avoid_overlap(
    d_candidates: Sequence[Optional[float]],
    segment_lengths: Sequence[float],
    corner_flags: Sequence[bool],
    margin_ratio: float,
    min_d: float,
) -> List[Optional[float]]:
    """Adjust d values so adjacent smoothing intervals do not overlap.

    On segment i (between points i and i+1), the right-side smoothing of point i
    and the left-side smoothing of point i+1 must satisfy:
        d_i + d_{i+1} <= margin_ratio * segment_length_i

    Args:
        d_candidates: Initial d values indexed by point.
        segment_lengths: Segment lengths indexed by segment.
        corner_flags: Corner flags indexed by point.
        margin_ratio: Margin ratio for non-overlap.
        min_d: Minimum acceptable d.

    Returns:
        Adjusted d values.
    """
    adjusted = list(d_candidates)

    for seg_idx, seg_len in enumerate(segment_lengths):
        left_point_idx = seg_idx
        right_point_idx = seg_idx + 1

        d_left = adjusted[left_point_idx]
        d_right = adjusted[right_point_idx]

        if d_left is None or not corner_flags[left_point_idx]:
            d_left = 0.0
        if d_right is None or not corner_flags[right_point_idx]:
            d_right = 0.0

        total = d_left + d_right
        limit = margin_ratio * seg_len

        if total <= limit or total == 0.0:
            continue

        scale = limit / total

        if corner_flags[left_point_idx] and adjusted[left_point_idx] is not None:
            adjusted[left_point_idx] = adjusted[left_point_idx] * scale

        if corner_flags[right_point_idx] and adjusted[right_point_idx] is not None:
            adjusted[right_point_idx] = adjusted[right_point_idx] * scale

    for idx, value in enumerate(adjusted):
        if value is not None and value < min_d:
            adjusted[idx] = None

    return adjusted


def _build_local_x_polynomial(d: float, lam: float) -> QuinticPolynomial1D:
    """Build the quintic polynomial for local x(u).

    Args:
        d: Connection length.
        lam: Tangent magnitude parameter.

    Returns:
        Quintic polynomial for x(u).
    """
    return QuinticPolynomial1D(
        a0=-d,
        a1=lam,
        a2=0.0,
        a3=10.0 * d - 6.0 * lam,
        a4=-15.0 * d + 8.0 * lam,
        a5=6.0 * d - 3.0 * lam,
    )


def _build_local_y_polynomial(d: float, lam: float) -> QuinticPolynomial1D:
    """Build the quintic polynomial for local y(u).

    Args:
        d: Connection length.
        lam: Tangent magnitude parameter.

    Returns:
        Quintic polynomial for y(u).
    """
    return QuinticPolynomial1D(
        a0=0.0,
        a1=0.0,
        a2=0.0,
        a3=10.0 * d - 4.0 * lam,
        a4=-15.0 * d + 7.0 * lam,
        a5=6.0 * d - 3.0 * lam,
    )


def _add(a: Vector2, b: Vector2) -> Vector2:
    """Add two 2D vectors."""
    return (a[0] + b[0], a[1] + b[1])


def _sub(a: Vector2, b: Vector2) -> Vector2:
    """Subtract two 2D vectors."""
    return (a[0] - b[0], a[1] - b[1])


def _scale(v: Vector2, s: float) -> Vector2:
    """Scale a 2D vector."""
    return (v[0] * s, v[1] * s)


def _dot(a: Vector2, b: Vector2) -> float:
    """Dot product of two 2D vectors."""
    return a[0] * b[0] + a[1] * b[1]


def _norm(v: Vector2) -> float:
    """Euclidean norm."""
    return math.hypot(v[0], v[1])


def _distance(a: Vector2, b: Vector2) -> float:
    """Euclidean distance."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _normalize(v: Vector2) -> Vector2:
    """Normalize a 2D vector.

    Args:
        v: Input vector.

    Returns:
        Normalized vector.

    Raises:
        ValueError: If the vector has zero norm.
    """
    norm = _norm(v)
    if norm == 0.0:
        raise ValueError("Cannot normalize a zero vector.")
    return (v[0] / norm, v[1] / norm)


def _points_close(a: Vector2, b: Vector2, atol: float = 1e-10) -> bool:
    """Check whether two points are almost identical.

    Args:
        a: First point.
        b: Second point.
        atol: Absolute tolerance.

    Returns:
        True if close.
    """
    return _distance(a, b) <= atol


if __name__ == "__main__":
    mapf_points_example = [
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, 2.0),
        (5.0, 2.0),
        (5.0, 4.0),
        (7.0, 4.0),
    ]

    config_example = GlobalSmoothingConfig(
        d_default=0.45,
        rho=1.0,
        corner_angle_threshold_deg=5.0,
        segment_margin_ratio=0.95,
        min_d=1e-3,
        curve_samples_per_corner=100,
    )

    result_example = smooth_mapf_polyline(
        points=mapf_points_example,
        config=config_example,
    )

    print("Number of smoothed corners:", len(result_example.corner_results))
    for corner_result in result_example.corner_results:
        print(
            f"corner_index={corner_result.corner_index}, "
            f"d={corner_result.d:.4f}, "
            f"lam={corner_result.lam:.4f}, "
            f"q_minus={corner_result.curve.q_minus}, "
            f"q_plus={corner_result.curve.q_plus}"
        )

    plot_smoothed_polyline(result_example)