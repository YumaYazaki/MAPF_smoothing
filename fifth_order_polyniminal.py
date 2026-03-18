from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import math
import matplotlib.pyplot as plt


Vector2 = Tuple[float, float]


@dataclass(frozen=True)
class QuinticTurnParameters:
    """Parameters for quintic local smoothing at a corner.

    Attributes:
        d: Connection length from the corner along the incoming/outgoing
            directions.
        lam: Tangent magnitude parameter in the normalized quintic curve.
    """

    d: float
    lam: float


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
        """Evaluate the polynomial.

        Args:
            u: Curve parameter.

        Returns:
            Polynomial value at u.
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
        """Evaluate the first derivative.

        Args:
            u: Curve parameter.

        Returns:
            First derivative at u.
        """
        return (
            self.a1
            + 2.0 * self.a2 * u
            + 3.0 * self.a3 * u**2
            + 4.0 * self.a4 * u**3
            + 5.0 * self.a5 * u**4
        )

    def second_derivative(self, u: float) -> float:
        """Evaluate the second derivative.

        Args:
            u: Curve parameter.

        Returns:
            Second derivative at u.
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
            2D position.
        """
        x_local = self.x_poly.value(u)
        y_local = self.y_poly.value(u)
        return _add(
            self.corner,
            _add(_scale(self.e_in, x_local), _scale(self.e_out, y_local)),
        )

    def first_derivative(self, u: float) -> Vector2:
        """Evaluate the first derivative with respect to u.

        Args:
            u: Curve parameter in [0, 1].

        Returns:
            2D derivative vector.
        """
        dx = self.x_poly.first_derivative(u)
        dy = self.y_poly.first_derivative(u)
        return _add(_scale(self.e_in, dx), _scale(self.e_out, dy))

    def second_derivative(self, u: float) -> Vector2:
        """Evaluate the second derivative with respect to u.

        Args:
            u: Curve parameter in [0, 1].

        Returns:
            2D second derivative vector.
        """
        ddx = self.x_poly.second_derivative(u)
        ddy = self.y_poly.second_derivative(u)
        return _add(_scale(self.e_in, ddx), _scale(self.e_out, ddy))

    def curvature(self, u: float, eps: float = 1e-12) -> float:
        """Evaluate curvature kappa(u).

        Args:
            u: Curve parameter in [0, 1].
            eps: Small value to avoid division by zero.

        Returns:
            Curvature at u.
        """
        dx, dy = self.first_derivative(u)
        ddx, ddy = self.second_derivative(u)

        numerator = dx * ddy - dy * ddx
        denominator = (dx * dx + dy * dy) ** 1.5

        if denominator < eps:
            return 0.0
        return numerator / denominator

    def sample(self, num_points: int) -> List[Vector2]:
        """Sample points on the curve.

        Args:
            num_points: Number of sample points including both endpoints.

        Returns:
            Sampled 2D points.

        Raises:
            ValueError: If num_points < 2.
        """
        if num_points < 2:
            raise ValueError("num_points must be at least 2.")
        return [self.point(i / (num_points - 1)) for i in range(num_points)]

    def sample_arc_length_and_curvature(
        self,
        num_points: int,
    ) -> Tuple[List[float], List[float], List[float]]:
        """Sample arc length s and curvature kappa along the curve.

        Arc length is approximated by cumulative chord length of sampled points.

        Args:
            num_points: Number of sample points including both endpoints.

        Returns:
            Tuple of:
                - u samples
                - arc length samples s
                - curvature samples kappa(s)

        Raises:
            ValueError: If num_points < 2.
        """
        if num_points < 2:
            raise ValueError("num_points must be at least 2.")

        u_values = [i / (num_points - 1) for i in range(num_points)]
        points = [self.point(u) for u in u_values]
        kappa_values = [self.curvature(u) for u in u_values]

        s_values: List[float] = [0.0]
        for idx in range(1, len(points)):
            ds = _distance(points[idx - 1], points[idx])
            s_values.append(s_values[-1] + ds)

        return u_values, s_values, kappa_values


@dataclass(frozen=True)
class ComparisonResult:
    """Stores one parameter set and its sampled comparison data."""

    label: str
    d: float
    lam: float
    s_values: List[float]
    kappa_values: List[float]
    sampled_points: List[Vector2]
    peak_abs_kappa: float


def build_quintic_corner_curve(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    params: QuinticTurnParameters,
) -> QuinticCornerCurve:
    """Build a C2 quintic local smoothing curve for one MAPF corner.

    Args:
        p_prev: A point before the corner.
        p_corner: The corner point to be smoothed.
        p_next: A point after the corner.
        params: Quintic smoothing parameters.

    Returns:
        QuinticCornerCurve object.

    Raises:
        ValueError: If parameters are invalid.
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

    if d >= len_in:
        raise ValueError("d must be smaller than incoming segment length.")
    if d >= len_out:
        raise ValueError("d must be smaller than outgoing segment length.")

    e_in = _normalize(incoming)
    e_out = _normalize(outgoing)

    dot = _dot(e_in, e_out)
    if abs(dot) > 1.0 - 1e-9:
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


def run_parameter_comparison(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    parameter_sets: Sequence[QuinticTurnParameters],
    num_curve_points: int = 400,
) -> List[ComparisonResult]:
    """Run comparison for multiple (d, lam) parameter sets.

    Args:
        p_prev: Point before the corner.
        p_corner: Corner point.
        p_next: Point after the corner.
        parameter_sets: Sequence of parameter sets to compare.
        num_curve_points: Number of samples for each curve.

    Returns:
        Comparison results for all parameter sets.
    """
    results: List[ComparisonResult] = []

    for params in parameter_sets:
        curve = build_quintic_corner_curve(
            p_prev=p_prev,
            p_corner=p_corner,
            p_next=p_next,
            params=params,
        )

        _, s_values, kappa_values = curve.sample_arc_length_and_curvature(
            num_curve_points
        )
        sampled_points = curve.sample(num_curve_points)
        peak_abs_kappa = max(abs(kappa) for kappa in kappa_values)

        label = f"d={params.d:.3f}, lam={params.lam:.3f}"
        results.append(
            ComparisonResult(
                label=label,
                d=params.d,
                lam=params.lam,
                s_values=s_values,
                kappa_values=kappa_values,
                sampled_points=sampled_points,
                peak_abs_kappa=peak_abs_kappa,
            )
        )

    return results


def build_parameter_sets_from_rho(
    d_values: Sequence[float],
    rho_values: Sequence[float],
) -> List[QuinticTurnParameters]:
    """Create parameter sets using lam = rho * d.

    Args:
        d_values: Connection lengths.
        rho_values: Dimensionless tangent scales.

    Returns:
        Flattened list of parameter sets.
    """
    parameter_sets: List[QuinticTurnParameters] = []
    for d in d_values:
        for rho in rho_values:
            parameter_sets.append(
                QuinticTurnParameters(d=d, lam=rho * d)
            )
    return parameter_sets

def plot_parameter_comparison(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    results: Sequence[ComparisonResult],
) -> None:
    """Plot geometry, curvature-vs-s, and peak curvature comparison.

    Args:
        p_prev: Point before the corner.
        p_corner: Corner point.
        p_next: Point after the corner.
        results: Comparison results.
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    ax_geom = axes[0][0]
    ax_kappa = axes[0][1]
    ax_peak = axes[1][0]
    axes[1][1].axis("off")

    original_x = [p_prev[0], p_corner[0], p_next[0]]
    original_y = [p_prev[1], p_corner[1], p_next[1]]

    peak_labels: List[str] = []
    peak_values: List[float] = []

    # 先に各パラメータの曲線を両方のグラフへ同じ順序で描く
    # これにより、Geometry comparison と Curvature comparison の色を揃える
    for result in results:
        smoothed_x = [p_prev[0]] + [p[0] for p in result.sampled_points] + [
            p_next[0]
        ]
        smoothed_y = [p_prev[1]] + [p[1] for p in result.sampled_points] + [
            p_next[1]
        ]

        line_geom, = ax_geom.plot(smoothed_x, smoothed_y, label=result.label)
        ax_kappa.plot(
            result.s_values,
            result.kappa_values,
            label=result.label,
            color=line_geom.get_color(),
        )

        peak_labels.append(result.label)
        peak_values.append(result.peak_abs_kappa)

    # 元の MAPF 折れ線と点は最後に描く
    ax_geom.plot(
        original_x,
        original_y,
        "--",
        color="black",
        linewidth=1.5,
        label="Original MAPF polyline",
    )
    ax_geom.scatter(
        original_x,
        original_y,
        marker="o",
        color="black",
        label="MAPF points",
        zorder=5,
    )

    ax_geom.axis("equal")
    ax_geom.grid(True)
    ax_geom.set_xlabel("x")
    ax_geom.set_ylabel("y")
    ax_geom.set_title("Geometry comparison")
    ax_geom.legend()

    ax_kappa.grid(True)
    ax_kappa.set_xlabel("Arc length s")
    ax_kappa.set_ylabel("kappa(s)")
    ax_kappa.set_title("Curvature comparison")
    ax_kappa.legend()

    x_positions = list(range(len(peak_values)))
    ax_peak.bar(x_positions, peak_values)
    ax_peak.set_xticks(x_positions)
    ax_peak.set_xticklabels(peak_labels, rotation=45, ha="right")
    ax_peak.set_ylabel("max |kappa(s)|")
    ax_peak.set_title("Peak curvature comparison")
    ax_peak.grid(True, axis="y")

    plt.tight_layout()
    plt.show()

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
    """Euclidean norm of a 2D vector."""
    return math.hypot(v[0], v[1])


def _distance(a: Vector2, b: Vector2) -> float:
    """Euclidean distance between two points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _normalize(v: Vector2) -> Vector2:
    """Normalize a 2D vector.

    Args:
        v: Input vector.

    Returns:
        Unit vector.

    Raises:
        ValueError: If the vector norm is zero.
    """
    n = _norm(v)
    if n == 0.0:
        raise ValueError("Cannot normalize a zero vector.")
    return (v[0] / n, v[1] / n)


if __name__ == "__main__":
    p_prev_example = (-2.0, 0.0)
    p_corner_example = (0.0, 0.0)
    p_next_example = (0.0, 2.0)

    # 例1: lam を直接指定する場合
    parameter_sets_direct = [
        QuinticTurnParameters(d=0.30, lam=0.30),
        QuinticTurnParameters(d=0.40, lam=0.40),
        QuinticTurnParameters(d=0.50, lam=0.50),
        QuinticTurnParameters(d=0.50, lam=0.35),
        QuinticTurnParameters(d=0.50, lam=0.65),
        QuinticTurnParameters(d=0.50, lam=0.80),
    ]

    results_direct = run_parameter_comparison(
        p_prev=p_prev_example,
        p_corner=p_corner_example,
        p_next=p_next_example,
        parameter_sets=parameter_sets_direct,
        num_curve_points=400,
    )
    plot_parameter_comparison(
        p_prev=p_prev_example,
        p_corner=p_corner_example,
        p_next=p_next_example,
        results=results_direct,
    )

    # 例2: lam = rho * d でまとめて作る場合
    d_values = [0.30, 0.40, 0.50]
    rho_values = [0.8, 1.0, 1.2]

    parameter_sets_rho = build_parameter_sets_from_rho(
        d_values=d_values,
        rho_values=rho_values,
    )

    results_rho = run_parameter_comparison(
        p_prev=p_prev_example,
        p_corner=p_corner_example,
        p_next=p_next_example,
        parameter_sets=parameter_sets_rho,
        num_curve_points=400,
    )
    plot_parameter_comparison(
        p_prev=p_prev_example,
        p_corner=p_corner_example,
        p_next=p_next_example,
        results=results_rho,
    )