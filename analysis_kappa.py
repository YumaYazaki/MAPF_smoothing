from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import math

import matplotlib.pyplot as plt
import numpy as np


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
class CurveEvaluationResult:
    """Stores evaluation results for one quintic corner.

    Attributes:
        d: Connection length.
        lam: Tangent magnitude parameter.
        rho: Dimensionless ratio lam / d.
        kappa_max: Maximum absolute curvature on u in [0, 1].
        u_at_kappa_max: Parameter u where maximum absolute curvature is attained.
        curve_length: Arc length of the corner curve.
        v_curve: Maximum allowable constant speed under lateral acceleration
            constraint.
        travel_time: Travel time through the corner at constant speed.
    """

    d: float
    lam: float
    rho: float
    kappa_max: float
    u_at_kappa_max: float
    curve_length: float
    v_curve: float
    travel_time: float


@dataclass(frozen=True)
class QuinticPolynomial1D:
    """Represents a 1D quintic polynomial.

    The polynomial is:
        p(u) = a0 + a1*u + a2*u^2 + a3*u^3 + a4*u^4 + a5*u^5
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
        """Evaluate first derivative.

        Args:
            u: Curve parameter.

        Returns:
            First derivative.
        """
        return (
            self.a1
            + 2.0 * self.a2 * u
            + 3.0 * self.a3 * u**2
            + 4.0 * self.a4 * u**3
            + 5.0 * self.a5 * u**4
        )

    def second_derivative(self, u: float) -> float:
        """Evaluate second derivative.

        Args:
            u: Curve parameter.

        Returns:
            Second derivative.
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
        """Evaluate position on the curve.

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
        """Evaluate first derivative with respect to u.

        Args:
            u: Curve parameter in [0, 1].

        Returns:
            2D derivative vector.
        """
        dx = self.x_poly.first_derivative(u)
        dy = self.y_poly.first_derivative(u)
        return _add(_scale(self.e_in, dx), _scale(self.e_out, dy))

    def second_derivative(self, u: float) -> Vector2:
        """Evaluate second derivative with respect to u.

        Args:
            u: Curve parameter in [0, 1].

        Returns:
            2D second derivative vector.
        """
        ddx = self.x_poly.second_derivative(u)
        ddy = self.y_poly.second_derivative(u)
        return _add(_scale(self.e_in, ddx), _scale(self.e_out, ddy))

    def speed_norm_u(self, u: float) -> float:
        """Evaluate norm of dp/du.

        Args:
            u: Curve parameter.

        Returns:
            Norm of first derivative.
        """
        dx, dy = self.first_derivative(u)
        return math.hypot(dx, dy)

    def curvature(self, u: float, eps: float = 1e-12) -> float:
        """Evaluate curvature kappa(u).

        Args:
            u: Curve parameter.
            eps: Small value to avoid division by zero.

        Returns:
            Curvature.
        """
        dx, dy = self.first_derivative(u)
        ddx, ddy = self.second_derivative(u)

        numerator = dx * ddy - dy * ddx
        denominator = (dx * dx + dy * dy) ** 1.5

        if denominator < eps:
            return 0.0
        return numerator / denominator
    

@dataclass(frozen=True)
class FeasibilityConfig:
    """Optional feasibility constraints for design search.

    Attributes:
        d_min: Minimum allowable d.
        d_max: Maximum allowable d.
        rho_min: Minimum allowable rho.
        rho_max: Maximum allowable rho.
        max_curve_length: Optional upper bound on curve length.
        max_kappa: Optional upper bound on maximum curvature.
        custom_feasibility_fn: Optional user-supplied feasibility function.
    """

    d_min: Optional[float] = None
    d_max: Optional[float] = None
    rho_min: Optional[float] = None
    rho_max: Optional[float] = None
    max_curve_length: Optional[float] = None
    max_kappa: Optional[float] = None
    custom_feasibility_fn: Optional[Callable[[CurveEvaluationResult], bool]] = None


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


def evaluate_curve_performance(
    curve: QuinticCornerCurve,
    d: float,
    lam: float,
    a_lat_max: float,
    num_u_samples: int = 4001,
) -> CurveEvaluationResult:
    """Evaluate max curvature, curve length, constant-speed limit, and travel time.

    Args:
        curve: Quintic corner curve.
        d: Connection length.
        lam: Tangent magnitude parameter.
        a_lat_max: Maximum allowable lateral acceleration.
        num_u_samples: Number of u samples for numerical search/integration.

    Returns:
        Curve evaluation result.

    Raises:
        ValueError: If a_lat_max <= 0 or num_u_samples < 3.
    """
    if a_lat_max <= 0.0:
        raise ValueError("a_lat_max must be positive.")
    if num_u_samples < 3:
        raise ValueError("num_u_samples must be at least 3.")

    u_values = np.linspace(0.0, 1.0, num_u_samples)

    kappa_values = np.array([curve.curvature(float(u)) for u in u_values])
    abs_kappa_values = np.abs(kappa_values)

    idx_max = int(np.argmax(abs_kappa_values))
    kappa_max = float(abs_kappa_values[idx_max])
    u_at_kappa_max = float(u_values[idx_max])

    speed_norm_values = np.array([curve.speed_norm_u(float(u)) for u in u_values])
    curve_length = float(np.trapezoid(speed_norm_values, u_values))

    if kappa_max <= 1e-14:
        v_curve = float("inf")
        travel_time = 0.0 if curve_length <= 1e-14 else curve_length / v_curve
    else:
        v_curve = math.sqrt(a_lat_max / kappa_max)
        travel_time = curve_length / v_curve

    return CurveEvaluationResult(
        d=d,
        lam=lam,
        rho=lam / d,
        kappa_max=kappa_max,
        u_at_kappa_max=u_at_kappa_max,
        curve_length=curve_length,
        v_curve=v_curve,
        travel_time=travel_time,
    )


def sweep_parameters(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    d_values: List[float],
    rho_values: List[float],
    a_lat_max: float,
    num_u_samples: int = 4001,
) -> List[CurveEvaluationResult]:
    """Evaluate multiple (d, rho) combinations.

    Here lam = rho * d.

    Args:
        p_prev: Previous point.
        p_corner: Corner point.
        p_next: Next point.
        d_values: Candidate d values.
        rho_values: Candidate rho values.
        a_lat_max: Maximum allowable lateral acceleration.
        num_u_samples: Number of u samples.

    Returns:
        List of evaluation results.
    """
    results: List[CurveEvaluationResult] = []

    for d in d_values:
        for rho in rho_values:
            lam = rho * d
            params = QuinticTurnParameters(d=d, lam=lam)
            curve = build_quintic_corner_curve(
                p_prev=p_prev,
                p_corner=p_corner,
                p_next=p_next,
                params=params,
            )
            result = evaluate_curve_performance(
                curve=curve,
                d=d,
                lam=lam,
                a_lat_max=a_lat_max,
                num_u_samples=num_u_samples,
            )
            results.append(result)

    return results


def plot_kappa_and_time_maps(
    results: List[CurveEvaluationResult],
    d_values: List[float],
    rho_values: List[float],
) -> None:
    """Plot heatmaps of kappa_max and travel time.

    Args:
        results: Evaluation results.
        d_values: d grid values.
        rho_values: rho grid values.
    """
    def _closest_index(value: float, grid: List[float], tol: float = 1e-6) -> int:
        diffs = [abs(value - g) for g in grid]
        idx = int(np.argmin(diffs))
        if abs(value - grid[idx]) > tol:
            raise ValueError(
                f"Value {value:.12g} is not within {tol} of any grid point: {grid}"
            )
        return idx

    kappa_map = np.full((len(rho_values), len(d_values)), np.nan)
    time_map = np.full((len(rho_values), len(d_values)), np.nan)

    for result in results:
        i = _closest_index(result.rho, rho_values)
        j = _closest_index(result.d, d_values)
        kappa_map[i, j] = result.kappa_max
        time_map[i, j] = result.travel_time

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    im0 = axes[0].imshow(
        kappa_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
    )
    axes[0].set_xlabel("d")
    axes[0].set_ylabel("rho = lam / d")
    axes[0].set_title("kappa_max(d, lam)")
    fig.colorbar(im0, ax=axes[0])

    im1 = axes[1].imshow(
        time_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
    )
    axes[1].set_xlabel("d")
    axes[1].set_ylabel("rho = lam / d")
    axes[1].set_title("Travel time T(d, lam)")
    fig.colorbar(im1, ax=axes[1])

    plt.tight_layout()
    plt.show()


def _build_local_x_polynomial(d: float, lam: float) -> QuinticPolynomial1D:
    """Build the quintic polynomial for local x(u)."""
    return QuinticPolynomial1D(
        a0=-d,
        a1=lam,
        a2=0.0,
        a3=10.0 * d - 6.0 * lam,
        a4=-15.0 * d + 8.0 * lam,
        a5=6.0 * d - 3.0 * lam,
    )


def _build_local_y_polynomial(d: float, lam: float) -> QuinticPolynomial1D:
    """Build the quintic polynomial for local y(u)."""
    return QuinticPolynomial1D(
        a0=0.0,
        a1=0.0,
        a2=0.0,
        a3=10.0 * d - 4.0 * lam,
        a4=-15.0 * d + 7.0 * lam,
        a5=6.0 * d - 3.0 * lam,
    )

def is_result_feasible(
    result: CurveEvaluationResult,
    feasibility: Optional[FeasibilityConfig] = None,
) -> bool:
    """Check whether one evaluation result is feasible.

    Args:
        result: Evaluation result.
        feasibility: Optional feasibility configuration.

    Returns:
        True if feasible.
    """
    if feasibility is None:
        return True

    if feasibility.d_min is not None and result.d < feasibility.d_min:
        return False
    if feasibility.d_max is not None and result.d > feasibility.d_max:
        return False
    if feasibility.rho_min is not None and result.rho < feasibility.rho_min:
        return False
    if feasibility.rho_max is not None and result.rho > feasibility.rho_max:
        return False
    if (
        feasibility.max_curve_length is not None
        and result.curve_length > feasibility.max_curve_length
    ):
        return False
    if feasibility.max_kappa is not None and result.kappa_max > feasibility.max_kappa:
        return False
    if feasibility.custom_feasibility_fn is not None:
        return feasibility.custom_feasibility_fn(result)

    return True


def build_metric_maps(
    results: List[CurveEvaluationResult],
    d_values: List[float],
    rho_values: List[float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build grid maps for kappa_max, travel_time, and u_at_kappa_max.

    Args:
        results: Sweep results.
        d_values: d grid values.
        rho_values: rho grid values.

    Returns:
        Tuple of:
            - kappa_map
            - time_map
            - u_peak_map
    """
    def _closest_index(value: float, grid: List[float], tol: float = 1e-6) -> int:
        diffs = [abs(value - g) for g in grid]
        idx = int(np.argmin(diffs))
        if abs(value - grid[idx]) > tol:
            raise ValueError(
                f"Value {value:.12g} is not within {tol} of any grid point: {grid}"
            )
        return idx

    kappa_map = np.full((len(rho_values), len(d_values)), np.nan)
    time_map = np.full((len(rho_values), len(d_values)), np.nan)
    u_peak_map = np.full((len(rho_values), len(d_values)), np.nan)

    for result in results:
        i = _closest_index(result.rho, rho_values)
        j = _closest_index(result.d, d_values)
        kappa_map[i, j] = result.kappa_max
        time_map[i, j] = result.travel_time
        u_peak_map[i, j] = result.u_at_kappa_max

    return kappa_map, time_map, u_peak_map


def plot_u_at_kappa_max_map(
    results: List[CurveEvaluationResult],
    d_values: List[float],
    rho_values: List[float],
) -> None:
    """Plot heatmap of u_at_kappa_max(d, rho).

    Args:
        results: Sweep results.
        d_values: d grid values.
        rho_values: rho grid values.
    """
    _, _, u_peak_map = build_metric_maps(results, d_values, rho_values)

    plt.figure(figsize=(6, 5))
    im = plt.imshow(
        u_peak_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
        vmin=0.0,
        vmax=1.0,
    )
    plt.xlabel("d")
    plt.ylabel("rho = lam / d")
    plt.title("u_at_kappa_max(d, rho)")
    plt.colorbar(im, label="u at max |kappa|")
    plt.tight_layout()
    plt.show()


def plot_all_maps(
    results: List[CurveEvaluationResult],
    d_values: List[float],
    rho_values: List[float],
) -> None:
    """Plot kappa_max, travel_time, and u_at_kappa_max maps together.

    Args:
        results: Sweep results.
        d_values: d grid values.
        rho_values: rho grid values.
    """
    kappa_map, time_map, u_peak_map = build_metric_maps(
        results=results,
        d_values=d_values,
        rho_values=rho_values,
    )

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    im0 = axes[0].imshow(
        kappa_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
    )
    axes[0].set_xlabel("d")
    axes[0].set_ylabel("rho = lam / d")
    axes[0].set_title("kappa_max(d, rho)")
    fig.colorbar(im0, ax=axes[0])

    im1 = axes[1].imshow(
        time_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
    )
    axes[1].set_xlabel("d")
    axes[1].set_ylabel("rho = lam / d")
    axes[1].set_title("travel_time(d, rho)")
    fig.colorbar(im1, ax=axes[1])

    im2 = axes[2].imshow(
        u_peak_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
        vmin=0.0,
        vmax=1.0,
    )
    axes[2].set_xlabel("d")
    axes[2].set_ylabel("rho = lam / d")
    axes[2].set_title("u_at_kappa_max(d, rho)")
    fig.colorbar(im2, ax=axes[2])

    plt.tight_layout()
    plt.show()


def find_best_result_by_travel_time(
    results: List[CurveEvaluationResult],
    feasibility: Optional[FeasibilityConfig] = None,
) -> CurveEvaluationResult:
    """Find the feasible design with minimum travel time.

    Args:
        results: Sweep results.
        feasibility: Optional feasibility constraints.

    Returns:
        Best feasible result.

    Raises:
        ValueError: If no feasible result exists.
    """
    feasible_results = [
        result for result in results if is_result_feasible(result, feasibility)
    ]
    if not feasible_results:
        raise ValueError("No feasible result found.")

    return min(feasible_results, key=lambda result: result.travel_time)


def find_pareto_candidates(
    results: List[CurveEvaluationResult],
    feasibility: Optional[FeasibilityConfig] = None,
) -> List[CurveEvaluationResult]:
    """Find Pareto candidates for (travel_time, kappa_max).

    A result is Pareto-optimal if no other feasible result is better in both
    travel_time and kappa_max, with at least one strict improvement.

    Args:
        results: Sweep results.
        feasibility: Optional feasibility constraints.

    Returns:
        Pareto candidate list sorted by travel_time.
    """
    feasible_results = [
        result for result in results if is_result_feasible(result, feasibility)
    ]

    pareto: List[CurveEvaluationResult] = []
    for candidate in feasible_results:
        dominated = False
        for other in feasible_results:
            if other is candidate:
                continue

            no_worse = (
                other.travel_time <= candidate.travel_time
                and other.kappa_max <= candidate.kappa_max
            )
            strictly_better = (
                other.travel_time < candidate.travel_time
                or other.kappa_max < candidate.kappa_max
            )
            if no_worse and strictly_better:
                dominated = True
                break

        if not dominated:
            pareto.append(candidate)

    pareto.sort(key=lambda result: result.travel_time)
    return pareto


def print_best_result(result: CurveEvaluationResult) -> None:
    """Print one best-result summary.

    Args:
        result: Best result.
    """
    print("Best result by minimum travel time")
    print(f"d              = {result.d:.6f}")
    print(f"lam            = {result.lam:.6f}")
    print(f"rho            = {result.rho:.6f}")
    print(f"kappa_max      = {result.kappa_max:.6f}")
    print(f"u_at_kappa_max = {result.u_at_kappa_max:.6f}")
    print(f"curve_length   = {result.curve_length:.6f}")
    print(f"v_curve        = {result.v_curve:.6f}")
    print(f"travel_time    = {result.travel_time:.6f}")


def plot_pareto_front(
    results: List[CurveEvaluationResult],
    feasibility: Optional[FeasibilityConfig] = None,
) -> None:
    """Plot travel_time vs kappa_max and highlight Pareto candidates.

    Args:
        results: Sweep results.
        feasibility: Optional feasibility constraints.
    """
    feasible_results = [
        result for result in results if is_result_feasible(result, feasibility)
    ]
    pareto = find_pareto_candidates(results, feasibility)

    plt.figure(figsize=(7, 5))
    plt.scatter(
        [result.kappa_max for result in feasible_results],
        [result.travel_time for result in feasible_results],
        alpha=0.5,
        label="Feasible designs",
    )
    plt.scatter(
        [result.kappa_max for result in pareto],
        [result.travel_time for result in pareto],
        marker="x",
        s=80,
        label="Pareto front",
    )
    plt.xlabel("kappa_max")
    plt.ylabel("travel_time")
    plt.title("Pareto front: travel_time vs kappa_max")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()



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


def _normalize(v: Vector2) -> Vector2:
    """Normalize a 2D vector."""
    norm = _norm(v)
    if norm == 0.0:
        raise ValueError("Cannot normalize a zero vector.")
    return (v[0] / norm, v[1] / norm)


if __name__ == "__main__":
    # 90-degree corner example
    p_prev = (-2.0, 0.0)
    p_corner = (0.0, 0.0)
    p_next = (0.0, 2.0)

    a_lat_max = 1.0

    # Single evaluation
    d = 0.5
    lam = 0.5
    curve = build_quintic_corner_curve(
        p_prev=p_prev,
        p_corner=p_corner,
        p_next=p_next,
        params=QuinticTurnParameters(d=d, lam=lam),
    )
    result = evaluate_curve_performance(
        curve=curve,
        d=d,
        lam=lam,
        a_lat_max=a_lat_max,
        num_u_samples=8001,
    )

    print("Single evaluation")
    print(f"d              = {result.d:.6f}")
    print(f"lam            = {result.lam:.6f}")
    print(f"rho            = {result.rho:.6f}")
    print(f"kappa_max      = {result.kappa_max:.6f}")
    print(f"u_at_kappa_max = {result.u_at_kappa_max:.6f}")
    print(f"curve_length   = {result.curve_length:.6f}")
    print(f"v_curve        = {result.v_curve:.6f}")
    print(f"travel_time    = {result.travel_time:.6f}")

    # Parameter sweep

    # d_values = [0.2, 0.3, 0.4, 0.5, 0.6]
    # rho_values = [0.8, 1.0, 1.2, 1.4, 1.6]

    # sweep_results = sweep_parameters(
    #     p_prev=p_prev,
    #     p_corner=p_corner,
    #     p_next=p_next,
    #     d_values=d_values,
    #     rho_values=rho_values,
    #     a_lat_max=a_lat_max,
    #     num_u_samples=4001,
    # )

    # plot_kappa_and_time_maps(
    #     results=sweep_results,
    #     d_values=d_values,
    #     rho_values=rho_values,
    # )


    d_values = [0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60]
    rho_values = [0.80, 0.90, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60]

    sweep_results = sweep_parameters(
        p_prev=p_prev,
        p_corner=p_corner,
        p_next=p_next,
        d_values=d_values,
        rho_values=rho_values,
        a_lat_max=a_lat_max,
        num_u_samples=4001,
    )

    # 3つのマップをまとめて表示
    plot_all_maps(
        results=sweep_results,
        d_values=d_values,
        rho_values=rho_values,
    )

    # u_at_kappa_max のみを個別表示
    plot_u_at_kappa_max_map(
        results=sweep_results,
        d_values=d_values,
        rho_values=rho_values,
    )

    # 実務上は何らかの feasibility を入れるのが自然
    feasibility = FeasibilityConfig(
        d_min=0.20,
        d_max=0.60,
        rho_min=0.80,
        rho_max=1.60,
        # 例: 曲率最大値に上限を入れたい場合
        # max_kappa=8.0,
    )

    best_result = find_best_result_by_travel_time(
        results=sweep_results,
        feasibility=feasibility,
    )
    print_best_result(best_result)

    pareto = find_pareto_candidates(
        results=sweep_results,
        feasibility=feasibility,
    )
    print("\nPareto candidates")
    for result in pareto:
        print(
            f"d={result.d:.3f}, "
            f"rho={result.rho:.3f}, "
            f"kappa_max={result.kappa_max:.6f}, "
            f"travel_time={result.travel_time:.6f}, "
            f"u_peak={result.u_at_kappa_max:.6f}"
        )

    plot_pareto_front(
        results=sweep_results,
        feasibility=feasibility,
    )
