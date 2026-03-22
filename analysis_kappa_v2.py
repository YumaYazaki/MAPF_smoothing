from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import math

import matplotlib.pyplot as plt
import numpy as np


Vector2 = Tuple[float, float]


@dataclass(frozen=True)
class QuinticTurnParameters:
    """Parameters for quintic local smoothing at a corner."""

    d: float
    lam: float


@dataclass(frozen=True)
class CurveEvaluationResult:
    """Stores evaluation results for one quintic corner and full-path timing."""

    d: float
    lam: float
    rho: float
    kappa_max: float
    u_at_kappa_max: float
    u_at_kappa_max_sym: float
    dkappa_ds_max: float
    curve_length: float
    v_curve: float
    curve_travel_time: float
    full_path_time: float
    straight_before_length: float
    straight_after_length: float


@dataclass(frozen=True)
class FeasibilityConfig:
    """Optional feasibility constraints for design search."""

    d_min: Optional[float] = None
    d_max: Optional[float] = None
    rho_min: Optional[float] = None
    rho_max: Optional[float] = None
    max_curve_length: Optional[float] = None
    max_kappa: Optional[float] = None
    custom_feasibility_fn: Optional[Callable[[CurveEvaluationResult], bool]] = None


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
        """Evaluate polynomial."""
        return (
            self.a0
            + self.a1 * u
            + self.a2 * u**2
            + self.a3 * u**3
            + self.a4 * u**4
            + self.a5 * u**5
        )

    def first_derivative(self, u: float) -> float:
        """Evaluate first derivative."""
        return (
            self.a1
            + 2.0 * self.a2 * u
            + 3.0 * self.a3 * u**2
            + 4.0 * self.a4 * u**3
            + 5.0 * self.a5 * u**4
        )

    def second_derivative(self, u: float) -> float:
        """Evaluate second derivative."""
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
        """Evaluate position on the curve."""
        x_local = self.x_poly.value(u)
        y_local = self.y_poly.value(u)
        return _add(
            self.corner,
            _add(_scale(self.e_in, x_local), _scale(self.e_out, y_local)),
        )

    def first_derivative(self, u: float) -> Vector2:
        """Evaluate first derivative with respect to u."""
        dx = self.x_poly.first_derivative(u)
        dy = self.y_poly.first_derivative(u)
        return _add(_scale(self.e_in, dx), _scale(self.e_out, dy))

    def second_derivative(self, u: float) -> Vector2:
        """Evaluate second derivative with respect to u."""
        ddx = self.x_poly.second_derivative(u)
        ddy = self.y_poly.second_derivative(u)
        return _add(_scale(self.e_in, ddx), _scale(self.e_out, ddy))

    def speed_norm_u(self, u: float) -> float:
        """Evaluate norm of dp/du."""
        dx, dy = self.first_derivative(u)
        return math.hypot(dx, dy)

    def curvature(self, u: float, eps: float = 1e-12) -> float:
        """Evaluate curvature kappa(u)."""
        dx, dy = self.first_derivative(u)
        ddx, ddy = self.second_derivative(u)

        numerator = dx * ddy - dy * ddx
        denominator = (dx * dx + dy * dy) ** 1.5

        if denominator < eps:
            return 0.0
        return numerator / denominator


def build_quintic_corner_curve(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    params: QuinticTurnParameters,
) -> QuinticCornerCurve:
    """Build a C2 quintic local smoothing curve for one corner."""
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
    vmax_straight: float,
    amax_longitudinal: float,
    straight_before_length: float,
    straight_after_length: float,
    num_u_samples: int = 4001,
) -> CurveEvaluationResult:
    """Evaluate curvature, curve time, and full-path time.

    Full path:
        start -> straight_before -> curve -> straight_after -> goal

    Assumptions:
        - Straight sections can cruise at vmax_straight.
        - Curve is traversed at constant v_curve limited by lateral acceleration.
        - Acceleration/deceleration around curve uses amax_longitudinal.
        - Start and goal speeds are zero.
    """
    if a_lat_max <= 0.0:
        raise ValueError("a_lat_max must be positive.")
    if vmax_straight <= 0.0:
        raise ValueError("vmax_straight must be positive.")
    if amax_longitudinal <= 0.0:
        raise ValueError("amax_longitudinal must be positive.")
    if num_u_samples < 5:
        raise ValueError("num_u_samples must be at least 5.")

    u_values = np.linspace(0.0, 1.0, num_u_samples)

    kappa_values = np.array([curve.curvature(float(u)) for u in u_values])
    abs_kappa_values = np.abs(kappa_values)

    idx_max = int(np.argmax(abs_kappa_values))
    kappa_max = float(abs_kappa_values[idx_max])
    u_at_kappa_max = float(u_values[idx_max])
    u_at_kappa_max_sym = min(u_at_kappa_max, 1.0 - u_at_kappa_max)

    speed_norm_values = np.array([curve.speed_norm_u(float(u)) for u in u_values])
    curve_length = float(np.trapezoid(speed_norm_values, u_values))

    dkappa_du_values = np.gradient(kappa_values, u_values)
    with np.errstate(divide="ignore", invalid="ignore"):
        dkappa_ds_values = dkappa_du_values / speed_norm_values
    dkappa_ds_values = np.nan_to_num(
        dkappa_ds_values,
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    dkappa_ds_max = float(np.max(np.abs(dkappa_ds_values)))

    if kappa_max <= 1e-14:
        v_curve = vmax_straight
    else:
        v_curve = min(vmax_straight, math.sqrt(a_lat_max / kappa_max))

    curve_travel_time = curve_length / v_curve if v_curve > 0.0 else float("inf")

    full_path_time = compute_total_time_with_trapezoids(
        straight_before_length=straight_before_length,
        curve_length=curve_length,
        straight_after_length=straight_after_length,
        v_curve=v_curve,
        vmax_straight=vmax_straight,
        amax=amax_longitudinal,
    )

    return CurveEvaluationResult(
        d=d,
        lam=lam,
        rho=lam / d,
        kappa_max=kappa_max,
        u_at_kappa_max=u_at_kappa_max,
        u_at_kappa_max_sym=u_at_kappa_max_sym,
        dkappa_ds_max=dkappa_ds_max,
        curve_length=curve_length,
        v_curve=v_curve,
        curve_travel_time=curve_travel_time,
        full_path_time=full_path_time,
        straight_before_length=straight_before_length,
        straight_after_length=straight_after_length,
    )


def compute_total_time_with_trapezoids(
    straight_before_length: float,
    curve_length: float,
    straight_after_length: float,
    v_curve: float,
    vmax_straight: float,
    amax: float,
) -> float:
    """Compute total time with trapezoidal-like speed profile."""
    t_before = compute_segment_time_with_terminal_speeds(
        length=straight_before_length,
        v_start=0.0,
        v_end=v_curve,
        vmax=vmax_straight,
        amax=amax,
    )

    t_curve = curve_length / v_curve if v_curve > 0.0 else float("inf")

    t_after = compute_segment_time_with_terminal_speeds(
        length=straight_after_length,
        v_start=v_curve,
        v_end=0.0,
        vmax=vmax_straight,
        amax=amax,
    )

    return t_before + t_curve + t_after


def compute_segment_time_with_terminal_speeds(
    length: float,
    v_start: float,
    v_end: float,
    vmax: float,
    amax: float,
) -> float:
    """Minimum time over one straight segment with speed bounds."""
    if length < 0.0:
        raise ValueError("length must be non-negative.")
    if v_start < 0.0 or v_end < 0.0 or vmax <= 0.0 or amax <= 0.0:
        raise ValueError("Speeds and amax must be non-negative, vmax positive.")
    if v_start > vmax + 1e-12 or v_end > vmax + 1e-12:
        raise ValueError("v_start and v_end must not exceed vmax.")

    if length == 0.0:
        return 0.0

    dist_to_vmax = (
        max(0.0, vmax**2 - v_start**2) / (2.0 * amax)
        + max(0.0, vmax**2 - v_end**2) / (2.0 * amax)
    )

    if dist_to_vmax <= length:
        t_acc = max(0.0, vmax - v_start) / amax
        t_dec = max(0.0, vmax - v_end) / amax
        dist_cruise = length - dist_to_vmax
        t_cruise = dist_cruise / vmax
        return t_acc + t_cruise + t_dec

    vp_sq = amax * length + 0.5 * (v_start**2 + v_end**2)
    vp = math.sqrt(max(vp_sq, 0.0))
    if vp < max(v_start, v_end) - 1e-12:
        raise ValueError("Infeasible segment configuration.")

    t_acc = max(0.0, vp - v_start) / amax
    t_dec = max(0.0, vp - v_end) / amax
    return t_acc + t_dec


def sweep_parameters(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    d_values: List[float],
    rho_values: List[float],
    a_lat_max: float,
    vmax_straight: float,
    amax_longitudinal: float,
    num_u_samples: int = 4001,
) -> List[CurveEvaluationResult]:
    """Evaluate multiple (d, rho) combinations."""
    results: List[CurveEvaluationResult] = []

    total_in_length = _distance(p_prev, p_corner)
    total_out_length = _distance(p_corner, p_next)

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

            straight_before_length = total_in_length - d
            straight_after_length = total_out_length - d

            result = evaluate_curve_performance(
                curve=curve,
                d=d,
                lam=lam,
                a_lat_max=a_lat_max,
                vmax_straight=vmax_straight,
                amax_longitudinal=amax_longitudinal,
                straight_before_length=straight_before_length,
                straight_after_length=straight_after_length,
                num_u_samples=num_u_samples,
            )
            results.append(result)

    return results


def build_metric_maps(
    results: List[CurveEvaluationResult],
    d_values: List[float],
    rho_values: List[float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build grid maps for metrics."""
    def _closest_index(value: float, grid: List[float], tol: float = 1e-6) -> int:
        diffs = [abs(value - g) for g in grid]
        idx = int(np.argmin(diffs))
        if abs(value - grid[idx]) > tol:
            raise ValueError(
                f"Value {value:.12g} is not within {tol} of any grid point: {grid}"
            )
        return idx

    full_time_map = np.full((len(rho_values), len(d_values)), np.nan)
    kappa_map = np.full((len(rho_values), len(d_values)), np.nan)
    u_peak_map = np.full((len(rho_values), len(d_values)), np.nan)
    dkappa_ds_map = np.full((len(rho_values), len(d_values)), np.nan)

    for result in results:
        i = _closest_index(result.rho, rho_values)
        j = _closest_index(result.d, d_values)
        full_time_map[i, j] = result.full_path_time
        kappa_map[i, j] = result.kappa_max
        u_peak_map[i, j] = result.u_at_kappa_max_sym
        dkappa_ds_map[i, j] = result.dkappa_ds_max

    return full_time_map, kappa_map, u_peak_map, dkappa_ds_map

def find_pareto_candidates_3d(
    results: List[CurveEvaluationResult],
    feasibility: Optional[FeasibilityConfig] = None,
) -> List[CurveEvaluationResult]:
    """Find Pareto candidates for 3 objectives.

    Objectives to minimize:
        - full_path_time
        - kappa_max
        - dkappa_ds_max

    Args:
        results: Evaluation results.
        feasibility: Optional feasibility constraints.

    Returns:
        Pareto candidate list sorted by full_path_time.
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
                other.full_path_time <= candidate.full_path_time
                and other.kappa_max <= candidate.kappa_max
                and other.dkappa_ds_max <= candidate.dkappa_ds_max
            )
            strictly_better = (
                other.full_path_time < candidate.full_path_time
                or other.kappa_max < candidate.kappa_max
                or other.dkappa_ds_max < candidate.dkappa_ds_max
            )

            if no_worse and strictly_better:
                dominated = True
                break

        if not dominated:
            pareto.append(candidate)

    pareto.sort(key=lambda result: result.full_path_time)
    return pareto


def plot_pareto_front_2d_projections(
    results: List[CurveEvaluationResult],
    feasibility: Optional[FeasibilityConfig] = None,
) -> None:
    """Plot 2D projections of the 3-objective Pareto set.

    Projections:
        - full_path_time vs kappa_max
        - full_path_time vs dkappa_ds_max
        - kappa_max vs dkappa_ds_max

    Args:
        results: Evaluation results.
        feasibility: Optional feasibility constraints.
    """
    feasible_results = [
        result for result in results if is_result_feasible(result, feasibility)
    ]
    pareto = find_pareto_candidates_3d(results, feasibility)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # Projection 1
    axes[0].scatter(
        [result.kappa_max for result in feasible_results],
        [result.full_path_time for result in feasible_results],
        alpha=0.4,
        label="Feasible designs",
    )
    axes[0].scatter(
        [result.kappa_max for result in pareto],
        [result.full_path_time for result in pareto],
        marker="x",
        s=80,
        label="Pareto set",
    )
    axes[0].set_xlabel("kappa_max")
    axes[0].set_ylabel("full_path_time")
    axes[0].set_title("Projection: time vs kappa_max")
    axes[0].grid(True)
    axes[0].legend()

    # Projection 2
    axes[1].scatter(
        [result.dkappa_ds_max for result in feasible_results],
        [result.full_path_time for result in feasible_results],
        alpha=0.4,
        label="Feasible designs",
    )
    axes[1].scatter(
        [result.dkappa_ds_max for result in pareto],
        [result.full_path_time for result in pareto],
        marker="x",
        s=80,
        label="Pareto set",
    )
    axes[1].set_xlabel("max |dkappa/ds|")
    axes[1].set_ylabel("full_path_time")
    axes[1].set_title("Projection: time vs max |dkappa/ds|")
    axes[1].grid(True)
    axes[1].legend()

    # Projection 3
    axes[2].scatter(
        [result.kappa_max for result in feasible_results],
        [result.dkappa_ds_max for result in feasible_results],
        alpha=0.4,
        label="Feasible designs",
    )
    axes[2].scatter(
        [result.kappa_max for result in pareto],
        [result.dkappa_ds_max for result in pareto],
        marker="x",
        s=80,
        label="Pareto set",
    )
    axes[2].set_xlabel("kappa_max")
    axes[2].set_ylabel("max |dkappa/ds|")
    axes[2].set_title("Projection: kappa_max vs max |dkappa/ds|")
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()
    plt.show()


def plot_pareto_design_points(
    results: List[CurveEvaluationResult],
    feasibility: Optional[FeasibilityConfig] = None,
) -> None:
    """Plot feasible designs and Pareto candidates on the (d, rho) plane.

    Args:
        results: Evaluation results.
        feasibility: Optional feasibility constraints.
    """
    feasible_results = [
        result for result in results if is_result_feasible(result, feasibility)
    ]
    pareto = find_pareto_candidates_3d(results, feasibility)

    plt.figure(figsize=(7, 6))
    plt.scatter(
        [result.d for result in feasible_results],
        [result.rho for result in feasible_results],
        alpha=0.4,
        label="Feasible designs",
    )
    plt.scatter(
        [result.d for result in pareto],
        [result.rho for result in pareto],
        marker="x",
        s=100,
        label="Pareto set",
    )
    plt.xlabel("d")
    plt.ylabel("rho = lam / d")
    plt.title("Pareto-optimal design points on (d, rho)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def print_pareto_candidates_3d(
    results: List[CurveEvaluationResult],
    feasibility: Optional[FeasibilityConfig] = None,
) -> None:
    """Print the 3-objective Pareto candidate list.

    Args:
        results: Evaluation results.
        feasibility: Optional feasibility constraints.
    """
    pareto = find_pareto_candidates_3d(results, feasibility)

    print("\nPareto candidates for 3 objectives")
    print(
        "(minimize full_path_time, kappa_max, max|dkappa/ds|)"
    )
    for result in pareto:
        print(
            f"d={result.d:.3f}, "
            f"rho={result.rho:.3f}, "
            f"lam={result.lam:.3f}, "
            f"time={result.full_path_time:.6f}, "
            f"kappa_max={result.kappa_max:.6f}, "
            f"max|dkappa/ds|={result.dkappa_ds_max:.6f}, "
            f"u_peak_sym={result.u_at_kappa_max_sym:.6f}"
        )


def plot_all_maps(
    results: List[CurveEvaluationResult],
    d_values: List[float],
    rho_values: List[float],
) -> None:
    """Plot full_path_time, kappa_max, u_peak_sym, and dkappa_ds_max maps."""
    full_time_map, kappa_map, u_peak_map, dkappa_ds_map = build_metric_maps(
        results=results,
        d_values=d_values,
        rho_values=rho_values,
    )

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    im0 = axes[0, 0].imshow(
        full_time_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
    )
    axes[0, 0].set_xlabel("d")
    axes[0, 0].set_ylabel("rho = lam / d")
    axes[0, 0].set_title("full_path_time(d, rho)")
    fig.colorbar(im0, ax=axes[0, 0])

    im1 = axes[0, 1].imshow(
        kappa_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
    )
    axes[0, 1].set_xlabel("d")
    axes[0, 1].set_ylabel("rho = lam / d")
    axes[0, 1].set_title("kappa_max(d, rho)")
    fig.colorbar(im1, ax=axes[0, 1])

    im2 = axes[1, 0].imshow(
        u_peak_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
        vmin=0.0,
        vmax=0.5,
    )
    axes[1, 0].set_xlabel("d")
    axes[1, 0].set_ylabel("rho = lam / d")
    axes[1, 0].set_title("u_at_kappa_max_sym(d, rho)")
    fig.colorbar(im2, ax=axes[1, 0])

    im3 = axes[1, 1].imshow(
        dkappa_ds_map,
        aspect="auto",
        origin="lower",
        extent=[min(d_values), max(d_values), min(rho_values), max(rho_values)],
    )
    axes[1, 1].set_xlabel("d")
    axes[1, 1].set_ylabel("rho = lam / d")
    axes[1, 1].set_title("max |dkappa/ds|(d, rho)")
    fig.colorbar(im3, ax=axes[1, 1])

    plt.tight_layout()
    plt.show()

def sample_curve_arc_length_and_curvature(
    curve: QuinticCornerCurve,
    num_u_samples: int = 1000,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Sample one quintic corner curve and return arc length + curvature data.

    Args:
        curve: Quintic corner curve.
        num_u_samples: Number of u samples.

    Returns:
        Tuple of:
            - x values
            - y values
            - arc length s values
            - curvature kappa values
    """
    if num_u_samples < 2:
        raise ValueError("num_u_samples must be at least 2.")

    u_values = np.linspace(0.0, 1.0, num_u_samples)

    points = np.array([curve.point(float(u)) for u in u_values])
    x_values = points[:, 0]
    y_values = points[:, 1]

    kappa_values = np.array([curve.curvature(float(u)) for u in u_values])

    s_values = np.zeros(num_u_samples)
    for idx in range(1, num_u_samples):
        ds = math.hypot(
            x_values[idx] - x_values[idx - 1],
            y_values[idx] - y_values[idx - 1],
        )
        s_values[idx] = s_values[idx - 1] + ds

    return x_values, y_values, s_values, kappa_values


def plot_shape_comparison_subplots(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    d_values: List[float],
    rho_values: List[float],
    fixed_rho_for_d_sweep: float = 1.5,
    fixed_d_for_rho_sweep: float = 0.8,
    num_u_samples: int = 1000,
) -> None:
    """Plot 4 subplots for shape comparison.

    Subplots:
        - top-left:
            rho fixed, d sweep, 2D paths
        - top-right:
            rho fixed, d sweep, kappa(s)
        - bottom-left:
            d fixed, rho sweep, 2D paths
        - bottom-right:
            d fixed, rho sweep, kappa(s)

    Args:
        p_prev: Previous point.
        p_corner: Corner point.
        p_next: Next point.
        d_values: d sweep values.
        rho_values: rho sweep values.
        fixed_rho_for_d_sweep: Fixed rho for d sweep.
        fixed_d_for_rho_sweep: Fixed d for rho sweep.
        num_u_samples: Number of u samples on each curve.
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    ax_path_d = axes[0, 0]
    ax_kappa_d = axes[0, 1]
    ax_path_rho = axes[1, 0]
    ax_kappa_rho = axes[1, 1]

    # -----------------------------
    # Top row: rho fixed, d sweep
    # -----------------------------
    for d in d_values:
        lam = fixed_rho_for_d_sweep * d
        curve = build_quintic_corner_curve(
            p_prev=p_prev,
            p_corner=p_corner,
            p_next=p_next,
            params=QuinticTurnParameters(d=d, lam=lam),
        )
        x_values, y_values, s_values, kappa_values = sample_curve_arc_length_and_curvature(
            curve=curve,
            num_u_samples=num_u_samples,
        )

        label = f"d={d:.2f}"
        line, = ax_path_d.plot(x_values, y_values, label=label)
        ax_kappa_d.plot(
            s_values,
            kappa_values,
            label=label,
            color=line.get_color(),
        )

    ax_path_d.plot(
        [p_prev[0], p_corner[0], p_next[0]],
        [p_prev[1], p_corner[1], p_next[1]],
        "--",
        color="black",
        linewidth=1.5,
        label="Original MAPF polyline",
    )
    ax_path_d.scatter(
        [p_prev[0], p_corner[0], p_next[0]],
        [p_prev[1], p_corner[1], p_next[1]],
        color="black",
        s=25,
        label="MAPF points",
        zorder=5,
    )
    ax_path_d.axis("equal")
    ax_path_d.grid(True)
    ax_path_d.set_xlabel("x")
    ax_path_d.set_ylabel("y")
    ax_path_d.set_title(f"2D paths with rho={fixed_rho_for_d_sweep:.2f}, d sweep")
    ax_path_d.legend()

    ax_kappa_d.grid(True)
    ax_kappa_d.set_xlabel("Arc length s")
    ax_kappa_d.set_ylabel("kappa(s)")
    ax_kappa_d.set_title(
        f"Curvature on corner with rho={fixed_rho_for_d_sweep:.2f}, d sweep"
    )
    ax_kappa_d.legend()

    # -----------------------------
    # Bottom row: d fixed, rho sweep
    # -----------------------------
    for rho in rho_values:
        lam = rho * fixed_d_for_rho_sweep
        curve = build_quintic_corner_curve(
            p_prev=p_prev,
            p_corner=p_corner,
            p_next=p_next,
            params=QuinticTurnParameters(d=fixed_d_for_rho_sweep, lam=lam),
        )
        x_values, y_values, s_values, kappa_values = sample_curve_arc_length_and_curvature(
            curve=curve,
            num_u_samples=num_u_samples,
        )

        label = f"rho={rho:.2f}"
        line, = ax_path_rho.plot(x_values, y_values, label=label)
        ax_kappa_rho.plot(
            s_values,
            kappa_values,
            label=label,
            color=line.get_color(),
        )

    ax_path_rho.plot(
        [p_prev[0], p_corner[0], p_next[0]],
        [p_prev[1], p_corner[1], p_next[1]],
        "--",
        color="black",
        linewidth=1.5,
        label="Original MAPF polyline",
    )
    ax_path_rho.scatter(
        [p_prev[0], p_corner[0], p_next[0]],
        [p_prev[1], p_corner[1], p_next[1]],
        color="black",
        s=25,
        label="MAPF points",
        zorder=5,
    )
    ax_path_rho.axis("equal")
    ax_path_rho.grid(True)
    ax_path_rho.set_xlabel("x")
    ax_path_rho.set_ylabel("y")
    ax_path_rho.set_title(
        f"2D paths with d={fixed_d_for_rho_sweep:.2f}, rho sweep"
    )
    ax_path_rho.legend()

    ax_kappa_rho.grid(True)
    ax_kappa_rho.set_xlabel("Arc length s")
    ax_kappa_rho.set_ylabel("kappa(s)")
    ax_kappa_rho.set_title(
        f"Curvature on corner with d={fixed_d_for_rho_sweep:.2f}, rho sweep"
    )
    ax_kappa_rho.legend()

    plt.tight_layout()
    plt.show()

def plot_shape_comparison_subplots_with_normalized_s(
    p_prev: Vector2,
    p_corner: Vector2,
    p_next: Vector2,
    d_values: List[float],
    rho_values: List[float],
    fixed_rho_for_d_sweep: float = 1.5,
    fixed_d_for_rho_sweep: float = 0.8,
    num_u_samples: int = 1000,
) -> None:
    """Plot 8 subplots including normalized arc-length curvature plots.

    Layout:
        Row 1:
            - 2D paths, rho fixed and d sweep
            - kappa(s), rho fixed and d sweep
            - kappa(s/L), rho fixed and d sweep
        Row 2:
            - 2D paths, d fixed and rho sweep
            - kappa(s), d fixed and rho sweep
            - kappa(s/L), d fixed and rho sweep

    To keep it readable, this function uses a 2x3 layout.
    """
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))

    ax_path_d = axes[0, 0]
    ax_kappa_s_d = axes[0, 1]
    ax_kappa_sn_d = axes[0, 2]

    ax_path_rho = axes[1, 0]
    ax_kappa_s_rho = axes[1, 1]
    ax_kappa_sn_rho = axes[1, 2]

    # ---------------------------------
    # Top row: rho fixed, d sweep
    # ---------------------------------
    for d in d_values:
        lam = fixed_rho_for_d_sweep * d
        curve = build_quintic_corner_curve(
            p_prev=p_prev,
            p_corner=p_corner,
            p_next=p_next,
            params=QuinticTurnParameters(d=d, lam=lam),
        )
        x_values, y_values, s_values, kappa_values = sample_curve_arc_length_and_curvature(
            curve=curve,
            num_u_samples=num_u_samples,
        )
        s_norm_values = (
            s_values / s_values[-1] if s_values[-1] > 0.0 else s_values.copy()
        )

        label = f"d={d:.2f}"
        line, = ax_path_d.plot(x_values, y_values, label=label)
        color = line.get_color()

        ax_kappa_s_d.plot(
            s_values,
            kappa_values,
            label=label,
            color=color,
        )
        ax_kappa_sn_d.plot(
            s_norm_values,
            kappa_values,
            label=label,
            color=color,
        )

    ax_path_d.plot(
        [p_prev[0], p_corner[0], p_next[0]],
        [p_prev[1], p_corner[1], p_next[1]],
        "--",
        color="black",
        linewidth=1.5,
        label="Original MAPF polyline",
    )
    ax_path_d.scatter(
        [p_prev[0], p_corner[0], p_next[0]],
        [p_prev[1], p_corner[1], p_next[1]],
        color="black",
        s=25,
        label="MAPF points",
        zorder=5,
    )
    ax_path_d.axis("equal")
    ax_path_d.grid(True)
    ax_path_d.set_xlabel("x")
    ax_path_d.set_ylabel("y")
    ax_path_d.set_title(f"2D paths with rho={fixed_rho_for_d_sweep:.2f}, d sweep")
    ax_path_d.legend()

    ax_kappa_s_d.grid(True)
    ax_kappa_s_d.set_xlabel("Arc length s")
    ax_kappa_s_d.set_ylabel("kappa(s)")
    ax_kappa_s_d.set_title(
        f"Curvature vs s with rho={fixed_rho_for_d_sweep:.2f}, d sweep"
    )
    ax_kappa_s_d.legend()

    ax_kappa_sn_d.grid(True)
    ax_kappa_sn_d.set_xlabel("Normalized arc length s / L_curve")
    ax_kappa_sn_d.set_ylabel("kappa")
    ax_kappa_sn_d.set_title(
        f"Curvature vs normalized s with rho={fixed_rho_for_d_sweep:.2f}, d sweep"
    )
    ax_kappa_sn_d.legend()

    # ---------------------------------
    # Bottom row: d fixed, rho sweep
    # ---------------------------------
    for rho in rho_values:
        lam = rho * fixed_d_for_rho_sweep
        curve = build_quintic_corner_curve(
            p_prev=p_prev,
            p_corner=p_corner,
            p_next=p_next,
            params=QuinticTurnParameters(d=fixed_d_for_rho_sweep, lam=lam),
        )
        x_values, y_values, s_values, kappa_values = sample_curve_arc_length_and_curvature(
            curve=curve,
            num_u_samples=num_u_samples,
        )
        s_norm_values = (
            s_values / s_values[-1] if s_values[-1] > 0.0 else s_values.copy()
        )

        label = f"rho={rho:.2f}"
        line, = ax_path_rho.plot(x_values, y_values, label=label)
        color = line.get_color()

        ax_kappa_s_rho.plot(
            s_values,
            kappa_values,
            label=label,
            color=color,
        )
        ax_kappa_sn_rho.plot(
            s_norm_values,
            kappa_values,
            label=label,
            color=color,
        )

    ax_path_rho.plot(
        [p_prev[0], p_corner[0], p_next[0]],
        [p_prev[1], p_corner[1], p_next[1]],
        "--",
        color="black",
        linewidth=1.5,
        label="Original MAPF polyline",
    )
    ax_path_rho.scatter(
        [p_prev[0], p_corner[0], p_next[0]],
        [p_prev[1], p_corner[1], p_next[1]],
        color="black",
        s=25,
        label="MAPF points",
        zorder=5,
    )
    ax_path_rho.axis("equal")
    ax_path_rho.grid(True)
    ax_path_rho.set_xlabel("x")
    ax_path_rho.set_ylabel("y")
    ax_path_rho.set_title(
        f"2D paths with d={fixed_d_for_rho_sweep:.2f}, rho sweep"
    )
    ax_path_rho.legend()

    ax_kappa_s_rho.grid(True)
    ax_kappa_s_rho.set_xlabel("Arc length s")
    ax_kappa_s_rho.set_ylabel("kappa(s)")
    ax_kappa_s_rho.set_title(
        f"Curvature vs s with d={fixed_d_for_rho_sweep:.2f}, rho sweep"
    )
    ax_kappa_s_rho.legend()

    ax_kappa_sn_rho.grid(True)
    ax_kappa_sn_rho.set_xlabel("Normalized arc length s / L_curve")
    ax_kappa_sn_rho.set_ylabel("kappa")
    ax_kappa_sn_rho.set_title(
        f"Curvature vs normalized s with d={fixed_d_for_rho_sweep:.2f}, rho sweep"
    )
    ax_kappa_sn_rho.legend()

    plt.tight_layout()
    plt.show()


def is_result_feasible(
    result: CurveEvaluationResult,
    feasibility: Optional[FeasibilityConfig] = None,
) -> bool:
    """Check whether one evaluation result is feasible."""
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


def find_best_result_by_full_path_time(
    results: List[CurveEvaluationResult],
    feasibility: Optional[FeasibilityConfig] = None,
) -> CurveEvaluationResult:
    """Find feasible design with minimum full-path time."""
    feasible_results = [
        result for result in results if is_result_feasible(result, feasibility)
    ]
    if not feasible_results:
        raise ValueError("No feasible result found.")

    return min(feasible_results, key=lambda result: result.full_path_time)


def print_best_result(result: CurveEvaluationResult) -> None:
    """Print best-result summary."""
    print("Best result by minimum full-path time")
    print(f"d                  = {result.d:.6f}")
    print(f"lam                = {result.lam:.6f}")
    print(f"rho                = {result.rho:.6f}")
    print(f"kappa_max          = {result.kappa_max:.6f}")
    print(f"u_at_kappa_max     = {result.u_at_kappa_max:.6f}")
    print(f"u_at_kappa_max_sym = {result.u_at_kappa_max_sym:.6f}")
    print(f"max|dkappa/ds|     = {result.dkappa_ds_max:.6f}")
    print(f"curve_length       = {result.curve_length:.6f}")
    print(f"v_curve            = {result.v_curve:.6f}")
    print(f"curve_travel_time  = {result.curve_travel_time:.6f}")
    print(f"full_path_time     = {result.full_path_time:.6f}")


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
    vmax_straight = 1.0
    amax_longitudinal = 1.0

    # d_values = [0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60]
    # rho_values = [0.80, 0.90, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60]
    d_values = [0.20, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    # rho_values = [0.60, 0.90, 1.20, 1.50, 1.80, 2.10, 2.40, 2.7, 3.0]
    rho_values = [1.4, 1.45, 1.50, 1.55, 1.60, 1.65, 1.70, 1.75, 1.8]

    sweep_results = sweep_parameters(
        p_prev=p_prev,
        p_corner=p_corner,
        p_next=p_next,
        d_values=d_values,
        rho_values=rho_values,
        a_lat_max=a_lat_max,
        vmax_straight=vmax_straight,
        amax_longitudinal=amax_longitudinal,
        num_u_samples=4001,
    )

    plot_all_maps(
        results=sweep_results,
        d_values=d_values,
        rho_values=rho_values,
    )

    feasibility = FeasibilityConfig(
        d_min=min(d_values),
        d_max=max(d_values),
        rho_min=min(rho_values),
        rho_max=max(rho_values),
    )

    best_result = find_best_result_by_full_path_time(
        results=sweep_results,
        feasibility=feasibility,
    )
    print_best_result(best_result)

    plot_pareto_front_2d_projections(
        results=sweep_results,
        feasibility=feasibility,
    )

    plot_pareto_design_points(
        results=sweep_results,
        feasibility=feasibility,
    )

    print_pareto_candidates_3d(
        results=sweep_results,
        feasibility=feasibility,
    )

    # plot_shape_comparison_subplots(
    #     p_prev=p_prev,
    #     p_corner=p_corner,
    #     p_next=p_next,
    #     d_values=d_values,
    #     rho_values=rho_values,
    #     fixed_rho_for_d_sweep=1.5,
    #     fixed_d_for_rho_sweep=0.8,
    #     num_u_samples=1200,
    # )

    plot_shape_comparison_subplots_with_normalized_s(
        p_prev=p_prev,
        p_corner=p_corner,
        p_next=p_next,
        d_values=d_values,
        rho_values=rho_values,
        fixed_rho_for_d_sweep=1.5,
        fixed_d_for_rho_sweep=0.8,
        num_u_samples=1200,
    )