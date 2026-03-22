from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import math
import numpy as np


Vector2 = Tuple[float, float]


@dataclass(frozen=True)
class QuinticTurnParameters:
    """Parameters for quintic local smoothing at a corner.

    Attributes:
        d: Connection length from the corner along incoming/outgoing segments.
        lam: Tangent magnitude parameter of the quintic corner curve.
    """

    d: float
    lam: float


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
        """Evaluate the first derivative.

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
        """Evaluate the second derivative.

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

    The global curve is:
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
        """Evaluate dp/du.

        Args:
            u: Curve parameter in [0, 1].

        Returns:
            2D derivative vector.
        """
        dx = self.x_poly.first_derivative(u)
        dy = self.y_poly.first_derivative(u)
        return _add(_scale(self.e_in, dx), _scale(self.e_out, dy))

    def second_derivative(self, u: float) -> Vector2:
        """Evaluate d^2p/du^2.

        Args:
            u: Curve parameter in [0, 1].

        Returns:
            2D second derivative vector.
        """
        ddx = self.x_poly.second_derivative(u)
        ddy = self.y_poly.second_derivative(u)
        return _add(_scale(self.e_in, ddx), _scale(self.e_out, ddy))

    def speed_norm(self, u: float) -> float:
        """Evaluate ||dp/du||.

        Args:
            u: Curve parameter.

        Returns:
            Norm of dp/du.
        """
        dx, dy = self.first_derivative(u)
        return math.hypot(dx, dy)

    def curvature(self, u: float, eps: float = 1e-12) -> float:
        """Evaluate curvature kappa(u).

        Args:
            u: Curve parameter.
            eps: Small positive value to avoid zero-division.

        Returns:
            Curvature value.
        """
        dx, dy = self.first_derivative(u)
        ddx, ddy = self.second_derivative(u)

        numerator = dx * ddy - dy * ddx
        denominator = (dx * dx + dy * dy) ** 1.5
        if denominator < eps:
            return 0.0
        return numerator / denominator


@dataclass(frozen=True)
class CornerArcLengthDescriptor:
    """Arc-length descriptor for one quintic corner segment.

    Attributes:
        curve: Quintic corner curve.
        length_L: Total curve length.
        table_u: Monotone u table from 0 to 1.
        table_s: Monotone s(u) table from 0 to L.
    """

    curve: QuinticCornerCurve
    length_L: float
    table_u: np.ndarray
    table_s: np.ndarray


@dataclass(frozen=True)
class GeometryAtS:
    """Geometry evaluated at arc-length parameter s.

    Attributes:
        s_query: Queried arc length.
        u_value: Evaluated internal parameter u(s).
        position: Position p(s).
        tangent: Unit tangent dr/ds.
        curvature: Curvature kappa(s).
    """

    s_query: float
    u_value: float
    position: Vector2
    tangent: Vector2
    curvature: float


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
        raise ValueError("Incoming/outgoing segment lengths must be positive.")
    if d >= len_in or d >= len_out:
        raise ValueError("d must be smaller than both adjacent segment lengths.")

    e_in = _normalize(incoming)
    e_out = _normalize(outgoing)

    if abs(_dot(e_in, e_out)) > 1.0 - 1e-9:
        raise ValueError("Corner is degenerate or nearly straight.")

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


def build_corner_arclength_descriptor(
    curve: QuinticCornerCurve,
    num_table_points: int = 201,
    quadrature_order: int = 16,
) -> CornerArcLengthDescriptor:
    """Build high-accuracy s(u) table for one corner segment.

    This is intended for A-side offline construction.

    Args:
        curve: Quintic corner curve.
        num_table_points: Number of u grid points.
        quadrature_order: Gauss-Legendre quadrature order per integral call.

    Returns:
        Arc-length descriptor.

    Raises:
        ValueError: If the generated table is not strictly monotone.
    """
    if num_table_points < 2:
        raise ValueError("num_table_points must be at least 2.")
    if quadrature_order < 2:
        raise ValueError("quadrature_order must be at least 2.")

    table_u = np.linspace(0.0, 1.0, num_table_points)
    table_s = np.zeros(num_table_points, dtype=float)

    for idx in range(1, num_table_points):
        table_s[idx] = arclength_u(
            curve=curve,
            u=table_u[idx],
            quadrature_order=quadrature_order,
        )

    length_L = float(table_s[-1])

    if not np.all(np.diff(table_u) > 0.0):
        raise ValueError("table_u must be strictly increasing.")
    if not np.all(np.diff(table_s) > 0.0):
        raise ValueError("table_s must be strictly increasing.")

    return CornerArcLengthDescriptor(
        curve=curve,
        length_L=length_L,
        table_u=table_u,
        table_s=table_s,
    )


def arclength_u(
    curve: QuinticCornerCurve,
    u: float,
    quadrature_order: int = 16,
) -> float:
    """Evaluate s(u) = integral_0^u ||dp/du|| du by Gauss-Legendre quadrature.

    Args:
        curve: Quintic corner curve.
        u: Upper integration limit.
        quadrature_order: Gauss-Legendre quadrature order.

    Returns:
        Arc length from 0 to u.
    """
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        u = 1.0

    nodes, weights = np.polynomial.legendre.leggauss(quadrature_order)

    # Map [-1, 1] -> [0, u]
    midpoint = 0.5 * u
    half = 0.5 * u

    total = 0.0
    for node, weight in zip(nodes, weights):
        xi = midpoint + half * float(node)
        total += float(weight) * curve.speed_norm(xi)

    return half * total


def bracket_from_table(table_s: np.ndarray, s_query: float) -> Tuple[int, int]:
    """Find bracket indices such that table_s[i] <= s_query <= table_s[i+1].

    Args:
        table_s: Strictly increasing arc-length table.
        s_query: Query arc length.

    Returns:
        Pair of neighboring indices.
    """
    if s_query <= float(table_s[0]):
        return 0, 1
    if s_query >= float(table_s[-1]):
        return len(table_s) - 2, len(table_s) - 1

    right = int(np.searchsorted(table_s, s_query, side="right"))
    left = right - 1
    return left, right


def evaluate_u_from_s(
    descriptor: CornerArcLengthDescriptor,
    s_query: float,
    tol_s: float = 1e-10,
    tol_u: float = 1e-12,
    max_iter: int = 30,
    quadrature_order: int = 16,
) -> float:
    """Evaluate u(s) using hybrid Newton + bisection.

    This is intended for B-side online evaluation.

    Args:
        descriptor: Arc-length descriptor for one corner segment.
        s_query: Arc-length query in [0, L].
        tol_s: Residual tolerance in s.
        tol_u: Interval-width tolerance in u.
        max_iter: Maximum number of iterations.
        quadrature_order: Quadrature order for s(u) evaluation.

    Returns:
        Evaluated u in [0, 1].
    """
    length_L = descriptor.length_L

    # Boundary sticking
    if s_query <= 0.0:
        return 0.0
    if s_query >= length_L:
        return 1.0

    i_left, i_right = bracket_from_table(descriptor.table_s, s_query)

    u_left = float(descriptor.table_u[i_left])
    u_right = float(descriptor.table_u[i_right])
    s_left = float(descriptor.table_s[i_left])
    s_right = float(descriptor.table_s[i_right])

    if s_right > s_left:
        alpha = (s_query - s_left) / (s_right - s_left)
    else:
        alpha = 0.0

    u = u_left + alpha * (u_right - u_left)

    for _ in range(max_iter):
        s_u = arclength_u(
            curve=descriptor.curve,
            u=u,
            quadrature_order=quadrature_order,
        )
        f_u = s_u - s_query

        if abs(f_u) <= tol_s:
            return _clamp(u, 0.0, 1.0)

        speed = descriptor.curve.speed_norm(u)

        # Default candidate: bisection
        u_candidate = 0.5 * (u_left + u_right)

        # Safe Newton step
        if speed > 0.0:
            u_newton = u - f_u / speed
            if u_left < u_newton < u_right:
                u_candidate = u_newton

        s_candidate = arclength_u(
            curve=descriptor.curve,
            u=u_candidate,
            quadrature_order=quadrature_order,
        )
        f_candidate = s_candidate - s_query

        if f_candidate > 0.0:
            u_right = u_candidate
        else:
            u_left = u_candidate

        u = u_candidate

        if abs(u_right - u_left) <= tol_u:
            return _clamp(0.5 * (u_left + u_right), 0.0, 1.0)

    return _clamp(0.5 * (u_left + u_right), 0.0, 1.0)


def evaluate_geometry_at_s(
    descriptor: CornerArcLengthDescriptor,
    s_query: float,
    tol_s: float = 1e-10,
    tol_u: float = 1e-12,
    max_iter: int = 30,
    quadrature_order: int = 16,
) -> GeometryAtS:
    """Evaluate geometry on the corner segment at arc-length parameter s.

    Args:
        descriptor: Arc-length descriptor.
        s_query: Arc-length query.
        tol_s: Residual tolerance in s.
        tol_u: Interval-width tolerance in u.
        max_iter: Maximum number of iterations.
        quadrature_order: Quadrature order for s(u) evaluation.

    Returns:
        Geometry evaluated at s.
    """
    u_value = evaluate_u_from_s(
        descriptor=descriptor,
        s_query=s_query,
        tol_s=tol_s,
        tol_u=tol_u,
        max_iter=max_iter,
        quadrature_order=quadrature_order,
    )

    position = descriptor.curve.point(u_value)
    du1 = descriptor.curve.first_derivative(u_value)
    speed = math.hypot(du1[0], du1[1])

    if speed > 0.0:
        tangent = (du1[0] / speed, du1[1] / speed)
    else:
        tangent = (0.0, 0.0)

    curvature = descriptor.curve.curvature(u_value)

    return GeometryAtS(
        s_query=s_query,
        u_value=u_value,
        position=position,
        tangent=tangent,
        curvature=curvature,
    )


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
    """Dot product of two vectors."""
    return a[0] * b[0] + a[1] * b[1]


def _norm(v: Vector2) -> float:
    """Euclidean norm."""
    return math.hypot(v[0], v[1])


def _normalize(v: Vector2) -> Vector2:
    """Normalize a vector."""
    n = _norm(v)
    if n == 0.0:
        raise ValueError("Cannot normalize a zero vector.")
    return (v[0] / n, v[1] / n)


def _clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a scalar into [lower, upper]."""
    return max(lower, min(upper, value))


if __name__ == "__main__":
    # Example:
    p_prev = (-2.0, 0.0)
    p_corner = (0.0, 0.0)
    p_next = (0.0, 2.0)

    params = QuinticTurnParameters(d=0.8, lam=1.2)
    curve = build_quintic_corner_curve(
        p_prev=p_prev,
        p_corner=p_corner,
        p_next=p_next,
        params=params,
    )

    descriptor = build_corner_arclength_descriptor(
        curve=curve,
        num_table_points=301,
        quadrature_order=20,
    )

    print(f"Total length L = {descriptor.length_L:.12f}")

    query_s_values = [
        0.0,
        0.1 * descriptor.length_L,
        0.5 * descriptor.length_L,
        0.9 * descriptor.length_L,
        descriptor.length_L,
    ]

    for s_query in query_s_values:
        geom = evaluate_geometry_at_s(
            descriptor=descriptor,
            s_query=s_query,
            tol_s=1e-12,
            tol_u=1e-13,
            max_iter=40,
            quadrature_order=20,
        )
        print(
            f"s={geom.s_query:.12f}, "
            f"u={geom.u_value:.12f}, "
            f"pos=({geom.position[0]:.12f}, {geom.position[1]:.12f}), "
            f"tan=({geom.tangent[0]:.12f}, {geom.tangent[1]:.12f}), "
            f"kappa={geom.curvature:.12f}"
        )