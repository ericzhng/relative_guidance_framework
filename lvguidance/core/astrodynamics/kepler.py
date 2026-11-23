"""Universal-variable Keplerian orbit propagation and Lambert targeting."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.astrodynamics.constants import MU_EARTH

Vec = NDArray[np.float64]


def stumpff_c(z: float) -> float:
    """Stumpff function C(z) = (1 - cos sqrt(z)) / z."""
    if abs(z) < 1e-4:
        return 0.5 - z / 24.0 + z * z / 720.0
    if z > 0.0:
        return (1.0 - math.cos(math.sqrt(z))) / z
    return (math.cosh(math.sqrt(-z)) - 1.0) / (-z)


def stumpff_s(z: float) -> float:
    """Stumpff function S(z) = (sqrt(z) - sin sqrt(z)) / z^(3/2)."""
    if abs(z) < 1e-4:
        return 1.0 / 6.0 - z / 120.0 + z * z / 5040.0
    if z > 0.0:
        root = math.sqrt(z)
        return (root - math.sin(root)) / root**3
    root = math.sqrt(-z)
    return (math.sinh(root) - root) / root**3


def propagate_kepler(r0: Vec, v0: Vec, dt: float, mu: float = MU_EARTH) -> tuple[Vec, Vec]:
    """Propagate an elliptic/hyperbolic two-body orbit with universal variables.

    Args:
        r0: Initial position [m].
        v0: Initial velocity [m/s].
        dt: Propagation time interval (can be negative) [s].
        mu: Gravitational parameter [m^3/s^2].

    Returns:
        Tuple (r, v) after dt seconds.

    Raises:
        RuntimeError: If Newton-Raphson iteration on the universal Kepler equation fails.
    """
    r0n = float(np.linalg.norm(r0))
    v0n = float(np.linalg.norm(v0))
    vr0 = float(np.dot(r0, v0)) / r0n
    alpha = 2.0 / r0n - v0n**2 / mu
    sqmu = math.sqrt(mu)
    chi = sqmu * alpha * dt

    for _ in range(100):
        z = alpha * chi**2
        c, s = stumpff_c(z), stumpff_s(z)
        f_val = (
            r0n * vr0 / sqmu * chi**2 * c
            + (1.0 - alpha * r0n) * chi**3 * s
            + r0n * chi
            - sqmu * dt
        )
        df_val = (
            r0n * vr0 / sqmu * chi * (1.0 - alpha * chi**2 * s)
            + (1.0 - alpha * r0n) * chi**2 * c
            + r0n
        )
        step = f_val / df_val
        chi -= step
        if abs(step) < 1e-10:
            break
    else:
        raise RuntimeError("Universal Kepler equation did not converge")

    z = alpha * chi**2
    c, s = stumpff_c(z), stumpff_s(z)
    f = 1.0 - chi**2 / r0n * c
    g = dt - chi**3 / sqmu * s
    r = f * r0 + g * v0
    rn = float(np.linalg.norm(r))
    fdot = sqmu / (rn * r0n) * (alpha * chi**3 * s - chi)
    gdot = 1.0 - chi**2 / rn * c
    return r, fdot * r0 + gdot * v0


def solve_lambert(
    r1: Vec,
    r2: Vec,
    tof: float,
    mu: float = MU_EARTH,
    prograde: bool = True,
    normal: Vec | None = None,
) -> tuple[Vec, Vec]:
    """Solve zero-revolution Lambert problem using universal variables.

    Args:
        r1: Departure position vector [m].
        r2: Arrival position vector [m].
        tof: Time of flight [s].
        mu: Gravitational parameter [m^3/s^2].
        prograde: Transfer in sense of normal if True.
        normal: Reference orbital normal (defaults to +z).

    Returns:
        Tuple (v1, v2) of departure and arrival velocity vectors [m/s].

    Raises:
        ValueError: If geometry is degenerate or tof is outside valid range.
    """
    ref = np.array([0.0, 0.0, 1.0]) if normal is None else normal
    r1n, r2n = float(np.linalg.norm(r1)), float(np.linalg.norm(r2))
    cos_d = float(np.clip(np.dot(r1, r2) / (r1n * r2n), -1.0, 1.0))
    dtheta = math.acos(cos_d)
    sense = float(np.dot(np.cross(r1, r2), ref))
    if (prograde and sense < 0.0) or (not prograde and sense >= 0.0):
        dtheta = 2.0 * math.pi - dtheta
    a_par = math.sin(dtheta) * math.sqrt(r1n * r2n / (1.0 - cos_d))
    if abs(a_par) < 1e-9:
        raise ValueError("Lambert geometry degenerate (transfer angle 0 or pi)")
    sqmu = math.sqrt(mu)

    def y_of(z: float) -> float:
        return r1n + r2n + a_par * (z * stumpff_s(z) - 1.0) / math.sqrt(stumpff_c(z))

    def residual(z: float) -> float:
        y = y_of(z)
        if y < 0.0:
            return -math.inf if a_par > 0.0 else math.inf
        c, s = stumpff_c(z), stumpff_s(z)
        return (y / c) ** 1.5 * s + a_par * math.sqrt(y) - sqmu * tof

    lo, hi = -400.0, 4.0 * math.pi**2 * (1.0 - 1e-6)
    if not (residual(lo) < 0.0 < residual(hi)):
        raise ValueError("Time of flight outside single-revolution Lambert range")
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if residual(mid) < 0.0:
            lo = mid
        else:
            hi = mid
    y = y_of(0.5 * (lo + hi))
    f = 1.0 - y / r1n
    g = a_par * math.sqrt(y / mu)
    gdot = 1.0 - y / r2n
    return (r2 - f * r1) / g, (gdot * r2 - r1) / g
