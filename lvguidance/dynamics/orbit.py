"""Two-body point-mass dynamics and fixed-step RK4 propagation."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]

MU_EARTH = 3.986004418e14  # Earth gravitational parameter [m^3/s^2]
R_EARTH = 6.378137e6  # Earth equatorial radius [m]


def gravity(r: Vec, mu: float = MU_EARTH) -> Vec:
    """Central-body gravitational acceleration.

    Args:
        r: Inertial position vector [m].
        mu: Gravitational parameter [m^3/s^2].

    Returns:
        Gravitational acceleration vector [m/s^2].
    """
    return -mu * r / np.linalg.norm(r) ** 3


def circular_state(
    radius: float,
    inclination: float = 0.0,
    phase: float = 0.0,
    mu: float = MU_EARTH,
) -> tuple[Vec, Vec]:
    """Build the inertial state of a circular orbit.

    The orbit starts on the line of nodes rotated by ``phase`` about the orbit normal.

    Args:
        radius: Orbit radius [m].
        inclination: Orbit inclination [rad].
        phase: Argument of latitude at epoch [rad].
        mu: Gravitational parameter [m^3/s^2].

    Returns:
        Tuple ``(r, v)`` of inertial position [m] and velocity [m/s].
    """
    speed = np.sqrt(mu / radius)
    ci, si = np.cos(inclination), np.sin(inclination)
    cp, sp = np.cos(phase), np.sin(phase)
    r_hat = np.array([cp, sp * ci, sp * si])
    t_hat = np.array([-sp, cp * ci, cp * si])
    return radius * r_hat, speed * t_hat


def _deriv(state: Vec, accel: Vec, mu: float) -> Vec:
    """Derivative of a 6-state ``[r, v]`` under gravity plus constant thrust acceleration."""
    return np.concatenate((state[3:], gravity(state[:3], mu) + accel))


def propagate_step(
    state: Vec,
    accel: Vec,
    dt: float,
    substeps: int = 4,
    mu: float = MU_EARTH,
) -> Vec:
    """Advance a body by ``dt`` with RK4, holding the thrust acceleration constant.

    Args:
        state: Inertial state ``[x, y, z, vx, vy, vz]``.
        accel: Constant non-gravitational acceleration over the step [m/s^2].
        dt: Step length [s].
        substeps: Number of RK4 sub-steps inside ``dt``.
        mu: Gravitational parameter [m^3/s^2].

    Returns:
        State at the end of the step.
    """
    h = dt / substeps
    y = np.asarray(state, dtype=float)
    for _ in range(substeps):
        k1 = _deriv(y, accel, mu)
        k2 = _deriv(y + 0.5 * h * k1, accel, mu)
        k3 = _deriv(y + 0.5 * h * k2, accel, mu)
        k4 = _deriv(y + h * k3, accel, mu)
        y = y + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    return y
