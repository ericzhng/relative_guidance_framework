"""Relative kinematics and line-of-sight (LOS) geometry in 3D."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]


def compute_range(rel_pos: Vec) -> float:
    """Euclidean distance between chaser and target [m]."""
    return float(np.linalg.norm(rel_pos))


def compute_los(rel_pos: Vec, eps: float = 1e-9) -> Vec:
    """Unit line-of-sight vector pointing from chaser to target."""
    rng = max(compute_range(rel_pos), eps)
    return rel_pos / rng


def compute_range_rate(rel_pos: Vec, rel_vel: Vec, eps: float = 1e-9) -> float:
    """Range rate [m/s]; negative while closing."""
    los = compute_los(rel_pos, eps=eps)
    return float(los @ rel_vel)


def compute_closing_speed(rel_pos: Vec, rel_vel: Vec, eps: float = 1e-9) -> float:
    """Closing speed V_c = -dR/dt [m/s]."""
    return -compute_range_rate(rel_pos, rel_vel, eps=eps)


def compute_los_rate_vector(rel_pos: Vec, rel_vel: Vec, eps: float = 1e-9) -> Vec:
    """Inertial line-of-sight angular velocity vector Omega = (x cross xdot) / R^2 [rad/s]."""
    r2 = max(float(rel_pos @ rel_pos), eps**2)
    return np.cross(rel_pos, rel_vel) / r2


def compute_los_derivative(rel_pos: Vec, rel_vel: Vec, eps: float = 1e-9) -> Vec:
    """Time derivative of the unit LOS vector: d(l_hat)/dt = Omega cross l_hat [1/s]."""
    omega = compute_los_rate_vector(rel_pos, rel_vel, eps=eps)
    los = compute_los(rel_pos, eps=eps)
    return np.cross(omega, los)


def compute_look_angle(chaser_vel: Vec, rel_pos: Vec, eps: float = 1e-9) -> float:
    """Angle between the chaser's velocity vector and the line of sight [rad]."""
    speed = float(np.linalg.norm(chaser_vel))
    if speed < eps:
        return 0.0
    los = compute_los(rel_pos, eps=eps)
    cos_angle = float(np.clip((chaser_vel / speed) @ los, -1.0, 1.0))
    return float(np.arccos(cos_angle))


def estimate_t_go(
    range_m: float,
    closing_speed_mps: float,
    accel_nom: float = 0.2,
    t_go_min: float = 0.5,
) -> float:
    """Estimate time-to-go using closing speed and acceleration capabilities [s].

    Combines closing-speed extrapolation with a nominal acceleration cap:
    t_cap = 2 * sqrt(R / a_nom).
    """
    cap = 2.0 * float(np.sqrt(max(range_m, 0.0) / max(accel_nom, 1e-3)))
    if closing_speed_mps > 1e-3:
        stop_time = 2.0 * range_m / closing_speed_mps
        t_est = min(stop_time, cap)
    else:
        t_est = cap
    return max(t_est, t_go_min)
