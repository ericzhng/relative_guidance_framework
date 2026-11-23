"""Two-body orbital dynamics and Local-Vertical-Local-Horizontal (LVLH) frame."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]

MU_EARTH = 3.986004418e14  # Earth gravitational parameter [m^3/s^2]
R_EARTH = 6.378137e6  # Earth equatorial radius [m]


def gravity(r: Vec, mu: float = MU_EARTH) -> Vec:
    """Central-body gravitational acceleration vector [m/s^2]."""
    r_norm = float(np.linalg.norm(r))
    return -mu * r / max(r_norm**3, 1e-9)


def circular_state(
    radius: float,
    inclination: float = 0.0,
    phase: float = 0.0,
    mu: float = MU_EARTH,
) -> tuple[Vec, Vec]:
    """Build inertial state [r, v] of a circular orbit."""
    speed = float(np.sqrt(mu / radius))
    ci, si = np.cos(inclination), np.sin(inclination)
    cp, sp = np.cos(phase), np.sin(phase)
    r_hat = np.array([cp, sp * ci, sp * si], dtype=float)
    t_hat = np.array([-sp, cp * ci, cp * si], dtype=float)
    return radius * r_hat, speed * t_hat


def lvlh_rotation(r_ref: Vec, v_ref: Vec) -> NDArray[np.float64]:
    """Rotation matrix from inertial frame to the LVLH frame of a reference body."""
    r_ref = np.atleast_2d(r_ref)
    v_ref = np.atleast_2d(v_ref)
    x = r_ref / np.linalg.norm(r_ref, axis=1, keepdims=True)
    h = np.cross(r_ref, v_ref)
    z = h / np.linalg.norm(h, axis=1, keepdims=True)
    y = np.cross(z, x)
    rot = np.stack((x, y, z), axis=1)
    return rot[0] if rot.shape[0] == 1 else rot


def to_lvlh(vec: Vec, r_ref: Vec, v_ref: Vec) -> Vec:
    """Express inertial vectors in the LVLH frame of a reference body."""
    rot = lvlh_rotation(r_ref, v_ref)
    if rot.ndim == 2:
        return vec @ rot.T
    return np.einsum("nij,nj->ni", rot, vec)
