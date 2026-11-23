"""Two-body orbital dynamics, CW dynamics, and frame transformations."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.astrodynamics import (
    MU_EARTH,
    R_EARTH,
    cw_matrices,
    cw_required_velocity,
    cw_stm,
    discretize_zoh,
    eci_to_lvlh,
    lvlh_angular_velocity,
    lvlh_dcm,
    lvlh_to_eci,
    mean_motion,
    propagate_kepler,
    solve_lambert,
)
from lvguidance.applications.orbital_rendezvous.scenario import circular_state, gravity

Vec = NDArray[np.float64]


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


__all__ = [
    "MU_EARTH",
    "R_EARTH",
    "circular_state",
    "cw_matrices",
    "cw_required_velocity",
    "cw_stm",
    "discretize_zoh",
    "eci_to_lvlh",
    "gravity",
    "lvlh_angular_velocity",
    "lvlh_dcm",
    "lvlh_rotation",
    "lvlh_to_eci",
    "mean_motion",
    "propagate_kepler",
    "solve_lambert",
    "to_lvlh",
]
