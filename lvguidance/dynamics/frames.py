"""Local-Vertical-Local-Horizontal (LVLH) frame utilities."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]


def lvlh_rotation(r_ref: Vec, v_ref: Vec) -> NDArray[np.float64]:
    """Rotation matrix from the inertial frame to the LVLH frame of a reference body.

    Axes: x radial (outward), z orbit normal, y completes the right-handed triad
    (along-track for a circular orbit).

    Args:
        r_ref: Reference inertial position, shape ``(3,)`` or ``(N, 3)``.
        v_ref: Reference inertial velocity, same shape as ``r_ref``.

    Returns:
        Matrix of shape ``(3, 3)`` or ``(N, 3, 3)`` whose rows are the LVLH axes.
    """
    r_ref = np.atleast_2d(r_ref)
    v_ref = np.atleast_2d(v_ref)
    x = r_ref / np.linalg.norm(r_ref, axis=1, keepdims=True)
    h = np.cross(r_ref, v_ref)
    z = h / np.linalg.norm(h, axis=1, keepdims=True)
    y = np.cross(z, x)
    rot = np.stack((x, y, z), axis=1)
    return rot[0] if rot.shape[0] == 1 else rot


def to_lvlh(vec: Vec, r_ref: Vec, v_ref: Vec) -> Vec:
    """Express inertial vectors in the LVLH frame of a reference body.

    Args:
        vec: Inertial vectors, shape ``(3,)`` or ``(N, 3)``.
        r_ref: Reference position(s), shape matching ``vec``.
        v_ref: Reference velocity(ies), shape matching ``vec``.

    Returns:
        Vectors in the LVLH frame with the same shape as ``vec``.
    """
    rot = lvlh_rotation(r_ref, v_ref)
    if rot.ndim == 2:
        return vec @ rot.T
    return np.einsum("nij,nj->ni", rot, vec)
