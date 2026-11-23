"""Exact transformations between inertial (ECI) and rotating LVLH (Hill) frames."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]


def lvlh_dcm(r_t: Vec, v_t: Vec) -> NDArray[np.float64]:
    """Direction cosine matrix from ECI to the target LVLH frame.

    Rows are x (radial), y (along-track), and z (orbit-normal) unit vectors in ECI.
    """
    r_norm = np.linalg.norm(r_t)
    x_hat = r_t / r_norm
    h = np.cross(r_t, v_t)
    z_hat = h / np.linalg.norm(h)
    y_hat = np.cross(z_hat, x_hat)
    return np.vstack((x_hat, y_hat, z_hat))


def lvlh_angular_velocity(r_t: Vec, v_t: Vec) -> Vec:
    """Inertial angular velocity vector of the LVLH frame: omega = h / r^2 [rad/s]."""
    return np.cross(r_t, v_t) / float(np.dot(r_t, r_t))


def eci_to_lvlh(r_t: Vec, v_t: Vec, r_c: Vec, v_c: Vec) -> Vec:
    """Transform inertial target and chaser states into the target-centred LVLH state.

    Returns:
        6-vector [x, y, z, vx, vy, vz] in LVLH (velocity differentiated in rotating frame).
    """
    rot = lvlh_dcm(r_t, v_t)
    omega = lvlh_angular_velocity(r_t, v_t)
    rho = r_c - r_t
    rho_dot = v_c - v_t - np.cross(omega, rho)
    return np.concatenate((rot @ rho, rot @ rho_dot))


def lvlh_to_eci(r_t: Vec, v_t: Vec, rel: Vec) -> tuple[Vec, Vec]:
    """Transform target ECI state and relative LVLH state into chaser ECI state."""
    rot = lvlh_dcm(r_t, v_t)
    omega = lvlh_angular_velocity(r_t, v_t)
    rho = rot.T @ rel[:3]
    rho_dot = rot.T @ rel[3:]
    r_c = r_t + rho
    v_c = v_t + rho_dot + np.cross(omega, rho)
    return r_c, v_c


to_lvlh = eci_to_lvlh


