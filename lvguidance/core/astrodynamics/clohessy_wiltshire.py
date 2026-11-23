"""Linear Clohessy-Wiltshire (Hill) relative dynamics and state transition matrices."""

from __future__ import annotations

import math
import numpy as np
from numpy.typing import NDArray
from scipy.linalg import expm

from lvguidance.core.astrodynamics.constants import MU_EARTH

Mat = NDArray[np.float64]
Vec = NDArray[np.float64]

_SINGULAR_COND = 1e8


def mean_motion(radius: float, mu: float = MU_EARTH) -> float:
    """Mean motion sqrt(mu / a^3) of a circular orbit [rad/s]."""
    return math.sqrt(mu / radius**3)


def cw_matrices(n: float) -> tuple[Mat, Mat]:
    """Continuous CW state-space model x_dot = A x + B a in LVLH frame.

    State: [x, y, z, vx, vy, vz] in LVLH (x radial, y along-track, z cross-track).
    Input: a is continuous specific thrust acceleration [m/s^2].
    """
    a_mat = np.zeros((6, 6))
    a_mat[:3, 3:] = np.eye(3)
    a_mat[3, 0] = 3.0 * n**2
    a_mat[5, 2] = -(n**2)
    a_mat[3, 4] = 2.0 * n
    a_mat[4, 3] = -2.0 * n
    b_mat = np.zeros((6, 3))
    b_mat[3:, :] = np.eye(3)
    return a_mat, b_mat


def cw_stm(n: float, t: float) -> Mat:
    """Closed-form Clohessy-Wiltshire state transition matrix Phi(t)."""
    nt = n * t
    s, c = math.sin(nt), math.cos(nt)
    phi = np.zeros((6, 6))
    phi[0, 0] = 4.0 - 3.0 * c
    phi[1, 0] = 6.0 * (s - nt)
    phi[1, 1] = 1.0
    phi[2, 2] = c
    phi[0, 3] = s / n
    phi[0, 4] = 2.0 * (1.0 - c) / n
    phi[1, 3] = 2.0 * (c - 1.0) / n
    phi[1, 4] = (4.0 * s - 3.0 * nt) / n
    phi[2, 5] = s / n
    phi[3, 0] = 3.0 * n * s
    phi[4, 0] = 6.0 * n * (c - 1.0)
    phi[5, 2] = -n * s
    phi[3, 3] = c
    phi[3, 4] = 2.0 * s
    phi[4, 3] = -2.0 * s
    phi[4, 4] = 4.0 * c - 3.0
    phi[5, 5] = c
    return phi


def discretize_zoh(n: float, dt: float) -> tuple[Mat, Mat]:
    """Exact zero-order-hold (ZOH) discretization of the CW model x[k+1] = Ad x[k] + Bd a[k]."""
    a_mat, b_mat = cw_matrices(n)
    aug = np.zeros((9, 9))
    aug[:6, :6] = a_mat * dt
    aug[:6, 6:] = b_mat * dt
    e_aug = expm(aug)
    return e_aug[:6, :6], e_aug[:6, 6:]


def cw_required_velocity(
    n: float, tau: float, r0: NDArray[np.float64], r_goal: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Initial velocity that transfers the chaser from r0 to r_goal in time tau under CW.

    Solves: r_goal = Phi_rr * r0 + Phi_rv * v for v.
    """
    phi = cw_stm(n, tau)
    phi_rv = phi[:3, 3:]
    if np.linalg.cond(phi_rv) > _SINGULAR_COND:
        raise ValueError(
            f"CW targeting is singular for transfer time {tau:.1f} s "
            "(n*tau is a multiple of 2*pi); choose a different duration"
        )
    return np.linalg.solve(phi_rv, r_goal - phi[:3, :3] @ r0)
