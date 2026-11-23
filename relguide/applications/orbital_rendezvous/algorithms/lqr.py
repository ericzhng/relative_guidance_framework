"""Continuous-thrust discrete-time LQR regulator on CW dynamics."""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.linalg import solve_discrete_are

from relguide.core.astrodynamics.clohessy_wiltshire import discretize_zoh
from relguide.core.base import GuidanceLaw
from relguide.core.state import GuidanceState

Vec = NDArray[np.float64]
Mat = NDArray[np.float64]


def bryson_weights(pos_tol: float, vel_tol: float, acc_tol: float) -> tuple[Mat, Mat]:
    """Bryson-rule weighting matrices Q = diag(1/tol^2) and R = I/acc_tol^2."""
    q = np.diag([1.0 / pos_tol**2] * 3 + [1.0 / vel_tol**2] * 3)
    r = np.eye(3) / acc_tol**2
    return q, r


class LQRController(GuidanceLaw):
    """Infinite-horizon discrete-time LQR on the Clohessy-Wiltshire state space.

    Gains are computed by solving the discrete algebraic Riccati equation (DARE)
    and continuous acceleration commands are saturated per axis:
    a = clip(-K(x - goal), -a_max, a_max).

    Args:
        pos_tol: Position error scaling for one unit of cost [m].
        vel_tol: Velocity error scaling for one unit of cost [m/s].
        acc_tol: Acceleration scaling for one unit of cost [m/s^2].
        a_max: Per-axis acceleration saturation limit [m/s^2].
        dt: Control cycle period [s].
        label: Display name.
    """

    name = "LQR"

    def __init__(
        self,
        pos_tol: float = 100.0,
        vel_tol: float = 0.5,
        acc_tol: float = 2e-3,
        a_max: float = 5e-3,
        dt: float = 10.0,
        label: str = "LQR",
    ) -> None:
        self.name = label
        self.a_max = a_max
        self.control_dt = dt
        self._weights = bryson_weights(pos_tol, vel_tol, acc_tol)
        self._gain = np.zeros((3, 6))
        self._goal = np.zeros(6)

    def reset(self, scenario: Any = None) -> None:
        """Solve DARE for the scenario's mean motion."""
        assert self.control_dt is not None
        n = getattr(scenario, "mean_motion", 0.0011)
        self._goal = getattr(scenario, "goal", np.zeros(6))

        a_d, b_d = discretize_zoh(n, self.control_dt)
        q, r = self._weights
        p = solve_discrete_are(a_d, b_d, q, r)
        self._gain = np.linalg.solve(r + b_d.T @ p @ b_d, b_d.T @ p @ a_d)

    def command(self, state: GuidanceState) -> Vec:
        """Compute continuous acceleration in LVLH frame: -K (x - goal)."""
        x = (
            state.lvlh_state
            if state.lvlh_state is not None
            else np.concatenate((state.rel_pos, state.rel_vel))
        )
        return np.clip(-self._gain @ (x - self._goal), -self.a_max, self.a_max)
