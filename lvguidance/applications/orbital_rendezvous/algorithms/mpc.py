"""Receding-horizon Model Predictive Control (MPC) with hard box thrust limits."""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.linalg import cholesky, solve_discrete_are
from scipy.optimize import lsq_linear

from lvguidance.applications.orbital_rendezvous.algorithms.lqr import bryson_weights
from lvguidance.core.astrodynamics.clohessy_wiltshire import discretize_zoh
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.state import GuidanceState

Vec = NDArray[np.float64]
Mat = NDArray[np.float64]


class MPCController(GuidanceLaw):
    """Linear Model Predictive Control (MPC) on CW dynamics with hard box thrust bounds.

    At each guidance step, solves the finite-horizon constrained least-squares problem:
    min sum_k (e_k' Q e_k + u_k' R u_k) + e_N' P e_N  s.t. |u_k| <= a_max
    using bounded linear least squares (`lsq_linear`), applying the first control increment.

    Args:
        horizon: Prediction horizon in steps.
        dt: Control and prediction period [s].
        pos_tol: Position error scaling for one unit of cost [m].
        vel_tol: Velocity error scaling for one unit of cost [m/s].
        acc_tol: Acceleration scaling for one unit of cost [m/s^2].
        a_max: Per-axis acceleration saturation limit [m/s^2].
        label: Display name.
    """

    name = "MPC"

    def __init__(
        self,
        horizon: int = 30,
        dt: float = 30.0,
        pos_tol: float = 100.0,
        vel_tol: float = 0.5,
        acc_tol: float = 2e-3,
        a_max: float = 5e-3,
        label: str = "MPC",
    ) -> None:
        self.name = label
        self.horizon = horizon
        self.a_max = a_max
        self.control_dt = dt
        self._weights = bryson_weights(pos_tol, vel_tol, acc_tol)
        self._goal = np.zeros(6)
        self._free = np.zeros((6 * horizon, 6))
        self._forced = np.zeros((6 * horizon, 3 * horizon))
        self._cost_state = np.zeros((6 * horizon, 6 * horizon))
        self._cost_input = np.zeros((3 * horizon, 3 * horizon))

    def reset(self, scenario: Any = None) -> None:
        """Build condensed prediction matrices and square-root cost structures."""
        assert self.control_dt is not None
        n_h = self.horizon
        n = getattr(scenario, "mean_motion", 0.0011)
        self._goal = getattr(scenario, "goal", np.zeros(6))

        a_d, b_d = discretize_zoh(n, self.control_dt)
        q, r = self._weights
        p = solve_discrete_are(a_d, b_d, q, r)

        free = np.zeros((6 * n_h, 6))
        forced = np.zeros((6 * n_h, 3 * n_h))
        power = np.eye(6)
        for i in range(n_h):
            power = a_d @ power
            free[6 * i : 6 * i + 6] = power
            for j in range(i + 1):
                forced[6 * i : 6 * i + 6, 3 * j : 3 * j + 3] = (
                    np.linalg.matrix_power(a_d, i - j) @ b_d
                )

        root_q = np.linalg.cholesky(q).T
        root_p = cholesky(p)
        cost_state = np.zeros((6 * n_h, 6 * n_h))
        for i in range(n_h):
            blk = root_p if i == n_h - 1 else root_q
            cost_state[6 * i : 6 * i + 6, 6 * i : 6 * i + 6] = blk

        self._free = free
        self._forced = forced
        self._cost_state = cost_state
        self._cost_input = np.kron(np.eye(n_h), np.linalg.cholesky(r).T)

    def command(self, state: GuidanceState) -> Vec:
        """Solve bounded least squares and return first control acceleration."""
        x = state.lvlh_state if state.lvlh_state is not None else np.concatenate((state.rel_pos, state.rel_vel))
        err = x - self._goal
        mat = np.vstack((self._cost_state @ self._forced, self._cost_input))
        rhs = np.concatenate(
            (-self._cost_state @ self._free @ err, np.zeros(self._cost_input.shape[0]))
        )
        sol = lsq_linear(mat, rhs, bounds=(-self.a_max, self.a_max), method="bvls")
        return np.asarray(sol.x[:3])
