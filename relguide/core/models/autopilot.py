"""Autopilot response lag and actuator rate saturation models."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]


def _saturate_norm(vec: Vec, limit: float) -> Vec:
    """Scale ``vec`` so its Euclidean norm does not exceed ``limit``."""
    norm = float(np.linalg.norm(vec))
    return vec if norm <= limit else vec * (limit / norm)


class AutopilotModel:
    """Models guidance actuator lag and rate/magnitude limits.

    Dynamics (first-order response lag)::

        da/dt = (u_cmd - a) / tau_a

    subject to slew rate limit ``|da/dt| <= rate_limit`` and acceleration limit
    ``|a| <= max_accel``.

    Args:
        tau_a: Autopilot time constant [s]. If <= 0, response is instantaneous.
        max_accel: Maximum achievable acceleration [m/s^2].
        rate_limit: Maximum rate of change of acceleration [m/s^3] (optional).
    """

    def __init__(
        self,
        tau_a: float = 0.0,
        max_accel: float = 50.0,
        rate_limit: float | None = None,
    ) -> None:
        self.tau_a = tau_a
        self.max_accel = max_accel
        self.rate_limit = rate_limit
        self.current_accel: Vec = np.zeros(3)

    def reset(self) -> None:
        """Reset current applied acceleration to zero."""
        self.current_accel = np.zeros(3)

    def step(self, u_cmd: Vec, dt: float) -> Vec:
        """Advance autopilot dynamics by ``dt`` seconds.

        Args:
            u_cmd: Commanded acceleration vector [m/s^2].
            dt: Time step [s].

        Returns:
            Achieved acceleration vector [m/s^2].
        """
        # Command saturation
        cmd_sat = _saturate_norm(u_cmd, self.max_accel)

        if self.tau_a <= 1e-4:
            self.current_accel = cmd_sat
            return self.current_accel.copy()

        # First order lag derivative
        accel_deriv = (cmd_sat - self.current_accel) / self.tau_a

        # Slew rate saturation
        if self.rate_limit is not None and self.rate_limit > 0:
            accel_deriv = _saturate_norm(accel_deriv, self.rate_limit)

        # Euler step
        self.current_accel = self.current_accel + accel_deriv * dt
        self.current_accel = _saturate_norm(self.current_accel, self.max_accel)
        return self.current_accel.copy()
