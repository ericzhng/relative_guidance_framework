"""Unified guidance state representation for all relative guidance applications."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.kinematics import (
    compute_closing_speed,
    compute_look_angle,
    compute_los,
    compute_los_rate_vector,
    compute_range,
    compute_range_rate,
)

Vec = NDArray[np.float64]


@dataclass(frozen=True)
class GuidanceState:
    """Information available to a guidance law at each guidance cycle.

    Attributes:
        t: Elapsed mission time [s].
        rel_pos: Target position relative to the chaser [m] (x = r_t - r_c).
        rel_vel: Target velocity relative to the chaser [m/s] (xdot = v_t - v_c).
        t_final: Planned arrival time on the mission clock [s] (optional).
        target_accel: Estimated target acceleration [m/s^2] (optional, for APN).
        chaser_vel: Chaser velocity in inertial frame [m/s] (optional, for look angle/PPN).
    """

    t: float
    rel_pos: Vec
    rel_vel: Vec
    t_final: float | None = None
    target_accel: Vec | None = None
    chaser_vel: Vec | None = None
    lvlh_state: Vec | None = None

    @property
    def range(self) -> float:
        """Line-of-sight distance [m]."""
        return compute_range(self.rel_pos)

    @property
    def los(self) -> Vec:
        """Unit line-of-sight vector from chaser to target."""
        return compute_los(self.rel_pos)

    @property
    def range_rate(self) -> float:
        """Range rate [m/s]; negative while closing."""
        return compute_range_rate(self.rel_pos, self.rel_vel)

    @property
    def closing_speed(self) -> float:
        """Closing speed V_c = -range_rate [m/s]."""
        return compute_closing_speed(self.rel_pos, self.rel_vel)

    @property
    def los_rate_vector(self) -> Vec:
        """Inertial line-of-sight angular velocity vector Omega [rad/s]."""
        return compute_los_rate_vector(self.rel_pos, self.rel_vel)

    @property
    def los_rate(self) -> float:
        """Scalar magnitude of the LOS angular velocity [rad/s]."""
        return float(np.linalg.norm(self.los_rate_vector))

    @property
    def look_angle(self) -> float:
        """Look angle between chaser velocity and line-of-sight [rad]."""
        if self.chaser_vel is None:
            return 0.0
        return compute_look_angle(self.chaser_vel, self.rel_pos)

    @property
    def t_go(self) -> float:
        """Time remaining to planned arrival [s], or 0.0 if not specified."""
        if self.t_final is None:
            return 0.0
        return self.t_final - self.t
