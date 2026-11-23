"""Container for simulation histories and derived kinematic trajectories."""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
from numpy.typing import NDArray

Arr = NDArray[np.float64]


@dataclass(frozen=True)
class SimulationResult:
    """Time histories of one closed-loop guidance run.

    Attributes:
        law_name: Name of the guidance law.
        t: Time stamps [s], shape (N,).
        target: Target states [r, v], shape (N, 6).
        chaser: Chaser states [r, v], shape (N, 6).
        accel: Applied acceleration history, shape (N, 3) [m/s^2].
        cmd_accel: Commanded acceleration history, shape (N, 3) [m/s^2].
        max_accel: Maximum vehicle acceleration capability [m/s^2].
        events: Named mission-time events (e.g. {"handover": 250.0}).
    """

    law_name: str
    t: Arr
    target: Arr
    chaser: Arr
    accel: Arr
    cmd_accel: Arr
    max_accel: float
    events: dict[str, float] = field(default_factory=dict)

    @cached_property
    def rel_pos(self) -> Arr:
        """Relative position x = r_target - r_chaser [m]."""
        return self.target[:, :3] - self.chaser[:, :3]

    @cached_property
    def rel_vel(self) -> Arr:
        """Relative velocity xdot = v_target - v_chaser [m/s]."""
        return self.target[:, 3:] - self.chaser[:, 3:]

    @cached_property
    def range(self) -> Arr:
        """Range to target [m]."""
        return np.linalg.norm(self.rel_pos, axis=1)

    @cached_property
    def miss_distance(self) -> float:
        """Closest Point of Approach (CPA) / minimum miss distance [m].

        Computes the analytical minimum distance along the continuous trajectory
        between integration samples.
        """
        if len(self.rel_pos) < 2:
            return float(np.min(self.range))
        idx = int(np.argmin(self.range))
        min_d = float(self.range[idx])
        for i in (idx - 1, idx):
            if 0 <= i < len(self.rel_pos) - 1:
                p0 = self.rel_pos[i]
                v = self.rel_vel[i]
                v2 = float(v @ v)
                if v2 > 1e-6:
                    dt_seg = float(self.t[i + 1] - self.t[i])
                    tau_star = float(np.clip(-float(p0 @ v) / v2, 0.0, dt_seg))
                    p_cpa = p0 + v * tau_star
                    d_cpa = float(np.linalg.norm(p_cpa))
                    if d_cpa < min_d:
                        min_d = d_cpa
        return min_d

    @cached_property
    def range_rate(self) -> Arr:
        """Range rate [m/s] (negative while closing)."""
        return np.einsum("ij,ij->i", self.rel_pos, self.rel_vel) / np.maximum(self.range, 1e-9)

    @cached_property
    def closing_speed(self) -> Arr:
        """Closing speed V_c = -dR/dt [m/s]."""
        return -self.range_rate

    @cached_property
    def rel_speed(self) -> Arr:
        """Relative speed norm [m/s]."""
        return np.linalg.norm(self.rel_vel, axis=1)

    @cached_property
    def los_rate(self) -> Arr:
        """Inertial line-of-sight angular rate [rad/s]."""
        cross = np.linalg.norm(np.cross(self.rel_pos, self.rel_vel), axis=1)
        return cross / np.maximum(self.range**2, 1e-9)

    @cached_property
    def accel_norm(self) -> Arr:
        """Applied acceleration magnitude [m/s^2]."""
        return np.linalg.norm(self.accel, axis=1)

    @cached_property
    def delta_v(self) -> Arr:
        """Cumulative delta-v spent [m/s]."""
        dt = np.diff(self.t, prepend=self.t[0])
        return np.cumsum(self.accel_norm * dt)
