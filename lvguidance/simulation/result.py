"""Container for simulation histories and derived relative-motion quantities."""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
from numpy.typing import NDArray

from lvguidance.dynamics.frames import to_lvlh

Arr = NDArray[np.float64]


@dataclass(frozen=True)
class SimulationResult:
    """Time histories of one closed-loop run (all vectors inertial, row per sample).

    Attributes:
        law_name: Name of the guidance law.
        t: Time stamps [s], shape ``(N,)``.
        target: Target states ``[r, v]``, shape ``(N, 6)``.
        chaser: Chaser states ``[r, v]``, shape ``(N, 6)``.
        accel: Applied thrust acceleration, shape ``(N, 3)`` [m/s^2].
        max_accel: Acceleration limit used in the run [m/s^2].
        events: Named mission-time events, e.g. ``"handover"`` [s].
    """

    law_name: str
    t: Arr
    target: Arr
    chaser: Arr
    accel: Arr
    max_accel: float
    events: dict[str, float] = field(default_factory=dict)

    @cached_property
    def rel_pos(self) -> Arr:
        """Target minus chaser position [m], inertial axes."""
        return self.target[:, :3] - self.chaser[:, :3]

    @cached_property
    def rel_vel(self) -> Arr:
        """Target minus chaser velocity [m/s], inertial axes."""
        return self.target[:, 3:] - self.chaser[:, 3:]

    @cached_property
    def range(self) -> Arr:
        """Line-of-sight distance [m]."""
        return np.linalg.norm(self.rel_pos, axis=1)

    @cached_property
    def range_rate(self) -> Arr:
        """Range rate [m/s] (negative while closing)."""
        return np.einsum("ij,ij->i", self.rel_pos, self.rel_vel) / np.maximum(self.range, 1e-9)

    @cached_property
    def rel_speed(self) -> Arr:
        """Magnitude of the relative velocity [m/s]."""
        return np.linalg.norm(self.rel_vel, axis=1)

    @cached_property
    def chaser_lvlh(self) -> Arr:
        """Chaser position in the target's LVLH frame [m] (x radial, y along-track, z normal)."""
        return to_lvlh(-self.rel_pos, self.target[:, :3], self.target[:, 3:])

    @cached_property
    def los_lvlh(self) -> Arr:
        """Chaser-to-target line-of-sight vector in the target LVLH frame [m]."""
        return to_lvlh(self.rel_pos, self.target[:, :3], self.target[:, 3:])

    @cached_property
    def los_azimuth(self) -> Arr:
        """LOS azimuth in the LVLH x-y plane [rad]."""
        los = self.los_lvlh
        return np.arctan2(los[:, 1], los[:, 0])

    @cached_property
    def los_elevation(self) -> Arr:
        """LOS elevation above the LVLH x-y plane [rad]."""
        los = self.los_lvlh
        return np.arctan2(los[:, 2], np.hypot(los[:, 0], los[:, 1]))

    @cached_property
    def los_rate(self) -> Arr:
        """Magnitude of the inertial line-of-sight angular rate [rad/s]."""
        cross = np.linalg.norm(np.cross(self.rel_pos, self.rel_vel), axis=1)
        return cross / np.maximum(self.range**2, 1e-9)

    @cached_property
    def approach_angle(self) -> Arr:
        """Angle between the closing velocity and the line of sight [rad].

        Undefined (NaN) while the relative speed is below 1 cm/s.
        """
        denom = np.maximum(self.range * self.rel_speed, 1e-9)
        cos = np.clip(-np.einsum("ij,ij->i", self.rel_pos, self.rel_vel) / denom, -1.0, 1.0)
        angle = np.arccos(cos)
        angle[self.rel_speed < 1e-2] = np.nan
        return angle

    @cached_property
    def accel_norm(self) -> Arr:
        """Magnitude of the applied thrust acceleration [m/s^2]."""
        return np.linalg.norm(self.accel, axis=1)

    @cached_property
    def delta_v(self) -> Arr:
        """Cumulative delta-v spent [m/s]."""
        dt = np.diff(self.t, prepend=self.t[0])
        return np.cumsum(self.accel_norm * dt)
