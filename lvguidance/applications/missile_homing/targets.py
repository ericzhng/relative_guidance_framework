"""Target maneuver models for missile homing intercept evaluations."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]


class TargetManeuver(ABC):
    """Abstract target motion and maneuver profile."""

    @abstractmethod
    def acceleration(self, t: float, target_state: Vec) -> Vec:
        """Compute target acceleration at time t [m/s^2]."""


class NonManeuveringTarget(TargetManeuver):
    """Target travels with constant velocity (zero acceleration)."""

    def acceleration(self, t: float, target_state: Vec) -> Vec:
        """Zero maneuver acceleration."""
        return np.zeros(3)


class ConstantTurnTarget(TargetManeuver):
    """Target pulls a constant lateral acceleration perpendicular to velocity.

    Args:
        g_accel: Maneuver load factor [m/s^2] (e.g. 3 * 9.81 for 3g).
        plane_normal: Unit normal defining the maneuver plane.
    """

    def __init__(self, g_accel: float = 3.0 * 9.81, plane_normal: Vec | None = None) -> None:
        self.g_accel = g_accel
        self.plane_normal = (
            plane_normal / np.linalg.norm(plane_normal)
            if plane_normal is not None
            else np.array([0.0, 0.0, 1.0])
        )

    def acceleration(self, t: float, target_state: Vec) -> Vec:
        """Compute constant lateral acceleration normal to velocity."""
        vel = target_state[3:]
        speed = float(np.linalg.norm(vel))
        if speed < 1e-3:
            return np.zeros(3)
        v_hat = vel / speed
        lat_dir = np.cross(self.plane_normal, v_hat)
        norm_lat = float(np.linalg.norm(lat_dir))
        if norm_lat < 1e-6:
            return np.zeros(3)
        return self.g_accel * (lat_dir / norm_lat)


class WeavingTarget(TargetManeuver):
    """Target executes an evasive sinusoidal weaving maneuver.

    Acceleration profile::

        a_T(t) = a_max * sin(omega * t + phi) * n_hat

    Args:
        amplitude: Maneuver acceleration peak amplitude [m/s^2] (e.g. 4 * 9.81).
        frequency: Weaving angular frequency omega [rad/s].
        phase: Initial phase phi [rad].
        direction: Lateral maneuver axis unit vector.
    """

    def __init__(
        self,
        amplitude: float = 4.0 * 9.81,
        frequency: float = 0.5,
        phase: float = 0.0,
        direction: Vec | None = None,
    ) -> None:
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase
        axis = direction if direction is not None else np.array([0.0, 0.0, 1.0])
        self.direction = axis / np.linalg.norm(axis)

    def acceleration(self, t: float, target_state: Vec) -> Vec:
        """Compute weaving acceleration."""
        val = self.amplitude * float(np.sin(self.frequency * t + self.phase))
        return val * self.direction
