"""Sliding-mode guidance with boundary layer smoothing."""

from __future__ import annotations

import numpy as np

from lvguidance.core.base import GuidanceLaw, GuidanceState, Vec


class SlidingModeGuidance(GuidanceLaw):
    """Sliding-mode approach guidance with a speed-limited braking surface.

    The sliding surface enforces a scheduled closing speed::

        v_c = min(sqrt(2 * a_brake * range), v_max)
        s = rel_vel + v_c * los

    and commands smoothed switching::

        u = k * sat(s / phi)

    Args:
        a_brake: Braking deceleration profile [m/s^2].
        v_max: Scheduled speed cap [m/s].
        k: Switching gain [m/s^2].
        phi: Boundary layer thickness [m/s].
    """

    name = "Sliding mode"

    def __init__(
        self, a_brake: float = 0.2, v_max: float = 80.0, k: float = 0.4, phi: float = 2.0
    ) -> None:
        self.a_brake = a_brake
        self.v_max = v_max
        self.k = k
        self.phi = phi

    def command(self, state: GuidanceState) -> Vec:
        """Compute the smoothed sliding-mode acceleration."""
        v_close = min(float(np.sqrt(2.0 * self.a_brake * max(state.range, 0.0))), self.v_max)
        s = state.rel_vel + v_close * state.los
        norm = float(np.linalg.norm(s))
        return self.k * (s / self.phi if norm < self.phi else s / norm)


class SlidingModeHomingGuidance(GuidanceLaw):
    """Sliding-mode guidance for missile homing that nulls the LOS angular rate.

    Sliding surface::

        s = Omega x los

    with robust switching to enforce s -> 0.

    Args:
        k_gain: Switching gain [m/s^2].
        phi: Boundary layer width [rad/s].
        n_nav: Proportional gain multiplier.
    """

    name = "Sliding Mode Homing"

    def __init__(self, k_gain: float = 20.0, phi: float = 0.02, n_nav: float = 3.0) -> None:
        self.k_gain = k_gain
        self.phi = phi
        self.n_nav = n_nav

    def command(self, state: GuidanceState) -> Vec:
        """Compute the LOS-nulling sliding mode command."""
        v_c = max(state.closing_speed, 1.0)
        omega = state.los_rate_vector
        s = np.cross(omega, state.los)
        norm_s = float(np.linalg.norm(s))
        switch = s / self.phi if norm_s < self.phi else s / norm_s
        linear = self.n_nav * v_c * s
        return linear + self.k_gain * switch
