"""Sliding-mode guidance with a boundary layer."""

from __future__ import annotations

import numpy as np

from lvguidance.guidance.base import GuidanceLaw, GuidanceState, Vec


class SlidingModeGuidance(GuidanceLaw):
    """Sliding-mode approach guidance with a speed-limited surface.

    The sliding surface is ``s = xdot + v_c * los`` with the scheduled closing speed
    ``v_c = min(sqrt(2 * a_brake * range), v_max)``: cruise at ``v_max``, then brake at
    the constant deceleration ``a_brake`` so that range and speed vanish together. The
    discontinuous switching term is smoothed by a saturation inside a boundary layer to
    avoid chattering::

        u = k * sat(s / phi)

    Args:
        a_brake: Braking deceleration of the surface [m/s^2]; keep below ``k``.
        v_max: Cap on the scheduled closing speed [m/s].
        k: Switching gain [m/s^2]; must dominate the surface acceleration.
        phi: Boundary-layer thickness [m/s].
    """

    name = "Sliding mode"

    def __init__(
        self, a_brake: float = 0.2, v_max: float = 80.0, k: float = 0.4, phi: float = 2.0
    ) -> None:
        self.a_brake, self.v_max, self.k, self.phi = a_brake, v_max, k, phi

    def command(self, state: GuidanceState) -> Vec:
        """Return the smoothed sliding-mode command."""
        v_close = min(float(np.sqrt(2.0 * self.a_brake * state.range)), self.v_max)
        s = state.rel_vel + v_close * state.los
        norm = float(np.linalg.norm(s))
        return self.k * (s / self.phi if norm < self.phi else s / norm)
