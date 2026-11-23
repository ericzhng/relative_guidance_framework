"""Pursuit, proportional-derivative and glideslope corridor guidance laws."""

from __future__ import annotations

from typing import Any

import numpy as np

from lvguidance.core.base import GuidanceLaw, GuidanceState, Vec


class VelocityPursuit(GuidanceLaw):
    """Line-of-sight velocity pursuit with a range-scheduled closing speed.

    Desired relative velocity points along the LOS with speed::

        v_c = min(k_range * range, v_max)

    tracked by a first-order velocity loop: u = k_vel * (xdot + v_c * los).

    Args:
        k_range: Range-to-speed gain [1/s].
        v_max: Cap on closing speed [m/s].
        k_vel: Velocity loop tracking gain [1/s].
    """

    name = "Velocity pursuit"

    def __init__(self, k_range: float = 0.03, v_max: float = 80.0, k_vel: float = 0.15) -> None:
        self.k_range = k_range
        self.v_max = v_max
        self.k_vel = k_vel

    def command(self, state: GuidanceState) -> Vec:
        """Compute velocity pursuit command."""
        v_close = min(self.k_range * state.range, self.v_max)
        v_des = state.los * v_close
        return self.k_vel * (state.rel_vel + v_des)


class ProportionalDerivative(GuidanceLaw):
    """Spring-damper (PD) feedback on relative position and velocity.

    Command::

        u = wn^2 * x + 2 * zeta * wn * xdot

    Args:
        wn: Natural frequency [rad/s].
        zeta: Damping ratio [-].
    """

    name = "PD feedback"

    def __init__(self, wn: float = 0.012, zeta: float = 1.0) -> None:
        self.wn = wn
        self.zeta = zeta

    def command(self, state: GuidanceState) -> Vec:
        """Compute PD command."""
        return self.wn**2 * state.rel_pos + 2.0 * self.zeta * self.wn * state.rel_vel


class TerminalApproachGuidance(GuidanceLaw):
    """Glideslope final-approach guidance along an approach corridor axis.

    The approach axis is initialized to the LOS at handover. Longitudinal range
    governs closing speed down to `v_dock`, while lateral error off the axis
    is actively nulled.

    Args:
        k_glide: Glideslope closing speed per meter [1/s].
        v_dock: Docking contact speed floor [m/s].
        v_max: Speed cap [m/s].
        k_lat: Lateral error reduction gain [1/s].
        k_vel: Velocity loop gain [1/s].
    """

    name = "Terminal approach"

    def __init__(
        self,
        k_glide: float = 0.015,
        v_dock: float = 0.3,
        v_max: float = 25.0,
        k_lat: float = 0.03,
        k_vel: float = 0.2,
    ) -> None:
        self.k_glide = k_glide
        self.v_dock = v_dock
        self.v_max = v_max
        self.k_lat = k_lat
        self.k_vel = k_vel
        self._axis: Vec | None = None

    def reset(self, scenario: Any = None) -> None:
        """Reset corridor axis."""
        self._axis = None

    def command(self, state: GuidanceState) -> Vec:
        """Compute corridor glideslope command."""
        if self._axis is None:
            self._axis = state.los.copy()
        axis = self._axis
        longitudinal = float(state.rel_pos @ axis)
        lateral = state.rel_pos - longitudinal * axis
        v_close = float(np.clip(self.k_glide * longitudinal, self.v_dock, self.v_max))
        v_des = -v_close * axis - self.k_lat * lateral
        return self.k_vel * (state.rel_vel - v_des)
