"""Terminal (final approach) guidance and midcourse-to-terminal handover."""

from __future__ import annotations

import numpy as np

from lvguidance.guidance.base import GuidanceLaw, GuidanceState, Vec


class TerminalApproachGuidance(GuidanceLaw):
    """Glideslope final-approach guidance along a straight approach axis.

    The approach axis is the line of sight at the moment the law first runs (the
    handover). Relative position is split into a longitudinal part ``l`` along the axis
    and a lateral error ``x_perp`` off it. The desired relative velocity is::

        xdot_des = -v_c(l) * axis - k_lat * x_perp,   v_c = clip(k_glide * l, v_dock, v_max)

    so the closing speed shrinks linearly with distance down to the docking speed
    ``v_dock``, while the lateral term pulls the vehicle back onto the corridor. A
    first-order velocity loop tracks the desired velocity.

    Args:
        k_glide: Glideslope gain, closing speed per metre of distance [1/s].
        v_dock: Closing speed kept near contact [m/s].
        v_max: Cap on the closing speed [m/s].
        k_lat: Lateral position-to-velocity gain [1/s].
        k_vel: Velocity-loop gain [1/s].
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
        self.k_glide, self.v_dock, self.v_max = k_glide, v_dock, v_max
        self.k_lat, self.k_vel = k_lat, k_vel
        self._axis: Vec | None = None

    def reset(self) -> None:
        """Forget the stored approach axis."""
        self._axis = None

    def command(self, state: GuidanceState) -> Vec:
        """Return the acceleration that tracks the glideslope velocity."""
        if self._axis is None:
            self._axis = state.los.copy()
        axis = self._axis
        longitudinal = float(state.rel_pos @ axis)
        lateral = state.rel_pos - longitudinal * axis
        v_close = float(np.clip(self.k_glide * longitudinal, self.v_dock, self.v_max))
        v_des = -v_close * axis - self.k_lat * lateral
        return self.k_vel * (state.rel_vel - v_des)


class PhasedGuidance(GuidanceLaw):
    """Midcourse law that hands over to a terminal law inside a switch range.

    The handover is one-way: once the range falls below ``switch_range`` the terminal law
    stays in charge. The handover time is exposed as ``switch_time`` for plotting.

    Args:
        midcourse: Law used while far from the target.
        terminal: Law used for the final approach.
        switch_range: Range at which control passes to the terminal law [m].
    """

    def __init__(
        self, midcourse: GuidanceLaw, terminal: GuidanceLaw, switch_range: float = 1000.0
    ) -> None:
        self.midcourse, self.terminal, self.switch_range = midcourse, terminal, switch_range
        self.name = f"{midcourse.name} + terminal"
        self.switch_time: float | None = None

    def reset(self) -> None:
        """Return to the midcourse phase and reset both laws."""
        self.switch_time = None
        self.midcourse.reset()
        self.terminal.reset()

    def command(self, state: GuidanceState) -> Vec:
        """Delegate to the law of the current phase."""
        if self.switch_time is None and state.range <= self.switch_range:
            self.switch_time = state.t
        active = self.midcourse if self.switch_time is None else self.terminal
        return active.command(state)
