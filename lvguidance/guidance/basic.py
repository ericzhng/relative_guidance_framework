"""Basic guidance laws: velocity pursuit and proportional-derivative feedback."""

from __future__ import annotations

from lvguidance.guidance.base import GuidanceLaw, GuidanceState, Vec


class VelocityPursuit(GuidanceLaw):
    """Line-of-sight velocity pursuit with a range-scheduled closing speed.

    The desired relative velocity points along the line of sight with magnitude
    ``min(k_range * range, v_max)``; a first-order velocity loop tracks it.

    Args:
        k_range: Range-to-speed gain [1/s].
        v_max: Cap on the closing speed [m/s].
        k_vel: Velocity-loop gain [1/s].
    """

    name = "Velocity pursuit"

    def __init__(self, k_range: float = 0.03, v_max: float = 80.0, k_vel: float = 0.15) -> None:
        self.k_range, self.v_max, self.k_vel = k_range, v_max, k_vel

    def command(self, state: GuidanceState) -> Vec:
        """Return the acceleration that tracks the scheduled closing velocity."""
        v_close = min(self.k_range * state.range, self.v_max)
        v_des = state.los * v_close  # relative velocity xdot should equal -v_des
        return self.k_vel * (state.rel_vel + v_des)


class ProportionalDerivative(GuidanceLaw):
    """Spring-damper (PD) feedback on relative position and velocity.

    The gains follow from a natural frequency and damping ratio of ``xddot = -u``,
    giving ``u = wn^2 * x + 2 * zeta * wn * xdot``.

    Args:
        wn: Natural frequency [rad/s].
        zeta: Damping ratio [-].
    """

    name = "PD feedback"

    def __init__(self, wn: float = 0.012, zeta: float = 1.0) -> None:
        self.wn, self.zeta = wn, zeta

    def command(self, state: GuidanceState) -> Vec:
        """Return the PD acceleration command."""
        return self.wn**2 * state.rel_pos + 2.0 * self.zeta * self.wn * state.rel_vel
