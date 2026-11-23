"""Optimal guidance: ZEM/ZEV laws with fixed, adaptive and prescribed terminal velocity."""

from __future__ import annotations

import numpy as np

from lvguidance.guidance.base import GuidanceLaw, GuidanceState, Vec


def _energy_optimal(x: Vec, xdot: Vec, tau: float, v_final: Vec | None = None) -> Vec:
    """Energy-optimal command for arriving at ``x = 0`` with ``xdot = v_final`` in ``tau``.

    Minimises the integral of ``|u|^2`` for a double integrator, written with the
    zero-effort miss ``ZEM = x + xdot * tau`` and zero-effort velocity error
    ``ZEV = xdot - v_final``::

        u = 6 * ZEM / tau^2 - 2 * ZEV / tau
    """
    vf = np.zeros(3) if v_final is None else v_final
    zem = x + xdot * tau
    zev = xdot - vf
    return 6.0 * zem / tau**2 - 2.0 * zev / tau


class ZemZevGuidance(GuidanceLaw):
    """Fixed-time energy-optimal ZEM/ZEV guidance (zero terminal relative velocity).

    Args:
        t_go_min: Floor on time-to-go to avoid the terminal singularity [s].
    """

    name = "ZEM/ZEV (fixed time)"

    def __init__(self, t_go_min: float = 5.0) -> None:
        self.t_go_min = t_go_min

    def command(self, state: GuidanceState) -> Vec:
        """Return the optimal command for the remaining time-to-go."""
        tau = max(state.t_go, self.t_go_min)
        return _energy_optimal(state.rel_pos, state.rel_vel, tau)


class AdaptiveTgoGuidance(GuidanceLaw):
    """ZEM/ZEV guidance with a time-to-go estimated from range and closing speed.

    Removes the need for a planned arrival time. While closing, the vehicle will stop
    after ``2 * range / closing`` seconds if it decelerates uniformly; this estimate is
    capped by the time ``2 * sqrt(range / a_nom)`` of a nominal-acceleration
    accelerate-then-brake manoeuvre, which also covers the not-yet-closing case.

    Args:
        a_nom: Nominal acceleration of the reference manoeuvre [m/s^2].
        t_go_min: Floor on the time-to-go estimate [s].
    """

    name = "ZEM/ZEV (adaptive t_go)"

    def __init__(self, a_nom: float = 0.2, t_go_min: float = 10.0) -> None:
        self.a_nom, self.t_go_min = a_nom, t_go_min

    def command(self, state: GuidanceState) -> Vec:
        """Return the optimal command using the range-based time-to-go."""
        cap = 2.0 * float(np.sqrt(state.range / self.a_nom))
        stop_time = 2.0 * state.range / max(-state.range_rate, 1e-3)
        tau = max(min(stop_time, cap), self.t_go_min)
        return _energy_optimal(state.rel_pos, state.rel_vel, tau)


class TerminalVelocityGuidance(GuidanceLaw):
    """Energy-optimal guidance that arrives with a prescribed docking speed along the LOS.

    The terminal relative velocity is ``-v_dock`` along the initial line of sight, which
    gives a controlled, non-zero closing speed at contact.

    Args:
        v_dock: Desired closing speed at arrival [m/s].
        t_go_min: Floor on time-to-go [s].
    """

    name = "Optimal (docking speed)"

    def __init__(self, v_dock: float = 0.5, t_go_min: float = 5.0) -> None:
        self.v_dock, self.t_go_min = v_dock, t_go_min
        self._axis: Vec | None = None

    def reset(self) -> None:
        """Forget the stored approach axis."""
        self._axis = None

    def command(self, state: GuidanceState) -> Vec:
        """Return the optimal command that meets the terminal-velocity condition."""
        if self._axis is None:
            self._axis = state.los.copy()
        tau = max(state.t_go, self.t_go_min)
        v_final = -self.v_dock * self._axis
        return _energy_optimal(state.rel_pos, state.rel_vel, tau, v_final)
