"""Multi-impulse straight-line glideslope corridor approach."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.astrodynamics.clohessy_wiltshire import cw_required_velocity
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.state import GuidanceState

Vec = NDArray[np.float64]


def _get_x(state: GuidanceState | Vec) -> Vec:
    if isinstance(state, GuidanceState):
        return state.lvlh_state if state.lvlh_state is not None else np.concatenate((state.rel_pos, state.rel_vel))
    return np.asarray(state, dtype=float)


class ImpulsiveGlideslope(GuidanceLaw):
    """Multi-impulse approach through equally spaced waypoints on a straight corridor.

    At each of the ``n_legs`` nodes, the controller re-targets the next waypoint
    from the measured state using CW targeting, and the last burn nulls residual velocity.

    Args:
        n_legs: Number of coasting legs between burns (>= 1).
    """

    def __init__(self, n_legs: int = 6) -> None:
        if n_legs < 1:
            raise ValueError("n_legs must be >= 1")
        self.n_legs = n_legs
        self.name = f"Glideslope ({n_legs} legs)"
        self._n = 0.0
        self._tf = 0.0
        self._goal = np.zeros(6)
        self._start = np.zeros(3)

    def reset(self, scenario: Any = None) -> None:
        """Store scenario parameters and initial corridor start point."""
        if scenario is not None:
            self._n = getattr(scenario, "mean_motion", 0.0011)
            self._tf = getattr(scenario, "duration", getattr(scenario, "t_final", 3600.0))
            self._goal = getattr(scenario, "goal", np.zeros(6))
            rel0 = getattr(scenario, "chaser_rel0", getattr(scenario, "rel_pos0", np.zeros(6)))
            self._start = np.asarray(rel0[:3], dtype=float)

    def burn_times(self) -> Sequence[float]:
        """Evenly spaced burn nodes across the manoeuvre duration."""
        return tuple(np.linspace(0.0, self._tf, self.n_legs + 1))

    def burn(self, t: float, state: GuidanceState | Vec) -> Vec:
        """Re-target the next waypoint, or null velocity at the final node."""
        x = _get_x(state)
        leg = self._tf / self.n_legs
        k = int(round(t / leg))
        if k >= self.n_legs:
            return self._goal[3:] - x[3:]
        frac = (k + 1) / self.n_legs
        waypoint = self._start + frac * (self._goal[:3] - self._start)
        v_req = cw_required_velocity(self._n, leg, x[:3], waypoint)
        return v_req - x[3:]
