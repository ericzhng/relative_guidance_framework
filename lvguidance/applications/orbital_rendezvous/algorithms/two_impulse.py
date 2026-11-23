"""Two-impulse Clohessy-Wiltshire (CW) analytical targeting transfer."""

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
        if state.lvlh_state is not None:
            return state.lvlh_state
        return np.concatenate((state.rel_pos, state.rel_vel))
    return np.asarray(state, dtype=float)


class TwoImpulseCW(GuidanceLaw):
    """Two-impulse transfer using the closed-form Clohessy-Wiltshire solution.

    Burn 1 (t=0) applies the velocity change that reaches the goal position at t=tf;
    Burn 2 (t=tf) cancels residual relative velocity. Both burns are evaluated
    in closed loop on the measured state, so the arrival burn absorbs linearization error.
    """

    name = "Two-impulse CW"

    def __init__(self) -> None:
        self._n = 0.0
        self._tf = 0.0
        self._goal = np.zeros(6)

    def reset(self, scenario: Any = None) -> None:
        """Store target mean motion, duration and goal from scenario."""
        if scenario is not None:
            self._n = getattr(scenario, "mean_motion", 0.0011)
            self._tf = getattr(scenario, "duration", getattr(scenario, "t_final", 3600.0))
            self._goal = getattr(scenario, "goal", np.zeros(6))

    def burn_times(self) -> Sequence[float]:
        """Departure at t=0 and arrival at t=tf."""
        return (0.0, self._tf)

    def burn(self, t: float, state: GuidanceState | Vec) -> Vec:
        """Compute departure or arrival impulsive velocity increment [m/s]."""
        x = _get_x(state)
        if t < 0.5 * self._tf:
            v_req = cw_required_velocity(self._n, self._tf - t, x[:3], self._goal[:3])
            return v_req - x[3:]
        return self._goal[3:] - x[3:]

    def command(self, state: GuidanceState) -> Vec:
        """Continuous acceleration command (zero for impulsive transfers)."""
        return np.zeros(3)
