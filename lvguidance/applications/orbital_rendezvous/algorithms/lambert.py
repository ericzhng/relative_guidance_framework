"""Two-impulse rendezvous based on exact universal-variable Lambert targeting."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.astrodynamics.frames import lvlh_dcm, lvlh_to_eci
from lvguidance.core.astrodynamics.kepler import propagate_kepler, solve_lambert
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.state import GuidanceState

Vec = NDArray[np.float64]


def _get_x(state: GuidanceState | Vec) -> Vec:
    if isinstance(state, GuidanceState):
        if state.lvlh_state is not None:
            return state.lvlh_state
        return np.concatenate((state.rel_pos, state.rel_vel))
    return np.asarray(state, dtype=float)


class LambertTransfer(GuidanceLaw):
    """Two-impulse rendezvous solving the exact nonlinear Lambert boundary-value problem.

    Solves for the Keplerian transfer arc between current chaser inertial position and
    the target's future position at t=tf, eliminating small-separation CW assumptions.
    """

    name = "Lambert"

    def __init__(self) -> None:
        self._scenario: Any = None

    def reset(self, scenario: Any = None) -> None:
        """Store reference scenario with target orbit and duration."""
        self._scenario = scenario

    def burn_times(self) -> Sequence[float]:
        """Departure at t=0 and arrival at duration."""
        tf = getattr(self._scenario, "duration", getattr(self._scenario, "t_final", 3600.0))
        return (0.0, tf)

    def burn(self, t: float, state: GuidanceState | Vec) -> Vec:
        """Compute Lambert departure burn or velocity-matching arrival burn in LVLH."""
        scn = self._scenario
        tf = getattr(scn, "duration", getattr(scn, "t_final", 3600.0))
        goal = getattr(scn, "goal", np.zeros(6))
        mu = getattr(scn, "mu", 3.986004418e14)
        target_r0 = scn.target_r0
        target_v0 = scn.target_v0

        x = _get_x(state)
        if t >= 0.5 * tf:
            return goal[3:] - x[3:]

        r_t, v_t = propagate_kepler(target_r0, target_v0, t, mu=mu)
        r_c, v_c = lvlh_to_eci(r_t, v_t, x)
        r_tf, v_tf = propagate_kepler(target_r0, target_v0, tf, mu=mu)
        r_goal = r_tf + lvlh_dcm(r_tf, v_tf).T @ goal[:3]
        normal = np.cross(target_r0, target_v0)

        v1, _ = solve_lambert(r_c, r_goal, tf - t, mu=mu, normal=normal)
        return lvlh_dcm(r_t, v_t) @ (v1 - v_c)

    def command(self, state: GuidanceState) -> Vec:
        """Continuous acceleration command (zero for impulsive transfers)."""
        return np.zeros(3)
