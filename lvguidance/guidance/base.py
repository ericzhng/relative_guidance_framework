"""Common interface for rendezvous guidance laws.

Conventions:
    The relative state is ``x = r_target - r_chaser`` and ``xdot = v_target - v_chaser``
    (inertial components). Ignoring gravity differences, the chaser's thrust acceleration
    ``a`` obeys ``xddot = -a``. Every law returns the *free-space* command ``u`` meant to
    satisfy ``xddot = -u``; the engine adds the differential-gravity feed-forward.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]


@dataclass(frozen=True)
class GuidanceState:
    """Information available to a guidance law at one guidance cycle.

    Attributes:
        t: Elapsed mission time [s].
        rel_pos: Target position relative to the chaser [m].
        rel_vel: Target velocity relative to the chaser [m/s].
        t_final: Planned arrival time on the mission clock [s].
    """

    t: float
    rel_pos: Vec
    rel_vel: Vec
    t_final: float

    @property
    def range(self) -> float:
        """Line-of-sight distance [m]."""
        return float(np.linalg.norm(self.rel_pos))

    @property
    def los(self) -> Vec:
        """Unit line-of-sight vector from chaser to target."""
        return self.rel_pos / max(self.range, 1e-9)

    @property
    def range_rate(self) -> float:
        """Range rate [m/s]; negative while closing."""
        return float(self.los @ self.rel_vel)

    @property
    def t_go(self) -> float:
        """Time remaining to the planned arrival [s]."""
        return self.t_final - self.t


class GuidanceLaw(ABC):
    """Abstract guidance law."""

    name: str = "guidance"

    @abstractmethod
    def command(self, state: GuidanceState) -> Vec:
        """Compute the free-space acceleration command.

        Args:
            state: Current relative navigation state.

        Returns:
            Command ``u`` [m/s^2] such that the desired relative dynamics is ``xddot = -u``.
        """

    def reset(self) -> None:  # noqa: B027 - optional hook
        """Clear internal memory before a new run."""
