"""Abstract base interface for all guidance laws."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Any

import numpy as np
from numpy.typing import NDArray

from relguide.core.state import GuidanceState

Vec = NDArray[np.float64]


class GuidanceLaw(ABC):
    """Abstract base class for all homing and rendezvous guidance laws.

    Conventions:
        Relative state: ``x = r_target - r_chaser``, ``xdot = v_target - v_chaser``.
        Double-integrator control: ``xddot = -u + a_target + Delta_g``.
        Supports continuous acceleration commands, discrete impulsive burns (delta_v),
        or hybrid actuation.
    """

    name: str = "guidance"
    control_dt: float | None = None

    @abstractmethod
    def command(self, state: GuidanceState) -> Vec:
        """Compute the continuous acceleration command.

        Args:
            state: Current relative navigation state.

        Returns:
            Command acceleration vector ``u`` [m/s^2].
        """
        ...

    def reset(self, scenario: Any = None) -> None:  # noqa: B027
        """Clear internal memory before a new simulation run."""
        pass

    def burn_times(self) -> Sequence[float]:
        """Return scheduled mission times [s] for impulsive velocity burns."""
        return ()

    def burn(self, t: float, state: Any) -> Vec:
        """Compute the impulsive velocity change delta-v [m/s] at time t."""
        return np.zeros(3)
