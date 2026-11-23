"""Abstract base interface for all guidance laws."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.state import GuidanceState

Vec = NDArray[np.float64]


class GuidanceLaw(ABC):
    """Abstract base class for all homing and rendezvous guidance laws.

    Conventions:
        Relative state: ``x = r_target - r_chaser``, ``xdot = v_target - v_chaser``.
        Double-integrator control: ``xddot = -u + a_target + Delta_g``.
        A positive command ``u`` along the LOS accelerates the chaser toward the target.
    """

    name: str = "guidance"

    @abstractmethod
    def command(self, state: GuidanceState) -> Vec:
        """Compute the acceleration command.

        Args:
            state: Current relative navigation state.

        Returns:
            Command acceleration vector ``u`` [m/s^2].
        """

    def reset(self) -> None:
        """Clear internal memory before a new simulation run."""
        pass
