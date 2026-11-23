"""Scenario definition for a chaser approaching an orbiting target."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from lvguidance.dynamics.orbit import MU_EARTH, R_EARTH, circular_state, gravity
from lvguidance.guidance.base import Vec


@dataclass(frozen=True)
class Scenario:
    """Initial conditions and limits of one approach problem.

    Attributes:
        name: Scenario label.
        target_radius: Target circular-orbit radius [m].
        target_inclination: Target inclination [rad].
        rel_pos0: Initial target-minus-chaser position, inertial axes [m].
        rel_vel0: Initial target-minus-chaser velocity, inertial axes [m/s].
        t_final: Planned arrival time [s], used by fixed-time laws.
        t_max: Simulation time limit [s]; free-time laws may arrive after ``t_final``.
        max_accel: Thrust acceleration limit of the chaser [m/s^2].
        capture_radius: Range at which the approach is considered complete [m].
        dt: Guidance cycle [s].
        gravity_feedforward: Whether the chaser compensates the differential gravity.
        mu: Gravitational parameter [m^3/s^2].
    """

    name: str = "LEO approach"
    target_radius: float = R_EARTH + 400e3
    target_inclination: float = np.deg2rad(51.6)
    rel_pos0: Vec = field(default_factory=lambda: np.array([6000.0, -25000.0, 4000.0]))
    rel_vel0: Vec = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    t_final: float = 900.0
    t_max: float = 1500.0
    max_accel: float = 0.5
    capture_radius: float = 2.0
    dt: float = 1.0
    gravity_feedforward: bool = True
    mu: float = MU_EARTH

    def initial_states(self) -> tuple[Vec, Vec]:
        """Build inertial 6-states of the target and the chaser.

        Returns:
            Tuple ``(target_state, chaser_state)`` of ``[r, v]`` arrays.
        """
        r_t, v_t = circular_state(self.target_radius, self.target_inclination, mu=self.mu)
        target = np.concatenate((r_t, v_t))
        chaser = np.concatenate((r_t - self.rel_pos0, v_t - self.rel_vel0))
        return target, chaser

    def differential_gravity(self, r_target: Vec, r_chaser: Vec) -> Vec:
        """Gravity difference ``g(r_t) - g(r_c)`` used as feed-forward [m/s^2]."""
        return gravity(r_target, self.mu) - gravity(r_chaser, self.mu)
