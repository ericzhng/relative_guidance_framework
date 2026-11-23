"""Scenario definition for orbital rendezvous and proximity operations."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from lvguidance.applications.orbital_rendezvous.dynamics import (
    MU_EARTH,
    R_EARTH,
    circular_state,
    gravity,
)
from lvguidance.core.base import Vec


@dataclass(frozen=True)
class OrbitalScenario:
    """Orbital rendezvous scenario parameters.

    Attributes:
        name: Label of the scenario.
        target_radius: Circular orbit radius [m].
        target_inclination: Orbit inclination [rad].
        rel_pos0: Initial relative position (target - chaser) [m].
        rel_vel0: Initial relative velocity (target - chaser) [m/s].
        t_final: Nominal arrival time for fixed-time laws [s].
        t_max: Simulation time limit [s].
        max_accel: Vehicle thrust acceleration limit [m/s^2].
        capture_radius: Docking completion capture radius [m].
        dt: Guidance step size [s].
        gravity_feedforward: Whether differential gravity is compensated.
        mu: Gravitational parameter [m^3/s^2].
    """

    name: str = "LEO Rendezvous"
    target_radius: float = R_EARTH + 400e3
    target_inclination: float = float(np.deg2rad(51.6))
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
        """Construct inertial 6-states [r, v] for target and chaser."""
        r_t, v_t = circular_state(self.target_radius, self.target_inclination, mu=self.mu)
        target = np.concatenate((r_t, v_t))
        chaser = np.concatenate((r_t - self.rel_pos0, v_t - self.rel_vel0))
        return target, chaser

    def differential_gravity(self, r_target: Vec, r_chaser: Vec) -> Vec:
        """Differential gravity feedforward g(r_t) - g(r_c) [m/s^2]."""
        return gravity(r_target, self.mu) - gravity(r_chaser, self.mu)
