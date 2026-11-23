"""Simulation runner for orbital rendezvous scenarios."""

from __future__ import annotations

import numpy as np

from lvguidance.applications.orbital_rendezvous.dynamics import gravity
from lvguidance.applications.orbital_rendezvous.scenario import OrbitalScenario
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.models.autopilot import AutopilotModel
from lvguidance.core.simulation.engine import simulate_loop
from lvguidance.core.simulation.result import SimulationResult


def simulate_rendezvous(law: GuidanceLaw, scenario: OrbitalScenario | None = None) -> SimulationResult:
    """Run an orbital rendezvous simulation.

    Args:
        law: Guidance law to test.
        scenario: Orbital scenario settings (defaults to OrbitalScenario()).

    Returns:
        SimulationResult recording the flight.
    """
    sc = scenario or OrbitalScenario()
    target0, chaser0 = sc.initial_states()

    def target_deriv(state: np.ndarray, accel: np.ndarray) -> np.ndarray:
        return np.concatenate((state[3:], gravity(state[:3], sc.mu) + accel))

    def chaser_deriv(state: np.ndarray, accel: np.ndarray) -> np.ndarray:
        return np.concatenate((state[3:], gravity(state[:3], sc.mu) + accel))

    feedforward = sc.differential_gravity if sc.gravity_feedforward else None
    autopilot = AutopilotModel(tau_a=0.0, max_accel=sc.max_accel)

    return simulate_loop(
        law=law,
        target0=target0,
        chaser0=chaser0,
        target_deriv=target_deriv,
        chaser_deriv=chaser_deriv,
        feedforward_fn=feedforward,
        autopilot=autopilot,
        dt=sc.dt,
        t_max=sc.t_max,
        t_final=sc.t_final,
        capture_radius=sc.capture_radius,
        stop_on_opening=False,
    )
