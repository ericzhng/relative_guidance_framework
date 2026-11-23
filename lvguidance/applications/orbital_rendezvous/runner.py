"""Simulation runner for orbital rendezvous scenarios (supporting RK4 and solve_ivp)."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from lvguidance.applications.orbital_rendezvous.scenario import OrbitalScenario, gravity
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.models.autopilot import AutopilotModel
from lvguidance.core.simulation.engine import simulate_ivp, simulate_loop
from lvguidance.core.simulation.result import SimulationResult


def simulate_rendezvous(
    law: GuidanceLaw,
    scenario: OrbitalScenario | None = None,
    method: str = "auto",
) -> SimulationResult:
    """Run an orbital rendezvous simulation with either RK4 or SciPy solve_ivp (DOP853).

    Args:
        law: Guidance law or controller to evaluate.
        scenario: Orbital rendezvous scenario settings (defaults to OrbitalScenario()).
        method: Integrator selection ("auto", "rk4", or "solve_ivp"). "auto" uses solve_ivp
                if the law schedules impulsive burns, and RK4 otherwise.

    Returns:
        SimulationResult recording the flight.
    """
    sc = scenario or OrbitalScenario()

    use_ivp = (
        method in ("solve_ivp", "dop853")
        or (method == "auto" and len(list(law.burn_times())) > 0)
    )

    if use_ivp:
        return simulate_ivp(
            law=law,
            target_r0=sc.target_r0,
            target_v0=sc.target_v0,
            chaser_rel0=sc.chaser_rel0,
            duration=sc.duration,
            mu=sc.mu,
            sample_dt=sc.dt,
            scenario=sc,
        )

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
        scenario=sc,
    )


def run_comparison(
    scenario: OrbitalScenario,
    laws: Sequence[GuidanceLaw],
    method: str = "auto",
) -> list[SimulationResult]:
    """Run a suite of guidance algorithms against the same scenario."""
    return [simulate_rendezvous(law, scenario, method=method) for law in laws]
