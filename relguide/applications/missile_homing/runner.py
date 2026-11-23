"""Simulation runners and Monte Carlo analyzers for missile homing."""

from __future__ import annotations

from typing import Any

import numpy as np

from relguide.applications.missile_homing.scenario import MissileScenario
from relguide.core.base import GuidanceLaw
from relguide.core.simulation.engine import simulate_loop
from relguide.core.simulation.result import SimulationResult


def simulate_missile(law: GuidanceLaw, scenario: MissileScenario | None = None) -> SimulationResult:
    """Simulate a single missile homing engagement.

    Args:
        law: Guidance law under test (e.g., TPN, APN, OGL).
        scenario: Engagement scenario settings (defaults to MissileScenario()).

    Returns:
        SimulationResult containing the engagement trajectory.
    """
    sc = scenario or MissileScenario()
    target0, missile0 = sc.initial_states()

    def target_deriv(state: np.ndarray, accel: np.ndarray) -> np.ndarray:
        return np.concatenate((state[3:], accel))

    def missile_deriv(state: np.ndarray, accel: np.ndarray) -> np.ndarray:
        return np.concatenate((state[3:], accel))

    def target_accel(t: float, state: np.ndarray) -> np.ndarray:
        return sc.target_maneuver.acceleration(t, state)

    autopilot = sc.make_autopilot()

    return simulate_loop(
        law=law,
        target0=target0,
        chaser0=missile0,
        target_deriv=target_deriv,
        chaser_deriv=missile_deriv,
        target_accel_fn=target_accel,
        autopilot=autopilot,
        dt=sc.dt,
        t_max=sc.t_max,
        capture_radius=sc.capture_radius,
        stop_on_opening=True,
    )


def monte_carlo_missile(
    law: GuidanceLaw,
    base_scenario: MissileScenario,
    num_runs: int = 50,
    heading_error_std_deg: float = 5.0,
    seed: int = 42,
) -> dict[str, Any]:
    """Execute Monte Carlo dispersion analysis on a missile homing law.

    Args:
        law: Guidance law to evaluate.
        base_scenario: Baseline scenario configuration.
        num_runs: Number of dispersed runs.
        heading_error_std_deg: Standard deviation of initial heading error [deg].
        seed: Random seed for repeatability.

    Returns:
        Dictionary with statistical results (mean miss, max miss, CEP50, etc.).
    """
    rng = np.random.default_rng(seed)
    miss_distances = []

    for _ in range(num_runs):
        disp_heading = float(rng.normal(base_scenario.heading_error_deg, heading_error_std_deg))
        sc = MissileScenario(
            name=base_scenario.name,
            target_pos0=base_scenario.target_pos0.copy(),
            target_vel0=base_scenario.target_vel0.copy(),
            missile_pos0=base_scenario.missile_pos0.copy(),
            missile_speed0=base_scenario.missile_speed0,
            heading_error_deg=disp_heading,
            target_maneuver=base_scenario.target_maneuver,
            max_accel=base_scenario.max_accel,
            tau_a=base_scenario.tau_a,
            rate_limit=base_scenario.rate_limit,
            dt=base_scenario.dt,
            t_max=base_scenario.t_max,
            capture_radius=base_scenario.capture_radius,
        )
        res = simulate_missile(law, sc)
        miss_distances.append(res.miss_distance)

    misses = np.array(miss_distances)
    return {
        "law": law.name,
        "runs": num_runs,
        "mean_miss_m": float(np.mean(misses)),
        "median_miss_m": float(np.median(misses)),
        "cep_50_m": float(np.percentile(misses, 50)),
        "cep_95_m": float(np.percentile(misses, 95)),
        "max_miss_m": float(np.max(misses)),
        "all_misses": [float(m) for m in misses],
    }
