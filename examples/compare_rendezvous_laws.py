"""Run and compare all orbital rendezvous guidance and control algorithms.

Covers both continuous guidance laws (ZEM/ZEV, Sliding Mode, Phased, Pursuit)
and impulsive / optimal control algorithms (Two-Impulse CW, Lambert, Glideslope, LQR, MPC).

Usage::

    python examples/compare_rendezvous_laws.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lvguidance.applications.orbital_rendezvous import (
    ImpulsiveGlideslope,
    LQRController,
    LambertTransfer,
    MPCController,
    OrbitalScenario,
    TwoImpulseCW,
    animate_rendezvous,
    default_rendezvous_laws,
    plot_earth_orbit_view,
    plot_los_sphere,
    plot_metrics_comparison,
    plot_range_history,
    plot_spherical_history,
    plot_thrust_profile,
    plot_trajectories_2d,
    plot_trajectories_3d,
    run_comparison,
    simulate_rendezvous,
)
from lvguidance.applications.orbital_rendezvous.metrics import (
    format_rendezvous_table,
)
from lvguidance.core.simulation.metrics import format_table
from lvguidance.core.visualization import (
    plot_comparison,
    plot_engagement,
    plot_metric_bars,
)

OUT = Path(__file__).resolve().parents[1] / "outputs" / "rendezvous"


def main() -> None:
    """Run rendezvous simulation suite."""
    OUT.mkdir(parents=True, exist_ok=True)

    # ---------------------------------------------------------
    # Part 1: Continuous homing-style rendezvous guidance laws
    # ---------------------------------------------------------
    print("=== 1. Continuous Orbital Guidance Laws (RK4 Simulation) ===")
    scenario_rk4 = OrbitalScenario()
    results_rk4 = [simulate_rendezvous(law, scenario_rk4, method="rk4") for law in default_rendezvous_laws()]

    print(format_table(results_rk4, scenario_rk4.capture_radius))

    plot_comparison(results_rk4, OUT / "rendezvous_comparison.png")
    plot_metric_bars(results_rk4, scenario_rk4.capture_radius, OUT / "rendezvous_metrics.png")

    for res in results_rk4:
        slug = re.sub(r"[^a-z0-9]+", "_", res.law_name.lower()).strip("_")
        plot_engagement(res, OUT / f"engagement_{slug}.png")

    import matplotlib.pyplot as plt
    plt.close("all")

    # ---------------------------------------------------------
    # Part 2: Impulsive, LQR, and MPC algorithms (solve_ivp DOP853)
    # ---------------------------------------------------------
    print("\n=== 2. Impulsive & Optimal Control Algorithms (SciPy solve_ivp DOP853) ===")
    scenario_ivp = OrbitalScenario.circular(
        altitude=500e3,
        chaser_rel0=np.array([200.0, -5000.0, 100.0, 0.0, 0.0, 0.0]),
        duration=3600.0,
    )
    algorithms = [
        TwoImpulseCW(),
        LambertTransfer(),
        ImpulsiveGlideslope(n_legs=6),
        LQRController(label="LQR (gentle)", pos_tol=1000.0, vel_tol=1.0, a_max=5e-3),
        LQRController(label="LQR (aggressive)", pos_tol=100.0, vel_tol=0.5, a_max=2e-3),
        MPCController(label="MPC (aggressive)", pos_tol=100.0, vel_tol=0.5, a_max=2e-3),
    ]

    entries = run_comparison(scenario_ivp, algorithms, method="solve_ivp")
    print("\n" + format_rendezvous_table(entries))

    results_ivp = entries
    plot_trajectories_2d(results_ivp).savefig(OUT / "trajectories_2d.png")
    plot_trajectories_3d(results_ivp).savefig(OUT / "trajectories_3d.png")
    plot_range_history(results_ivp).savefig(OUT / "range_history.png")
    plot_thrust_profile(results_ivp).savefig(OUT / "thrust_profile.png")
    plot_metrics_comparison(entries).savefig(OUT / "metrics_comparison.png")
    plot_spherical_history(results_ivp).savefig(OUT / "spherical_history.png")
    plot_earth_orbit_view(results_ivp[:3]).savefig(OUT / "earth_orbit_view.png")

    for res in results_ivp[:3]:
        slug = re.sub(r"[^a-z0-9]+", "_", res.law_name.lower()).strip("_")
        plot_los_sphere(res).savefig(OUT / f"los_sphere_{slug}.png")

    animate_rendezvous(results_ivp, OUT / "rendezvous.gif")
    plt.close("all")
    print(f"\nAll orbital rendezvous figures & animations written to {OUT}")


if __name__ == "__main__":
    main()
