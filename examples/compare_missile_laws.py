"""Run and compare all missile homing guidance laws against maneuvering targets.

Usage::

    python examples/compare_missile_laws.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lvguidance.applications.missile_homing import (
    MissileScenario,
    WeavingTarget,
    default_missile_laws,
    monte_carlo_missile,
    simulate_missile,
)
from lvguidance.core.simulation.metrics import format_table
from lvguidance.core.visualization import (
    plot_comparison,
    plot_engagement,
    plot_metric_bars,
)

OUT = Path(__file__).resolve().parents[1] / "outputs" / "missile"


def main() -> None:
    """Run missile homing simulation suite."""
    print("=== Running Missile Homing Guidance Laws (vs 4g Weaving Target) ===")
    target = WeavingTarget(amplitude=4.0 * 9.81, frequency=0.4)
    scenario = MissileScenario(
        name="High-G Weaving Intercept",
        target_maneuver=target,
        heading_error_deg=6.0,
        max_accel=300.0,
    )
    laws = default_missile_laws()
    results = [simulate_missile(law, scenario) for law in laws]

    print(format_table(results, scenario.capture_radius))

    plot_comparison(results, OUT / "missile_comparison.png")
    plot_metric_bars(results, scenario.capture_radius, OUT / "missile_metrics.png")

    for res in results:
        slug = re.sub(r"[^a-z0-9]+", "_", res.law_name.lower()).strip("_")
        plot_engagement(res, OUT / f"engagement_{slug}.png")

    print("\n--- Monte Carlo Dispersion (50 runs with heading error noise) ---")
    for law in laws[:3]:
        mc = monte_carlo_missile(law, scenario, num_runs=20, heading_error_std_deg=3.0)
        mean_m = mc['mean_miss_m']
        cep_m = mc['cep_50_m']
        print(f"{law.name:<32}: Mean Miss = {mean_m:.2f} m, CEP50 = {cep_m:.2f} m")

    print(f"\nMissile figures saved to {OUT}")


if __name__ == "__main__":
    main()
