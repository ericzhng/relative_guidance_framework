"""Run and compare all orbital rendezvous guidance laws.

Usage::

    python examples/compare_rendezvous_laws.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lvguidance.applications.orbital_rendezvous import (
    OrbitalScenario,
    default_rendezvous_laws,
    simulate_rendezvous,
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
    print("=== Running Orbital Rendezvous Guidance Laws ===")
    scenario = OrbitalScenario()
    results = [simulate_rendezvous(law, scenario) for law in default_rendezvous_laws()]

    print(format_table(results, scenario.capture_radius))

    plot_comparison(results, OUT / "rendezvous_comparison.png")
    plot_metric_bars(results, scenario.capture_radius, OUT / "rendezvous_metrics.png")

    for res in results:
        slug = re.sub(r"[^a-z0-9]+", "_", res.law_name.lower()).strip("_")
        plot_engagement(res, OUT / f"engagement_{slug}.png")

    print(f"\nRendezvous figures saved to {OUT}")


if __name__ == "__main__":
    main()
