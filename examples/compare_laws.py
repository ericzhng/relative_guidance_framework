"""Run every guidance law on the default scenario, print metrics and save all figures.

Usage (from the project root)::

    python examples/compare_laws.py [--gif]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lvguidance.guidance import default_laws  # noqa: E402
from lvguidance.simulation import Scenario, format_table, simulate  # noqa: E402
from lvguidance.visualization import (  # noqa: E402
    animate_engagement,
    plot_comparison,
    plot_engagement,
    plot_metric_bars,
    plot_terminal_phase,
)

OUT = Path(__file__).resolve().parents[1] / "outputs"


def main() -> None:
    """Simulate, tabulate and plot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gif", action="store_true", help="also render GIF animations (slow)")
    args = parser.parse_args()

    scenario = Scenario()
    results = [simulate(law, scenario) for law in default_laws()]
    print(format_table(results, scenario.capture_radius))

    plot_comparison(results, OUT / "comparison.png")
    plot_metric_bars(results, scenario.capture_radius, OUT / "metrics.png")
    plot_terminal_phase(results, window=1000.0, save_path=OUT / "terminal_phase.png")
    for res in results:
        slug = re.sub(r"[^a-z0-9]+", "_", res.law_name.lower()).strip("_")
        plot_engagement(res, OUT / f"engagement_{slug}.png")
        if args.gif:
            animate_engagement(res, OUT / f"engagement_{slug}.gif")
    print(f"\nFigures written to {OUT}")


if __name__ == "__main__":
    main()
