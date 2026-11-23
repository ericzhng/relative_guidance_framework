"""Shared plotting style and color palettes."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.figure import Figure

TARGET_COLOR = "#D55E00"
CHASER_COLOR = "#0072B2"
LAW_COLORS = [
    "#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#000000", "#8C8C8C",
]
GRID_COLOR = "#d0d0d0"


def apply_style() -> None:
    """Apply package matplotlib visual defaults."""
    plt.rcParams.update(
        {
            "figure.dpi": 110,
            "savefig.dpi": 160,
            "axes.grid": True,
            "grid.color": GRID_COLOR,
            "grid.linewidth": 0.6,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.titlesize": 11,
            "axes.titleweight": "bold",
            "axes.labelsize": 9.5,
            "legend.frameon": False,
            "legend.fontsize": 8.5,
            "font.size": 9.5,
            "lines.linewidth": 1.8,
        }
    )


def color_for(index: int) -> str:
    """Return color for the given guidance law index."""
    return LAW_COLORS[index % len(LAW_COLORS)]


def save_figure(fig: Figure, path: str | Path | None) -> None:
    """Save figure if path is provided."""
    if path is None:
        return
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight")
