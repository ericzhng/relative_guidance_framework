"""Zoomed view of the terminal (final approach) phase."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from lvguidance.simulation.result import SimulationResult
from lvguidance.visualization.style import TARGET_COLOR, apply_style, color_for, save_figure


def _lateral_offset(res: SimulationResult, mask: np.ndarray) -> np.ndarray:
    """Distance of the chaser from the straight line through the target along the approach axis.

    The axis is the line of sight at the first sample inside the window.
    """
    axis = res.rel_pos[mask][0]
    axis = axis / np.linalg.norm(axis)
    pos = res.rel_pos[mask]
    return np.linalg.norm(pos - np.outer(pos @ axis, axis), axis=1)


def plot_terminal_phase(
    results: Sequence[SimulationResult],
    window: float = 1000.0,
    save_path: str | Path | None = None,
    show: bool = False,
) -> Figure:
    """Compare the last ``window`` metres of several runs.

    Panels: in-plane LVLH path, closing speed against range (log axis), lateral offset from
    the approach axis, and thrust acceleration against range.

    Args:
        results: Runs to compare.
        window: Range below which the approach is shown [m].
        save_path: Optional image path.
        show: Whether to display the figure interactively.

    Returns:
        The created figure.
    """
    apply_style()
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    ax_path, ax_speed, ax_lat, ax_acc = axes.ravel()

    for i, res in enumerate(results):
        mask = res.range <= window
        if mask.sum() < 2:
            continue
        c = color_for(i)
        rng = res.range[mask]
        p = res.chaser_lvlh[mask]
        ax_path.plot(p[:, 1], p[:, 0], color=c, label=res.law_name)
        closing = -res.range_rate[mask]
        ax_speed.loglog(rng, np.where(closing > 0, closing, np.nan), color=c)
        ax_lat.semilogx(rng, _lateral_offset(res, mask), color=c)
        ax_acc.semilogx(rng, res.accel_norm[mask], color=c)

    ax_path.scatter(0, 0, color=TARGET_COLOR, marker="*", s=130, zorder=5, label="Target")
    ax_path.set(xlabel="Along-track [m]", ylabel="Radial [m]", title=f"Last {window:.0f} m (LVLH)")
    ax_path.legend()
    ax_speed.set(xlabel="Range [m]", ylabel="Closing speed [m/s]", title="Closing-speed profile")
    ax_lat.set(xlabel="Range [m]", ylabel="Lateral offset [m]", title="Offset from approach axis")
    ax_acc.set(xlabel="Range [m]", ylabel="|a| [m/s²]", title="Thrust acceleration")
    for ax in (ax_speed, ax_lat, ax_acc):
        ax.invert_xaxis()
    fig.suptitle("Terminal approach phase", fontsize=14, fontweight="bold")
    save_figure(fig, save_path)
    if show:
        plt.show()
    return fig
