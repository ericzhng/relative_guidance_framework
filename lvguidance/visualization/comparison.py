"""Multi-law comparison figures."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from lvguidance.simulation.metrics import summarize
from lvguidance.simulation.result import SimulationResult
from lvguidance.visualization.style import TARGET_COLOR, apply_style, color_for, save_figure


def plot_comparison(
    results: Sequence[SimulationResult], save_path: str | Path | None = None, show: bool = False
) -> Figure:
    """Overlay the time histories of several guidance laws.

    Args:
        results: Runs of the same scenario, one per law.
        save_path: Optional image path.
        show: Whether to display the figure interactively.

    Returns:
        The created figure with six panels: LVLH path, range, closing speed versus range,
        thrust acceleration, cumulative delta-v and relative speed.
    """
    apply_style()
    fig, axes = plt.subplots(2, 3, figsize=(17, 9), constrained_layout=True)
    ax_path, ax_rng, ax_phase, ax_acc, ax_dv, ax_spd = axes.ravel()

    for i, res in enumerate(results):
        c = color_for(i)
        p = res.chaser_lvlh / 1e3
        ax_path.plot(p[:, 1], p[:, 0], color=c, label=res.law_name)
        ax_rng.semilogy(res.t, np.maximum(res.range, 1e-1), color=c)
        ax_phase.plot(res.range / 1e3, -res.range_rate, color=c)
        ax_acc.plot(res.t, res.accel_norm, color=c)
        ax_dv.plot(res.t, res.delta_v, color=c)
        ax_spd.semilogy(res.t, np.maximum(res.rel_speed, 1e-2), color=c)

    ax_path.scatter(0, 0, color=TARGET_COLOR, marker="*", s=130, zorder=5, label="Target")
    ax_path.set(xlabel="Along-track [km]", ylabel="Radial [km]", title="Relative path (LVLH)")
    ax_path.legend()
    ax_rng.set(xlabel="Time [s]", ylabel="Range [m]", title="Distance to target")
    ax_phase.set(xlabel="Range [km]", ylabel="Closing speed [m/s]", title="Approach profile")
    ax_acc.axhline(results[0].max_accel, color="gray", ls="--", lw=1)
    ax_acc.set(xlabel="Time [s]", ylabel="|a| [m/s²]", title="Thrust acceleration (dashed: limit)")
    ax_dv.set(xlabel="Time [s]", ylabel="Δv [m/s]", title="Cumulative fuel use")
    ax_spd.set(xlabel="Time [s]", ylabel="Relative speed [m/s]", title="Relative speed")
    fig.suptitle("Guidance-law comparison", fontsize=14, fontweight="bold")
    save_figure(fig, save_path)
    if show:
        plt.show()
    return fig


def plot_metric_bars(
    results: Sequence[SimulationResult],
    capture_radius: float = 2.0,
    save_path: str | Path | None = None,
    show: bool = False,
) -> Figure:
    """Bar charts of arrival time, delta-v, final relative speed and peak acceleration.

    Args:
        results: Runs to compare.
        capture_radius: Range regarded as successful arrival [m].
        save_path: Optional image path.
        show: Whether to display the figure interactively.

    Returns:
        The created figure.
    """
    apply_style()
    rows = [summarize(r, capture_radius) for r in results]
    keys = [
        ("time_s", "Arrival time [s]"),
        ("delta_v_mps", "Delta-v [m/s]"),
        ("final_rel_speed_mps", "Final relative speed [m/s]"),
        ("peak_accel_mps2", "Peak acceleration [m/s²]"),
    ]
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.6), constrained_layout=True)
    labels = [r["law"] for r in rows]
    colors = [color_for(i) for i in range(len(rows))]
    for ax, (key, title) in zip(axes, keys, strict=True):
        vals = [float(r[key]) for r in rows]
        bars = ax.barh(labels, vals, color=colors)
        ax.bar_label(bars, fmt="%.2f" if max(vals) < 10 else "%.0f", padding=3, fontsize=8)
        ax.set_title(title)
        ax.invert_yaxis()
        ax.grid(axis="y", visible=False)
        if ax is not axes[0]:
            ax.set_yticklabels([])
    save_figure(fig, save_path)
    if show:
        plt.show()
    return fig
