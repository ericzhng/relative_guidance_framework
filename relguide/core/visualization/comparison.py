"""Cross-law comparison visualizations."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from relguide.core.simulation.metrics import summarize
from relguide.core.simulation.result import SimulationResult
from relguide.core.visualization.style import TARGET_COLOR, apply_style, color_for, save_figure


def plot_comparison(
    results: Sequence[SimulationResult], save_path: str | Path | None = None, show: bool = False
) -> Figure:
    """Plot multi-panel comparison of guidance law histories."""
    apply_style()
    fig, axes = plt.subplots(2, 3, figsize=(16, 8.5), constrained_layout=True)
    ax_path, ax_rng, ax_phase, ax_acc, ax_dv, ax_spd = axes.ravel()

    for i, res in enumerate(results):
        c = color_for(i)
        # Relative trajectory in X-Y
        rel_pos_km = res.rel_pos / 1e3
        ax_path.plot(rel_pos_km[:, 1], rel_pos_km[:, 0], color=c, label=res.law_name)
        ax_rng.semilogy(res.t, np.maximum(res.range, 0.1), color=c)
        ax_phase.plot(res.range / 1e3, res.closing_speed, color=c)
        ax_acc.plot(res.t, res.accel_norm, color=c)
        ax_dv.plot(res.t, res.delta_v, color=c)
        ax_spd.plot(res.t, res.rel_speed, color=c)

    ax_path.scatter(0, 0, color=TARGET_COLOR, marker="*", s=140, zorder=5, label="Target")
    ax_path.set(
        xlabel="Cross-range / Y [km]", ylabel="Down-range / X [km]", title="Relative Trajectory"
    )
    ax_path.legend()
    ax_rng.set(xlabel="Time [s]", ylabel="Range [m]", title="Distance to Target")
    ax_phase.set(xlabel="Range [km]", ylabel="Closing speed [m/s]", title="Closing Speed Profile")
    ax_acc.axhline(results[0].max_accel, color="gray", ls="--", lw=1)
    ax_acc.set(xlabel="Time [s]", ylabel="|a| [m/s²]", title="Acceleration Demand")
    ax_dv.set(xlabel="Time [s]", ylabel="Δv [m/s]", title="Guidance Effort (Δv)")
    ax_spd.set(xlabel="Time [s]", ylabel="Relative speed [m/s]", title="Relative Speed")
    fig.suptitle("Guidance Law Comparison", fontsize=13, fontweight="bold")
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
    """Plot bar charts of key metrics."""
    apply_style()
    rows = [summarize(r, capture_radius) for r in results]
    keys = [
        ("time_s", "Intercept/Arrival Time [s]"),
        ("miss_distance_m", "Miss Distance [m]"),
        ("delta_v_mps", "Delta-v [m/s]"),
        ("peak_accel_mps2", "Peak Acceleration [m/s²]"),
    ]
    fig, axes = plt.subplots(1, 4, figsize=(16, 4.5), constrained_layout=True)
    labels = [str(r["law"]) for r in rows]
    colors = [color_for(i) for i in range(len(rows))]
    for ax, (key, title) in zip(axes, keys, strict=True):
        vals = [float(r[key]) for r in rows]
        bars = ax.barh(labels, vals, color=colors)
        ax.bar_label(bars, fmt="%.2f" if max(vals) < 10 else "%.1f", padding=3, fontsize=8)
        ax.set_title(title)
        ax.invert_yaxis()
        ax.grid(axis="y", visible=False)
        if ax is not axes[0]:
            ax.set_yticklabels([])
    save_figure(fig, save_path)
    if show:
        plt.show()
    return fig
