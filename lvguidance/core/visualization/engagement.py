"""Single-engagement comprehensive analysis dashboard."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from lvguidance.core.simulation.result import SimulationResult
from lvguidance.core.visualization.style import CHASER_COLOR, TARGET_COLOR, apply_style, save_figure


def plot_engagement(
    result: SimulationResult, save_path: str | Path | None = None, show: bool = False
) -> Figure:
    """Plot detailed 6-panel single engagement history."""
    apply_style()
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5), constrained_layout=True)
    ax_path, ax_rng, ax_phase, ax_acc, ax_los, ax_spd = axes.ravel()

    rel_pos_km = result.rel_pos / 1e3
    ax_path.plot(rel_pos_km[:, 1], rel_pos_km[:, 0], color=CHASER_COLOR, label="Chaser/Missile")
    ax_path.scatter(0, 0, color=TARGET_COLOR, marker="*", s=140, label="Target")
    ax_path.set(xlabel="Y [km]", ylabel="X [km]", title="Trajectory")
    ax_path.legend()

    ax_rng.plot(result.t, result.range / 1e3, color=CHASER_COLOR)
    ax_rng.set(xlabel="Time [s]", ylabel="Range [km]", title="Range to Target")

    ax_phase.plot(result.range / 1e3, result.closing_speed, color=CHASER_COLOR)
    ax_phase.set(xlabel="Range [km]", ylabel="Closing speed [m/s]", title="Closing Speed Profile")

    ax_acc.plot(result.t, result.accel_norm, color=CHASER_COLOR, label="Applied")
    ax_acc.axhline(result.max_accel, color="gray", ls="--", label="Limit")
    ax_acc.set(xlabel="Time [s]", ylabel="Acceleration [m/s²]", title="Acceleration Profile")
    ax_acc.legend()

    ax_los.plot(result.t, np.rad2deg(result.los_rate), color=CHASER_COLOR)
    ax_los.set(xlabel="Time [s]", ylabel="LOS Rate [deg/s]", title="Line-of-Sight Rate")

    ax_spd.plot(result.t, result.rel_speed, color=CHASER_COLOR)
    ax_spd.set(xlabel="Time [s]", ylabel="Relative Speed [m/s]", title="Relative Speed")

    # Mark handover events if present
    for event_name, event_time in result.events.items():
        for ax in (ax_rng, ax_acc, ax_los, ax_spd):
            ax.axvline(event_time, color="red", ls=":", label=event_name)

    fig.suptitle(f"Engagement: {result.law_name}", fontsize=13, fontweight="bold")
    save_figure(fig, save_path)
    if show:
        plt.show()
    return fig
