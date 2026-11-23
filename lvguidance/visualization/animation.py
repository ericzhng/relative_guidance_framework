"""Animation of an approach in the target LVLH frame with live history panels."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from lvguidance.simulation.result import SimulationResult
from lvguidance.visualization.style import CHASER_COLOR, TARGET_COLOR, apply_style


def animate_engagement(
    res: SimulationResult,
    save_path: str | Path,
    frames: int = 120,
    fps: int = 20,
) -> Path:
    """Render a GIF of the approach.

    Left: chaser trace in the target-centred LVLH plane (the target sits at the origin
    because the frame moves with it). Right: range and thrust acceleration histories with
    a moving cursor.

    Args:
        res: Simulation history.
        save_path: Output ``.gif`` path.
        frames: Number of animation frames.
        fps: Frames per second.

    Returns:
        Path of the written file.
    """
    apply_style()
    out = Path(save_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    p = res.chaser_lvlh / 1e3
    idx = np.unique(np.linspace(0, len(res.t) - 1, frames).astype(int))

    fig = plt.figure(figsize=(12, 5.5), constrained_layout=True)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.3, 1])
    ax = fig.add_subplot(gs[:, 0])
    ax_r = fig.add_subplot(gs[0, 1])
    ax_a = fig.add_subplot(gs[1, 1])

    pad = 0.05 * max(np.ptp(p[:, 0]), np.ptp(p[:, 1]), 1.0)
    ax.set(xlim=(p[:, 1].min() - pad, p[:, 1].max() + pad),
           ylim=(p[:, 0].min() - pad, p[:, 0].max() + pad),
           xlabel="Along-track [km]", ylabel="Radial [km]")
    ax.set_title(f"{res.law_name} (target-centred LVLH)")
    ax.scatter(0, 0, color=TARGET_COLOR, marker="*", s=160, zorder=5, label="Target")
    trace, = ax.plot([], [], color=CHASER_COLOR, label="Chaser trace")
    head, = ax.plot([], [], "o", color=CHASER_COLOR)
    los, = ax.plot([], [], color="gray", lw=0.8, ls=":")
    label = ax.text(0.02, 0.96, "", transform=ax.transAxes, va="top", fontsize=9)
    ax.legend(loc="lower right")

    ax_r.plot(res.t, res.range / 1e3, color="#bbbbbb")
    ax_r.set(ylabel="Range [km]", title="Distance")
    cur_r, = ax_r.plot([], [], color=CHASER_COLOR)
    ax_a.plot(res.t, res.accel_norm, color="#bbbbbb")
    ax_a.set(xlabel="Time [s]", ylabel="|a| [m/s²]", title="Thrust acceleration")
    cur_a, = ax_a.plot([], [], color=CHASER_COLOR)

    def update(k: int):
        i = idx[k]
        trace.set_data(p[: i + 1, 1], p[: i + 1, 0])
        head.set_data([p[i, 1]], [p[i, 0]])
        los.set_data([0, p[i, 1]], [0, p[i, 0]])
        cur_r.set_data(res.t[: i + 1], res.range[: i + 1] / 1e3)
        cur_a.set_data(res.t[: i + 1], res.accel_norm[: i + 1])
        label.set_text(
            f"t = {res.t[i]:.0f} s\nrange = {res.range[i]:.0f} m\n"
            f"closing = {-res.range_rate[i]:.1f} m/s"
        )
        return trace, head, los, cur_r, cur_a, label

    anim = FuncAnimation(fig, update, frames=len(idx), blit=False)
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out
