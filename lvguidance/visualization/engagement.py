"""Single-run dashboard: motion of target and chaser plus their relative-motion histories."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from lvguidance.dynamics.orbit import R_EARTH
from lvguidance.simulation.result import SimulationResult
from lvguidance.visualization.style import (
    CHASER_COLOR,
    TARGET_COLOR,
    apply_style,
    save_figure,
)


def mark_handover(ax: Axes, res: SimulationResult) -> None:
    """Draw a vertical line at the midcourse-to-terminal handover time, if there was one."""
    if "handover" in res.events:
        ax.axvline(res.events["handover"], color="gray", ls=":", lw=1.2, label="Handover")


def plot_orbit_view(ax: Axes, res: SimulationResult) -> None:
    """Draw both traces at orbit scale in the inertial frame (3D axes required).

    At this scale the separation is a tiny fraction of the orbit, which is why the
    relative-motion panels exist; the view gives context only.
    """
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 40), np.linspace(0, np.pi, 20))
    re = R_EARTH / 1e3
    ax.plot_surface(
        re * np.cos(u) * np.sin(v),
        re * np.sin(u) * np.sin(v),
        re * np.cos(v),
        color="#cfd8e3",
        alpha=0.35,
        linewidth=0,
    )
    t_km, c_km = res.target[:, :3] / 1e3, res.chaser[:, :3] / 1e3
    ax.plot(*t_km.T, color=TARGET_COLOR, label="Target trace")
    ax.plot(*c_km.T, color=CHASER_COLOR, label="Chaser trace")
    ax.scatter(*t_km[-1], color=TARGET_COLOR, s=30, marker="*")
    ax.scatter(*c_km[-1], color=CHASER_COLOR, s=22)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("x [km]")
    ax.set_ylabel("y [km]")
    ax.set_zlabel("z [km]")
    ax.set_title("Orbit scale (inertial)")
    ax.legend(loc="upper left")


def plot_lvlh_3d(ax: Axes, res: SimulationResult) -> None:
    """Draw the chaser path in the target-centred LVLH frame (3D axes required)."""
    p = res.chaser_lvlh / 1e3
    ax.plot(p[:, 1], p[:, 0], p[:, 2], color=CHASER_COLOR, label="Chaser")
    ax.scatter(0, 0, 0, color=TARGET_COLOR, marker="*", s=90, label="Target (origin)")
    ax.scatter(p[0, 1], p[0, 0], p[0, 2], color=CHASER_COLOR, s=25, marker="s")
    ax.set_xlabel("Along-track [km]")
    ax.set_ylabel("Radial [km]")
    ax.set_zlabel("Cross-track [km]")
    ax.set_title("Relative path (LVLH, 3D)")
    ax.legend(loc="upper left")


def plot_lvlh_plane(ax: Axes, res: SimulationResult) -> None:
    """Draw the along-track / radial projection of the approach with time markers."""
    p = res.chaser_lvlh / 1e3
    ax.plot(p[:, 1], p[:, 0], color=CHASER_COLOR, label="Chaser")
    step = max(len(res.t) // 8, 1)
    ax.scatter(p[::step, 1], p[::step, 0], color=CHASER_COLOR, s=12, zorder=3)
    for i in range(0, len(res.t), step):
        ax.annotate(f"{res.t[i]:.0f}s", (p[i, 1], p[i, 0]), fontsize=7, xytext=(3, 3),
                    textcoords="offset points")
    ax.scatter(0, 0, color=TARGET_COLOR, marker="*", s=110, label="Target", zorder=4)
    ax.set_xlabel("Along-track [km]")
    ax.set_ylabel("Radial [km]")
    ax.set_title("Relative path (LVLH, in-plane)")
    ax.legend()


def plot_range_panel(ax: Axes, res: SimulationResult) -> None:
    """Plot range (left axis) and range rate (right axis) against time."""
    ax.plot(res.t, res.range / 1e3, color=CHASER_COLOR, label="Range")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Range [km]", color=CHASER_COLOR)
    ax2 = ax.twinx()
    ax2.plot(res.t, res.range_rate, color=TARGET_COLOR, label="Range rate")
    ax2.set_ylabel("Range rate [m/s]", color=TARGET_COLOR)
    ax2.grid(False)
    ax2.spines["right"].set_visible(True)
    mark_handover(ax, res)
    ax.set_title("Distance and closing rate")


def plot_los_angles(ax: Axes, res: SimulationResult) -> None:
    """Plot the LOS azimuth and elevation in the target LVLH frame."""
    ax.plot(res.t, np.degrees(res.los_azimuth), color=CHASER_COLOR, label="Azimuth")
    ax.plot(res.t, np.degrees(res.los_elevation), color=TARGET_COLOR, label="Elevation")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle [deg]")
    ax.set_title("Line-of-sight angles (LVLH)")
    ax.legend()


def plot_los_rate(ax: Axes, res: SimulationResult) -> None:
    """Plot the inertial LOS rate and the angle between closing velocity and LOS."""
    ax.plot(res.t, np.degrees(res.los_rate) * 1e3, color=CHASER_COLOR, label="LOS rate")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("LOS rate [mdeg/s]", color=CHASER_COLOR)
    ax2 = ax.twinx()
    ax2.plot(res.t, np.degrees(res.approach_angle), color=TARGET_COLOR, label="Approach angle")
    ax2.set_ylabel("Velocity-LOS angle [deg]", color=TARGET_COLOR)
    ax2.grid(False)
    ax2.spines["right"].set_visible(True)
    ax.set_title("LOS rate and approach angle")


def plot_acceleration(ax: Axes, res: SimulationResult) -> None:
    """Plot the applied thrust acceleration, its components and the limit."""
    ax.plot(res.t, res.accel_norm, color="black", label="|a|")
    for k, (lab, col) in enumerate(zip("xyz", ["#0072B2", "#E69F00", "#009E73"], strict=True)):
        ax.plot(res.t, res.accel[:, k], color=col, lw=1.0, alpha=0.8, label=f"a_{lab} (inertial)")
    ax.axhline(res.max_accel, color="gray", ls="--", lw=1, label="Limit")
    mark_handover(ax, res)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_title("Thrust acceleration")
    ax.set_ylim(top=1.4 * res.max_accel)
    ax.legend(ncol=3, loc="upper center")


def plot_speed_fuel(ax: Axes, res: SimulationResult) -> None:
    """Plot the relative speed (left axis) and cumulative delta-v (right axis)."""
    ax.plot(res.t, res.rel_speed, color=CHASER_COLOR)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Relative speed [m/s]", color=CHASER_COLOR)
    ax2 = ax.twinx()
    ax2.plot(res.t, res.delta_v, color=TARGET_COLOR)
    ax2.set_ylabel("Cumulative Δv [m/s]", color=TARGET_COLOR)
    ax2.grid(False)
    ax2.spines["right"].set_visible(True)
    mark_handover(ax, res)
    ax.set_title("Relative speed and fuel use")


def plot_phase(ax: Axes, res: SimulationResult) -> None:
    """Plot closing speed against range (approach corridor)."""
    ax.plot(res.range / 1e3, -res.range_rate, color=CHASER_COLOR)
    ax.set_xlabel("Range [km]")
    ax.set_ylabel("Closing speed [m/s]")
    ax.set_title("Approach profile")


def plot_engagement(
    res: SimulationResult, save_path: str | Path | None = None, show: bool = False
) -> Figure:
    """Build the nine-panel dashboard for a single run.

    Panels: orbit-scale traces, 3D LVLH path, in-plane LVLH path, range and range rate,
    LOS angles, LOS rate with approach angle, thrust acceleration, speed with delta-v,
    and the closing-speed versus range profile.

    Args:
        res: Simulation history.
        save_path: Optional image path.
        show: Whether to display the figure interactively.

    Returns:
        The created figure.
    """
    apply_style()
    fig = plt.figure(figsize=(17, 12.5), constrained_layout=True)
    gs = fig.add_gridspec(3, 3)
    plot_orbit_view(fig.add_subplot(gs[0, 0], projection="3d"), res)
    plot_lvlh_3d(fig.add_subplot(gs[0, 1], projection="3d"), res)
    plot_lvlh_plane(fig.add_subplot(gs[0, 2]), res)
    plot_range_panel(fig.add_subplot(gs[1, 0]), res)
    plot_los_angles(fig.add_subplot(gs[1, 1]), res)
    plot_los_rate(fig.add_subplot(gs[1, 2]), res)
    plot_acceleration(fig.add_subplot(gs[2, 0]), res)
    plot_speed_fuel(fig.add_subplot(gs[2, 1]), res)
    plot_phase(fig.add_subplot(gs[2, 2]), res)
    fig.suptitle(f"Approach to a moving target - {res.law_name}", fontsize=14, fontweight="bold")
    save_figure(fig, save_path)
    if show:
        plt.show()
    return fig
