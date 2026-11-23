"""Orbital rendezvous visualization: 2-D/3-D trajectories, LOS sphere, and Earth orbit views."""

from __future__ import annotations

import logging
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.figure import Figure

from relguide.core.astrodynamics.constants import R_EARTH
from relguide.core.simulation.result import SimulationResult

logger = logging.getLogger(__name__)

PALETTE: tuple[str, ...] = (
    "#0072B2",
    "#D55E00",
    "#009E73",
    "#CC79A7",
    "#E69F00",
    "#56B4E9",
    "#000000",
)
LINESTYLES: tuple[str, ...] = ("-", "--", "-.", ":")


def apply_style() -> None:
    """Apply the package-wide matplotlib rcParams."""
    plt.rcParams.update(
        {
            "axes.grid": True,
            "grid.alpha": 0.3,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.dpi": 110,
            "legend.frameon": False,
            "axes.prop_cycle": plt.cycler(color=list(PALETTE)),
        }
    )


def color_for(index: int) -> str:
    """Return the palette colour for series index."""
    return PALETTE[index % len(PALETTE)]


def linestyle_for(index: int) -> str:
    """Return a linestyle so series stay distinguishable without colour."""
    return LINESTYLES[(index // len(PALETTE)) % len(LINESTYLES)]


def plot_trajectories_2d(
    results: Sequence[SimulationResult],
    coast_ref: SimulationResult | None = None,
) -> Figure:
    """In-plane relative trajectories (along-track y vs radial x), burns marked."""
    apply_style()
    fig, ax = plt.subplots(figsize=(9.6, 5.0))
    for i, res in enumerate(results):
        col, ls = color_for(i), linestyle_for(i)
        st = res.lvlh_state
        ax.plot(st[:, 1] / 1e3, st[:, 0] / 1e3, ls, color=col, label=res.law_name)
        for burn in res.burns:
            k = int(np.argmin(np.abs(res.t - burn.time)))
            ax.plot(st[k, 1] / 1e3, st[k, 0] / 1e3, "o", color=col, ms=5)

    if coast_ref is not None:
        ref_st = coast_ref.lvlh_state
        ax.plot(
            ref_st[:, 1] / 1e3,
            ref_st[:, 0] / 1e3,
            ":",
            color="k",
            lw=1.6,
            label="Chaser original orbit (no burns)",
            zorder=1,
        )

    start = results[0].lvlh_state[0]
    ax.axhline(0.0, color="grey", lw=0.8, alpha=0.6, zorder=0)  # V-bar (x = 0)
    ax.plot(start[1] / 1e3, start[0] / 1e3, "s", color="k", ms=8, label="Start")
    ax.plot(0, 0, "*", color="gold", mec="k", ms=15, label="Target")
    ax.set_xlabel("Along-track y [km]")
    ax.set_ylabel("Radial x [km]")
    ax.set_title("Relative trajectory in the orbital plane (dots = impulsive burns)")
    ax.legend(
        bbox_to_anchor=(1.02, 1.0),
        loc="upper left",
        borderaxespad=0.0,
        fontsize=8,
        frameon=True,
    )
    fig.tight_layout()
    return fig


def plot_trajectories_3d(results: Sequence[SimulationResult]) -> Figure:
    """3-D relative trajectories in the LVLH frame."""
    apply_style()
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(projection="3d")
    for i, res in enumerate(results):
        st = res.lvlh_state
        ax.plot(
            st[:, 1] / 1e3,
            st[:, 2] / 1e3,
            st[:, 0] / 1e3,
            linestyle_for(i),
            color=color_for(i),
            label=res.law_name,
        )
    ax.plot([0], [0], [0], "*", color="gold", mec="k", ms=15, label="Target")
    ax.set_xlabel("Along-track y [km]")
    ax.set_ylabel("Cross-track z [km]")
    ax.set_zlabel("Radial x [km]")
    ax.legend()
    fig.tight_layout()
    return fig


def spherical_coordinates(
    result: SimulationResult, min_range: float = 5.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Chaser position relative to the target as range / azimuth / elevation in LVLH."""
    pos = result.lvlh_state[:, :3]
    rng = np.linalg.norm(pos, axis=1)
    keep = rng >= min_range
    az = np.degrees(np.unwrap(np.arctan2(pos[keep, 1], pos[keep, 0])))
    el = np.degrees(np.arcsin(pos[keep, 2] / rng[keep]))
    return result.t[keep], rng[keep], az, el


def plot_spherical_history(results: Sequence[SimulationResult]) -> Figure:
    """Range, azimuth and elevation of the chaser versus time."""
    apply_style()
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for i, res in enumerate(results):
        t, rng, az, el = spherical_coordinates(res)
        for ax, data in zip(axes, (rng, az, el), strict=True):
            ax.plot(t / 60.0, data, linestyle_for(i), color=color_for(i), label=res.law_name)
    axes[0].set_yscale("log")
    axes[0].set_ylabel("Range [m]")
    axes[1].set_ylabel("Azimuth [deg]\n(from radial, in-plane)")
    axes[2].set_ylabel("Elevation [deg]\n(out of plane)")
    axes[2].set_xlabel("Time [min]")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    return fig


def plot_los_sphere(result: SimulationResult) -> Figure:
    """Line-of-sight track on the unit sphere centred on the target."""
    apply_style()
    fig = plt.figure(figsize=(7, 6.5))
    ax = fig.add_subplot(projection="3d")
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 40), np.linspace(0, np.pi, 20))
    ax.plot_wireframe(
        np.cos(u) * np.sin(v),
        np.sin(u) * np.sin(v),
        np.cos(v),
        color="grey",
        alpha=0.15,
        lw=0.4,
    )
    pos = result.lvlh_state[:, :3]
    rng = np.linalg.norm(pos, axis=1)
    keep = rng >= 5.0
    unit = pos[keep] / rng[keep, None]
    sc = ax.scatter(
        unit[:, 1], unit[:, 2], unit[:, 0], c=np.log10(rng[keep]), cmap="viridis", s=8
    )
    ax.plot(unit[:, 1], unit[:, 2], unit[:, 0], color="k", lw=0.5, alpha=0.5)
    ax.scatter(*unit[0][[1, 2, 0]], color="k", marker="s", s=50, label="Start")
    ax.scatter(*unit[-1][[1, 2, 0]], color="red", marker="X", s=60, label="End")
    ax.set_xlabel("y (along-track)")
    ax.set_ylabel("z (cross-track)")
    ax.set_zlabel("x (radial)")
    ax.set_box_aspect((1, 1, 1))
    ax.set_title(f"{result.law_name}: line-of-sight direction on the unit sphere")
    fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1, label="log10 range [m]")
    ax.legend(loc="upper left")
    fig.tight_layout()
    return fig


def plot_earth_orbit_view(
    results: Sequence[SimulationResult],
    offset_scale: float = 200.0,
) -> Figure:
    """Earth-centred (ECI) view of the target orbit and the magnified chaser tracks."""
    apply_style()
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(projection="3d")
    km = 1e3
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 48), np.linspace(0, np.pi, 24))
    ax.plot_surface(
        R_EARTH / km * np.cos(u) * np.sin(v),
        R_EARTH / km * np.sin(u) * np.sin(v),
        R_EARTH / km * np.cos(v),
        color="#9ecae1",
        alpha=0.35,
        linewidth=0,
    )
    r_t0 = results[0].target[0, :3]
    v_t0 = results[0].target[0, 3:]
    radius = float(np.linalg.norm(r_t0)) / km

    ang = np.linspace(0, 2 * np.pi, 200)
    e1 = r_t0 / np.linalg.norm(r_t0)
    e2 = v_t0 / np.linalg.norm(v_t0)
    ring = radius * (np.outer(np.cos(ang), e1) + np.outer(np.sin(ang), e2))
    ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color="grey", lw=0.8, label="Target orbit")

    for i, res in enumerate(results):
        r_t = res.target[:, :3]
        r_c = res.chaser[:, :3]
        shown = (r_t + offset_scale * (r_c - r_t)) / km
        if i == 0:
            arc = r_t / km
            ax.plot(
                arc[:, 0], arc[:, 1], arc[:, 2], color="k", lw=2.5, label="Target (manoeuvre arc)"
            )
            ax.scatter(*arc[0], color="k", marker="s", s=40)
            ax.scatter(
                *arc[-1], color="gold", edgecolor="k", marker="*", s=160, label="Target at arrival"
            )
        ax.plot(
            shown[:, 0],
            shown[:, 1],
            shown[:, 2],
            linestyle_for(i),
            color=color_for(i),
            label=res.law_name,
        )

    ax.set_xlabel("X ECI [km]")
    ax.set_ylabel("Y ECI [km]")
    ax.set_zlabel("Z ECI [km]")
    lim = radius * 1.05
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1))
    ax.set_title(f"Earth-centred view (chaser offset from target magnified x{offset_scale:g})")
    ax.legend(loc="upper left", fontsize=7)
    fig.tight_layout()
    return fig


def plot_range_history(results: Sequence[SimulationResult]) -> Figure:
    """Distance to target and relative speed versus time (log scale)."""
    apply_style()
    fig, (ax_r, ax_v) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for i, res in enumerate(results):
        col, ls = color_for(i), linestyle_for(i)
        t_min = res.t / 60.0
        ax_r.semilogy(t_min, np.maximum(res.range, 1e-3), ls, color=col, label=res.law_name)
        ax_v.semilogy(t_min, np.maximum(res.rel_speed, 1e-4), ls, color=col, label=res.law_name)
    ax_r.set_ylabel("Range to goal [m]")
    ax_v.set_ylabel("Relative speed [m/s]")
    ax_v.set_xlabel("Time [min]")
    ax_r.legend()
    fig.tight_layout()
    return fig


def plot_thrust_profile(results: Sequence[SimulationResult]) -> Figure:
    """Continuous thrust level and cumulative Delta-v versus time."""
    apply_style()
    fig, (ax_a, ax_d) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for i, res in enumerate(results):
        col, ls = color_for(i), linestyle_for(i)
        t_min = res.t / 60.0
        ax_a.step(
            t_min,
            res.accel_norm * 1e3,
            where="post",
            linestyle=ls,
            color=col,
            label=res.law_name,
        )
        ax_d.step(t_min, res.delta_v, where="post", linestyle=ls, color=col)
    ax_a.set_ylabel("Continuous thrust |a| [mm/s²]")
    ax_d.set_ylabel("Cumulative Delta-v [m/s]")
    ax_d.set_xlabel("Time [min]")
    ax_a.legend()
    fig.tight_layout()
    return fig


def plot_metrics_comparison(entries: Sequence[Any]) -> Figure:
    """Bar charts of Delta-v, final position error and final velocity error."""
    from relguide.applications.orbital_rendezvous.metrics import compute_metrics

    apply_style()
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5))
    results = [e.result if hasattr(e, "result") else e for e in entries]
    metrics = [e.metrics if hasattr(e, "metrics") else compute_metrics(e) for e in entries]
    labels = [r.law_name for r in results]
    colors = [color_for(i) for i in range(len(entries))]
    series = (
        ("Total Delta-v [m/s]", [m.total_delta_v for m in metrics], False),
        ("Final position error [m]", [m.final_position_error for m in metrics], True),
        ("Final velocity error [m/s]", [m.final_velocity_error for m in metrics], True),
    )
    for ax, (title, vals, log) in zip(axes, series, strict=True):
        ax.bar(range(len(vals)), np.maximum(vals, 1e-4) if log else vals, color=colors)
        ax.set_xticks(range(len(vals)), labels, rotation=35, ha="right", fontsize=8)
        ax.set_title(title, fontsize=10)
        if log:
            ax.set_yscale("log")
    fig.tight_layout()
    return fig


def animate_rendezvous(
    results: Sequence[SimulationResult],
    path: str | Path,
    fps: int = 20,
    max_frames: int = 120,
) -> Path:
    """Animate the chasers' motion in the orbital plane and save as a GIF."""
    apply_style()
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    t_end = max(float(r.t[-1]) for r in results)
    frame_t = np.linspace(0.0, t_end, max_frames)
    all_xy = np.vstack([r.lvlh_state[:, :2] for r in results])
    fig, ax = plt.subplots(figsize=(9.6, 5.0))
    ax.set_xlim(all_xy[:, 1].min() / 1e3 * 1.1 - 0.2, all_xy[:, 1].max() / 1e3 * 1.1 + 0.2)
    ax.set_ylim(all_xy[:, 0].min() / 1e3 * 1.1 - 0.2, all_xy[:, 0].max() / 1e3 * 1.1 + 0.2)
    ax.set_xlabel("Along-track y [km]")
    ax.set_ylabel("Radial x [km]")
    ax.axhline(0.0, color="grey", lw=0.8, alpha=0.4, zorder=0)  # V-bar
    ax.plot(0, 0, "*", color="gold", mec="k", ms=15, zorder=5, label="Target")
    trails = [
        ax.plot([], [], color=color_for(i), lw=1.2, label=r.law_name)[0]
        for i, r in enumerate(results)
    ]
    dots = [ax.plot([], [], "o", color=color_for(i), ms=7)[0] for i in range(len(results))]
    time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)
    ax.legend(
        bbox_to_anchor=(1.02, 1.0),
        loc="upper left",
        borderaxespad=0.0,
        fontsize=8,
        frameon=True,
    )
    fig.tight_layout()

    def update(frame_idx: int) -> list[Any]:
        curr_t = frame_t[frame_idx]
        for r, trail, dot in zip(results, trails, dots, strict=True):
            idx = int(np.searchsorted(r.t, curr_t, side="right"))
            idx = max(1, min(idx, len(r.t)))
            xy = r.lvlh_state[:idx, :2]
            trail.set_data(xy[:, 1] / 1e3, xy[:, 0] / 1e3)
            dot.set_data([xy[-1, 1] / 1e3], [xy[-1, 0] / 1e3])
        time_text.set_text(f"t = {curr_t / 60.0:.1f} min")
        return [*trails, *dots, time_text]

    anim = FuncAnimation(fig, update, frames=max_frames, blit=True)
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out
