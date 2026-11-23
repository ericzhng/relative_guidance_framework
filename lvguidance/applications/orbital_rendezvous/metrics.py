"""Orbital rendezvous metrics and performance figures."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from lvguidance.core.simulation.result import SimulationResult


@dataclass(frozen=True)
class RendezvousMetrics:
    """Summary figures of merit for one rendezvous run.

    Attributes:
        total_delta_v: Sum of impulsive burns plus integrated continuous thrust [m/s].
        impulsive_delta_v: Sum of impulsive burn magnitudes [m/s].
        continuous_delta_v: Integral of continuous acceleration over time [m/s].
        final_position_error: Distance to target at the end [m].
        final_velocity_error: Relative speed at the end [m/s].
        max_acceleration: Peak continuous thrust acceleration norm [m/s^2].
        num_burns: Number of impulsive burns applied.
    """

    total_delta_v: float
    impulsive_delta_v: float
    continuous_delta_v: float
    final_position_error: float
    final_velocity_error: float
    max_acceleration: float
    num_burns: int


def compute_metrics(result: SimulationResult) -> RendezvousMetrics:
    """Compute summary metrics for a rendezvous simulation run."""
    dt = np.diff(result.t, prepend=result.t[0])
    cont_dv = float(np.sum(result.accel_norm * dt))
    imp_dv = sum(float(np.linalg.norm(b.dv)) for b in result.burns)
    tot_dv = cont_dv + imp_dv
    final_pos_err = float(result.range[-1])
    final_vel_err = float(result.rel_speed[-1])
    max_acc = float(np.max(result.accel_norm)) if len(result.accel_norm) > 0 else 0.0

    return RendezvousMetrics(
        total_delta_v=tot_dv,
        impulsive_delta_v=imp_dv,
        continuous_delta_v=cont_dv,
        final_position_error=final_pos_err,
        final_velocity_error=final_vel_err,
        max_acceleration=max_acc,
        num_burns=len(result.burns),
    )


def format_table(results_or_entries: Sequence[Any], capture_radius: float = 2.0) -> str:
    """Render a comprehensive comparison table for rendezvous runs."""
    header = (
        f"{'Algorithm':<28}{'Captured':>9}{'Time [s]':>10}{'Pos.err [m]':>13}"
        f"{'Vel.err [m/s]':>15}{'Total Delta-v [m/s]':>20}{'Burns':>7}"
    )
    lines = [header, "-" * len(header)]
    for item in results_or_entries:
        if hasattr(item, "result") and hasattr(item, "metrics"):
            res = item.result
            m = item.metrics
        else:
            res = item
            m = compute_metrics(res)
        captured = "yes" if m.final_position_error <= capture_radius else "no"
        lines.append(
            f"{res.law_name:<28}{captured:>9}{float(res.t[-1]):>10.1f}"
            f"{m.final_position_error:>13.3f}{m.final_velocity_error:>15.4f}"
            f"{m.total_delta_v:>20.3f}{m.num_burns:>7d}"
        )
    return "\n".join(lines)


format_rendezvous_table = format_table

__all__ = ["RendezvousMetrics", "compute_metrics", "format_rendezvous_table", "format_table"]
