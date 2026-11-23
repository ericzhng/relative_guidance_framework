"""Performance metrics evaluation for guidance laws."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from lvguidance.core.simulation.result import SimulationResult


def summarize(result: SimulationResult, capture_radius: float = 2.0) -> dict[str, float | str | bool]:
    """Compute performance summary statistics of a guidance run.

    Args:
        result: Simulation history.
        capture_radius: Range regarded as successful intercept/capture [m].

    Returns:
        Dictionary with performance metrics:
        - law: Name of the law
        - success: Whether min range <= capture_radius
        - time_s: Duration of engagement [s]
        - final_range_m: Range at final sample [m]
        - miss_distance_m: Closest Point of Approach (CPA) [m]
        - final_rel_speed_mps: Relative speed at end [m/s]
        - delta_v_mps: Fuel spent (integral of acceleration) [m/s]
        - peak_accel_mps2: Peak acceleration demand [m/s^2]
        - saturated_fraction: Fraction of guidance cycles spent at acceleration limit
    """
    saturated = float((result.accel_norm >= 0.999 * result.max_accel).mean())
    cpa = result.miss_distance
    return {
        "law": result.law_name,
        "success": bool(cpa <= capture_radius),
        "time_s": float(result.t[-1]),
        "final_range_m": float(result.range[-1]),
        "miss_distance_m": float(cpa),
        "final_rel_speed_mps": float(result.rel_speed[-1]),
        "delta_v_mps": float(result.delta_v[-1]),
        "peak_accel_mps2": float(result.accel_norm.max()),
        "saturated_fraction": saturated,
    }


def format_table(results: Sequence[SimulationResult], capture_radius: float = 2.0) -> str:
    """Render a formatted comparison table for display."""
    header = (
        f"{'Law':<30}{'Success':>9}{'Time [s]':>10}{'Miss [m]':>11}"
        f"{'Rel.v [m/s]':>13}{'dV [m/s]':>10}{'Peak a':>9}{'Sat. %':>8}"
    )
    lines = [header, "-" * len(header)]
    for res in results:
        s = summarize(res, capture_radius)
        lines.append(
            f"{str(s['law']):<30}{'yes' if s['success'] else 'no':>9}{float(s['time_s']):>10.1f}"
            f"{float(s['miss_distance_m']):>11.2f}{float(s['final_rel_speed_mps']):>13.2f}"
            f"{float(s['delta_v_mps']):>10.1f}{float(s['peak_accel_mps2']):>9.2f}"
            f"{100 * float(s['saturated_fraction']):>8.1f}"
        )
    return "\n".join(lines)
