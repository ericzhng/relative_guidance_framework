"""Core guidance engine, kinematics, models, laws and simulation."""

from relguide.core.base import GuidanceLaw
from relguide.core.kinematics import (
    compute_closing_speed,
    compute_look_angle,
    compute_los,
    compute_los_rate_vector,
    compute_range,
    compute_range_rate,
    estimate_t_go,
)
from relguide.core.models.autopilot import AutopilotModel
from relguide.core.models.seeker import SeekerModel
from relguide.core.simulation.engine import simulate_loop
from relguide.core.simulation.metrics import format_table, summarize
from relguide.core.simulation.result import SimulationResult
from relguide.core.state import GuidanceState

__all__ = [
    "AutopilotModel",
    "GuidanceLaw",
    "GuidanceState",
    "SeekerModel",
    "SimulationResult",
    "compute_closing_speed",
    "compute_look_angle",
    "compute_los",
    "compute_los_rate_vector",
    "compute_range",
    "compute_range_rate",
    "estimate_t_go",
    "format_table",
    "simulate_loop",
    "summarize",
]
