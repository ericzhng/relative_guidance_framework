"""Core guidance engine, kinematics, models, laws and simulation."""

from lvguidance.core.base import GuidanceLaw
from lvguidance.core.kinematics import (
    compute_closing_speed,
    compute_look_angle,
    compute_los,
    compute_los_rate_vector,
    compute_range,
    compute_range_rate,
    estimate_t_go,
)
from lvguidance.core.models.autopilot import AutopilotModel
from lvguidance.core.models.seeker import SeekerModel
from lvguidance.core.simulation.engine import simulate_loop
from lvguidance.core.simulation.metrics import format_table, summarize
from lvguidance.core.simulation.result import SimulationResult
from lvguidance.core.state import GuidanceState

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
