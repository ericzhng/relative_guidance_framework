"""Orbital rendezvous and proximity operations package."""

from lvguidance.applications.orbital_rendezvous.dynamics import (
    MU_EARTH,
    R_EARTH,
    circular_state,
    gravity,
    lvlh_rotation,
    to_lvlh,
)
from lvguidance.applications.orbital_rendezvous.runner import simulate_rendezvous
from lvguidance.applications.orbital_rendezvous.scenario import OrbitalScenario
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.laws.optimal import (
    AdaptiveTgoGuidance,
    TerminalVelocityGuidance,
    ZemZevGuidance,
)
from lvguidance.core.laws.phased import PhasedGuidance
from lvguidance.core.laws.pursuit import (
    ProportionalDerivative,
    TerminalApproachGuidance,
    VelocityPursuit,
)
from lvguidance.core.laws.sliding_mode import SlidingModeGuidance

__all__ = [
    "MU_EARTH",
    "OrbitalScenario",
    "R_EARTH",
    "circular_state",
    "default_rendezvous_laws",
    "gravity",
    "lvlh_rotation",
    "simulate_rendezvous",
    "to_lvlh",
]


def default_rendezvous_laws() -> list[GuidanceLaw]:
    """Return default suite of guidance laws for orbital rendezvous."""
    return [
        VelocityPursuit(),
        ProportionalDerivative(),
        ZemZevGuidance(),
        AdaptiveTgoGuidance(),
        TerminalVelocityGuidance(),
        SlidingModeGuidance(),
        PhasedGuidance(AdaptiveTgoGuidance(), TerminalApproachGuidance()),
        PhasedGuidance(SlidingModeGuidance(), TerminalApproachGuidance()),
    ]
