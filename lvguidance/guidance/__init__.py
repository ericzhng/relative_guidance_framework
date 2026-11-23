"""Guidance laws, from basic pursuit to optimal and sliding-mode laws."""

from lvguidance.guidance.base import GuidanceLaw, GuidanceState
from lvguidance.guidance.basic import ProportionalDerivative, VelocityPursuit
from lvguidance.guidance.optimal import (
    AdaptiveTgoGuidance,
    TerminalVelocityGuidance,
    ZemZevGuidance,
)
from lvguidance.guidance.sliding_mode import SlidingModeGuidance
from lvguidance.guidance.terminal import PhasedGuidance, TerminalApproachGuidance

__all__ = [
    "AdaptiveTgoGuidance",
    "GuidanceLaw",
    "GuidanceState",
    "PhasedGuidance",
    "ProportionalDerivative",
    "SlidingModeGuidance",
    "TerminalApproachGuidance",
    "TerminalVelocityGuidance",
    "VelocityPursuit",
    "ZemZevGuidance",
    "default_laws",
]


def default_laws() -> list[GuidanceLaw]:
    """Return one instance of every law with tuned default parameters."""
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
