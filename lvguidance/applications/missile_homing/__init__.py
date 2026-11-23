"""Missile homing intercept application package."""

from lvguidance.applications.missile_homing.runner import (
    monte_carlo_missile,
    simulate_missile,
)
from lvguidance.applications.missile_homing.scenario import MissileScenario
from lvguidance.applications.missile_homing.targets import (
    BallisticTarget,
    ConstantTurnTarget,
    NonManeuveringTarget,
    TargetManeuver,
    WeavingTarget,
)
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.laws.optimal import OptimalHomingGuidance
from lvguidance.core.laws.proportional_nav import (
    AugmentedProportionalNavigation,
    BiasedProportionalNavigation,
    PureProportionalNavigation,
    TrueProportionalNavigation,
)
from lvguidance.core.laws.sliding_mode import SlidingModeHomingGuidance

__all__ = [
    "BallisticTarget",
    "ConstantTurnTarget",
    "MissileScenario",
    "NonManeuveringTarget",
    "TargetManeuver",
    "WeavingTarget",
    "default_missile_laws",
    "monte_carlo_missile",
    "simulate_missile",
]


def default_missile_laws() -> list[GuidanceLaw]:
    """Return default suite of guidance laws for missile intercept."""
    return [
        TrueProportionalNavigation(n_gain=3.0),
        TrueProportionalNavigation(n_gain=4.0),
        PureProportionalNavigation(n_gain=3.0),
        AugmentedProportionalNavigation(n_gain=3.0),
        OptimalHomingGuidance(),
        BiasedProportionalNavigation(),
        SlidingModeHomingGuidance(),
    ]
