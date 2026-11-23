"""Tests for missile homing intercept application."""

import numpy as np
import pytest

from lvguidance.applications.missile_homing import (
    ConstantTurnTarget,
    MissileScenario,
    NonManeuveringTarget,
    WeavingTarget,
    default_missile_laws,
    monte_carlo_missile,
    simulate_missile,
)
from lvguidance.core.laws.optimal import OptimalHomingGuidance
from lvguidance.core.laws.proportional_nav import (
    AugmentedProportionalNavigation,
    PureProportionalNavigation,
    TrueProportionalNavigation,
)
from lvguidance.core.laws.sliding_mode import SlidingModeHomingGuidance


@pytest.mark.parametrize("law", default_missile_laws(), ids=lambda law: law.name)
def test_missile_laws_intercept_non_maneuvering_target(law):
    scenario = MissileScenario(
        heading_error_deg=5.0,
        target_maneuver=NonManeuveringTarget(),
        capture_radius=2.0,
    )
    res = simulate_missile(law, scenario)
    # Missile should achieve a small miss distance (CPA < 2.0 m)
    assert res.miss_distance <= 2.0


def test_apn_intercepts_weaving_target():
    # Test APN vs a weaving target pulling 4g
    scenario = MissileScenario(
        heading_error_deg=5.0,
        target_maneuver=WeavingTarget(amplitude=4.0 * 9.81, frequency=0.5),
        max_accel=300.0,  # 30g
        capture_radius=2.5,
    )
    res = simulate_missile(AugmentedProportionalNavigation(n_gain=4.0), scenario)
    assert res.miss_distance <= 2.5


def test_monte_carlo_runner():
    scenario = MissileScenario(heading_error_deg=5.0)
    law = TrueProportionalNavigation(n_gain=3.0)
    mc = monte_carlo_missile(law, scenario, num_runs=5, heading_error_std_deg=2.0, seed=123)
    assert mc["runs"] == 5
    assert mc["mean_miss_m"] < 3.0
    assert len(mc["all_misses"]) == 5
