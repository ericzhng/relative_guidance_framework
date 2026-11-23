"""Tests for orbital rendezvous application."""

import numpy as np
import pytest

from lvguidance.applications.orbital_rendezvous import (
    MU_EARTH,
    OrbitalScenario,
    circular_state,
    default_rendezvous_laws,
    lvlh_rotation,
    simulate_rendezvous,
    to_lvlh,
)
from lvguidance.core.laws.optimal import AdaptiveTgoGuidance, ZemZevGuidance
from lvguidance.core.laws.phased import PhasedGuidance
from lvguidance.core.laws.pursuit import TerminalApproachGuidance
from lvguidance.core.laws.sliding_mode import SlidingModeGuidance
from lvguidance.core.simulation.engine import propagate_rk4
from lvguidance.core.simulation.metrics import summarize


def _energy(state: np.ndarray) -> float:
    r, v = state[:3], state[3:]
    return float(0.5 * v @ v - MU_EARTH / np.linalg.norm(r))


def test_circular_orbit_conserves_energy_and_radius():
    r, v = circular_state(6.8e6, np.deg2rad(51.6))
    s0 = np.concatenate((r, v))
    s = s0.copy()

    def deriv(st, acc):
        return np.concatenate((st[3:], -MU_EARTH * st[:3] / np.linalg.norm(st[:3]) ** 3 + acc))

    for _ in range(600):
        s = propagate_rk4(s, np.zeros(3), 1.0, deriv)

    assert _energy(s) == pytest.approx(_energy(s0), rel=1e-9)
    assert np.linalg.norm(s[:3]) == pytest.approx(6.8e6, rel=1e-6)


def test_lvlh_rotation_is_orthonormal():
    r, v = circular_state(6.8e6, 0.9, 0.3)
    rot = lvlh_rotation(r, v)
    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-12)
    assert np.allclose(to_lvlh(r, r, v), [np.linalg.norm(r), 0, 0], atol=1e-6)


def test_terminal_handover_gives_docking_speed():
    sc = OrbitalScenario()
    law = PhasedGuidance(
        SlidingModeGuidance(), TerminalApproachGuidance(v_dock=0.3), switch_range=1000.0
    )
    res = simulate_rendezvous(law, sc)
    handover = res.events["handover"]
    assert res.range[res.t < handover].min() > 1000.0 - 1e-6
    assert res.rel_speed[-1] == pytest.approx(0.3, abs=0.1)
    assert res.range[-1] <= sc.capture_radius


@pytest.mark.parametrize("law", default_rendezvous_laws(), ids=lambda law: law.name)
def test_every_rendezvous_law_captures_default_scenario(law):
    sc = OrbitalScenario()
    res = simulate_rendezvous(law, sc)
    stats = summarize(res, sc.capture_radius)
    assert stats["success"]
    assert res.accel_norm.max() <= sc.max_accel + 1e-6
