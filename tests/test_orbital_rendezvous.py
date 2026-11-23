"""Tests for orbital rendezvous application (both continuous and impulsive algorithms)."""

import numpy as np
import pytest

from relguide.applications.orbital_rendezvous import (
    MU_EARTH,
    ImpulsiveGlideslope,
    LambertTransfer,
    LQRController,
    MPCController,
    OrbitalScenario,
    TwoImpulseCW,
    circular_state,
    default_rendezvous_laws,
    lvlh_rotation,
    run_comparison,
    simulate_rendezvous,
    to_lvlh,
)
from relguide.core.laws.phased import PhasedGuidance
from relguide.core.laws.pursuit import TerminalApproachGuidance
from relguide.core.laws.sliding_mode import SlidingModeGuidance
from relguide.core.simulation.engine import propagate_rk4
from relguide.core.simulation.metrics import summarize


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


# === Tests ported from spacecraft_rendezvous ===

@pytest.fixture(scope="module")
def rendezvous_fixture_scenario() -> OrbitalScenario:
    rel0 = np.array([200.0, -5000.0, 100.0, 0.0, 0.0, 0.0])
    return OrbitalScenario.circular(500e3, rel0, 3600.0)


def test_impulsive_algorithms_arrive(rendezvous_fixture_scenario: OrbitalScenario):
    results = run_comparison(
        rendezvous_fixture_scenario,
        [LambertTransfer(), ImpulsiveGlideslope(6)],
        method="solve_ivp",
    )
    for res in results:
        assert res.range[-1] < 1.0
        assert res.rel_speed[-1] < 1e-3


def test_two_impulse_cw_transfers(rendezvous_fixture_scenario: OrbitalScenario):
    (res,) = run_comparison(
        rendezvous_fixture_scenario,
        [TwoImpulseCW()],
        method="solve_ivp",
    )
    assert 1.0 < res.range[-1] < 200.0
    assert len(res.burns) == 2


def test_mpc_respects_thrust_limit_and_converges(rendezvous_fixture_scenario: OrbitalScenario):
    mpc = MPCController(pos_tol=100.0, vel_tol=0.5, acc_tol=2e-3, a_max=5e-3)
    (res,) = run_comparison(
        rendezvous_fixture_scenario,
        [mpc],
        method="solve_ivp",
    )
    assert np.abs(res.accel).max() <= 5e-3 + 1e-9
    assert res.range[-1] < 50.0


def test_lqr_converges_without_saturation_on_small_offset():
    scn = OrbitalScenario.circular(500e3, np.array([0.0, -200.0, 0.0, 0.0, 0.0, 0.0]), 3000.0)
    (res,) = run_comparison(
        scn,
        [LQRController(pos_tol=100.0, vel_tol=0.5, acc_tol=5e-3)],
        method="solve_ivp",
    )
    assert res.range[-1] < 5.0
