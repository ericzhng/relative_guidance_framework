import numpy as np
import pytest

from lvguidance.dynamics import MU_EARTH, circular_state, lvlh_rotation, propagate_step, to_lvlh
from lvguidance.guidance import GuidanceState, ZemZevGuidance, default_laws
from lvguidance.simulation import Scenario, simulate, summarize


def _energy(state):
    r, v = state[:3], state[3:]
    return 0.5 * v @ v - MU_EARTH / np.linalg.norm(r)


def test_circular_orbit_conserves_energy_and_radius():
    r, v = circular_state(6.8e6, np.deg2rad(51.6))
    s0 = np.concatenate((r, v))
    s = s0.copy()
    for _ in range(600):
        s = propagate_step(s, np.zeros(3), 1.0)
    assert _energy(s) == pytest.approx(_energy(s0), rel=1e-9)
    assert np.linalg.norm(s[:3]) == pytest.approx(6.8e6, rel=1e-6)


def test_lvlh_rotation_is_orthonormal():
    r, v = circular_state(6.8e6, 0.9, 0.3)
    rot = lvlh_rotation(r, v)
    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-12)
    assert np.allclose(to_lvlh(r, r, v), [np.linalg.norm(r), 0, 0], atol=1e-6)


def test_zem_zev_is_zero_when_on_the_optimal_path():
    # x = 0 and xdot = 0 already satisfies the terminal condition -> no command.
    state = GuidanceState(0.0, np.zeros(3), np.zeros(3), 100.0)
    assert np.allclose(ZemZevGuidance().command(state), 0.0)


def test_terminal_handover_gives_docking_speed():
    from lvguidance.guidance import PhasedGuidance, SlidingModeGuidance, TerminalApproachGuidance

    sc = Scenario()
    law = PhasedGuidance(
        SlidingModeGuidance(), TerminalApproachGuidance(v_dock=0.3), switch_range=1000.0
    )
    res = simulate(law, sc)
    handover = res.events["handover"]
    assert res.range[res.t < handover].min() > 1000.0 - 1e-6
    assert res.rel_speed[-1] == pytest.approx(0.3, abs=0.1)
    assert res.range[-1] <= sc.capture_radius


@pytest.mark.parametrize("law", default_laws(), ids=lambda law: law.name)
def test_every_law_captures_default_scenario(law):
    sc = Scenario()
    res = simulate(law, sc)
    stats = summarize(res, sc.capture_radius)
    assert stats["success"]
    assert res.accel_norm.max() <= sc.max_accel + 1e-9
    assert res.t.shape == res.range.shape == (len(res.t),)
