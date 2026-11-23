"""Tests for core astrodynamics: Kepler propagation, Lambert solver, CW dynamics and frames."""

import numpy as np
import pytest
from scipy.linalg import expm

from lvguidance.core.astrodynamics import (
    MU_EARTH,
    cw_matrices,
    cw_required_velocity,
    cw_stm,
    discretize_zoh,
    eci_to_lvlh,
    lvlh_to_eci,
    mean_motion,
    propagate_kepler,
    solve_lambert,
)

N_REF = mean_motion(6_878_137.0)


def test_cw_stm_matches_matrix_exponential():
    a_mat, _ = cw_matrices(N_REF)
    assert np.allclose(cw_stm(N_REF, 1234.0), expm(a_mat * 1234.0), atol=1e-9)


def test_frame_round_trip():
    r_t = np.array([6.9e6, 1e5, 2e5])
    v_t = np.array([-100.0, 7400.0, 300.0])
    rel = np.array([100.0, -2000.0, 50.0, 0.1, -0.2, 0.3])
    r_c, v_c = lvlh_to_eci(r_t, v_t, rel)
    assert np.allclose(eci_to_lvlh(r_t, v_t, r_c, v_c), rel)


def test_cw_targeting_reaches_goal():
    r0 = np.array([100.0, -3000.0, 20.0])
    v = cw_required_velocity(N_REF, 2000.0, r0, np.zeros(3))
    x = cw_stm(N_REF, 2000.0) @ np.concatenate((r0, v))
    assert np.allclose(x[:3], 0.0, atol=1e-6)


def test_cw_targeting_singular_at_full_period():
    with pytest.raises(ValueError):
        cw_required_velocity(N_REF, 2 * np.pi / N_REF, np.ones(3), np.zeros(3))


@pytest.mark.parametrize("tof", [1500.0, 3500.0, 4500.0])
def test_lambert_reproduces_endpoint(tof: float):
    r1 = np.array([6_878_137.0, 0.0, 0.0])
    v1 = np.array([0.0, 7600.0, 0.0])
    r2, _ = propagate_kepler(r1, v1 + np.array([0.0, 30.0, 0.0]), tof, mu=MU_EARTH)
    v1_sol, v2_sol = solve_lambert(r1, r2, tof, mu=MU_EARTH)
    r_chk, v_chk = propagate_kepler(r1, v1_sol, tof, mu=MU_EARTH)
    assert np.allclose(r_chk, r2, atol=1e-3)
    assert np.allclose(v_chk, v2_sol, atol=1e-6)
