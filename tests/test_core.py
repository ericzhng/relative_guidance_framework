"""Tests for core relative kinematics, subsystem models, and guidance laws."""

import numpy as np
import pytest

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
from lvguidance.core.laws.optimal import (
    AdaptiveTgoGuidance,
    OptimalHomingGuidance,
    TerminalVelocityGuidance,
    ZemZevGuidance,
)
from lvguidance.core.laws.phased import PhasedGuidance
from lvguidance.core.laws.proportional_nav import (
    AugmentedProportionalNavigation,
    BiasedProportionalNavigation,
    PureProportionalNavigation,
    TrueProportionalNavigation,
)
from lvguidance.core.laws.pursuit import (
    ProportionalDerivative,
    TerminalApproachGuidance,
    VelocityPursuit,
)
from lvguidance.core.laws.sliding_mode import (
    SlidingModeGuidance,
    SlidingModeHomingGuidance,
)
from lvguidance.core.models.autopilot import AutopilotModel
from lvguidance.core.models.seeker import AlphaBetaFilter, SeekerModel
from lvguidance.core.state import GuidanceState


def test_kinematics_basic_properties():
    rel_pos = np.array([300.0, 400.0, 0.0])
    rel_vel = np.array([-30.0, -40.0, 0.0])

    assert compute_range(rel_pos) == pytest.approx(500.0)
    assert np.allclose(compute_los(rel_pos), [0.6, 0.8, 0.0])
    assert compute_range_rate(rel_pos, rel_vel) == pytest.approx(-50.0)
    assert compute_closing_speed(rel_pos, rel_vel) == pytest.approx(50.0)
    # Collinear position and velocity -> zero LOS rate
    assert np.allclose(compute_los_rate_vector(rel_pos, rel_vel), 0.0)


def test_kinematics_los_rate_cross_velocity():
    rel_pos = np.array([1000.0, 0.0, 0.0])
    rel_vel = np.array([0.0, 100.0, 0.0])

    omega = compute_los_rate_vector(rel_pos, rel_vel)
    # Omega = (r x v) / R^2 = [0, 0, 1000*100] / 1000^2 = [0, 0, 0.1] rad/s
    assert np.allclose(omega, [0.0, 0.0, 0.1])


def test_look_angle_and_t_go_estimation():
    chaser_vel = np.array([500.0, 0.0, 0.0])
    rel_pos = np.array([1000.0, 1000.0, 0.0])

    angle = compute_look_angle(chaser_vel, rel_pos)
    assert angle == pytest.approx(np.pi / 4.0, rel=1e-5)

    t_go_int = estimate_t_go(range_m=1000.0, closing_speed_mps=100.0, rendezvous=False)
    assert t_go_int == pytest.approx(10.0)
    t_go_rnd = estimate_t_go(range_m=1000.0, closing_speed_mps=100.0, rendezvous=True)
    assert t_go_rnd == pytest.approx(20.0)


def test_autopilot_saturation_and_lag():
    ap = AutopilotModel(tau_a=0.1, max_accel=50.0, rate_limit=200.0)
    u_cmd = np.array([100.0, 0.0, 0.0])  # Exceeds max_accel

    # Initial state is zero
    assert np.allclose(ap.current_accel, 0.0)

    # Step by 0.01s: rate is limited by 200 m/s^3 -> max delta = 2 m/s^2
    a1 = ap.step(u_cmd, dt=0.01)
    assert np.linalg.norm(a1) <= 2.0 + 1e-6

    # Step many times to converge to max_accel
    for _ in range(500):
        a1 = ap.step(u_cmd, dt=0.01)
    assert np.linalg.norm(a1) == pytest.approx(50.0, abs=1e-3)


def test_seeker_fov_and_filter():
    seeker = SeekerModel(fov_limit_deg=45.0, noise_std_rad=0.01, seed=42)
    chaser_vel = np.array([100.0, 0.0, 0.0])

    in_pos = np.array([100.0, 50.0, 0.0])  # ~26.5 deg look angle
    out_pos = np.array([10.0, 100.0, 0.0])  # ~84.3 deg look angle

    assert seeker.check_fov(chaser_vel, in_pos) is True
    assert seeker.check_fov(chaser_vel, out_pos) is False

    # Filter smoothing test
    ab = AlphaBetaFilter(alpha=0.6, beta=0.2)
    z1 = np.array([1.0, 0.0, 0.0])
    los_filt, los_rate = ab.update(z1, dt=0.01)
    assert np.allclose(los_filt, z1)
    assert np.allclose(los_rate, 0.0)


def test_guidance_laws_zero_command_on_ideal_state():
    # If state is at rest at target, ideal laws command zero
    state = GuidanceState(t=0.0, rel_pos=np.zeros(3), rel_vel=np.zeros(3), t_final=100.0)
    assert np.allclose(ZemZevGuidance().command(state), 0.0)
    assert np.allclose(TrueProportionalNavigation().command(state), 0.0)
    assert np.allclose(AugmentedProportionalNavigation().command(state), 0.0)
    assert np.allclose(ProportionalDerivative().command(state), 0.0)
    assert np.allclose(BiasedProportionalNavigation().command(state), 0.0)
