"""Optimal guidance: generalized ZEM/ZEV laws for intercept and rendezvous."""

from __future__ import annotations

import numpy as np

from lvguidance.core.base import GuidanceLaw, GuidanceState, Vec
from lvguidance.core.kinematics import estimate_t_go


def _optimal_command(
    x: Vec,
    xdot: Vec,
    tau: float,
    k_r: float = 6.0,
    k_v: float = 2.0,
    v_final: Vec | None = None,
    target_accel: Vec | None = None,
    k_a: float = 0.0,
) -> Vec:
    """Master linear-quadratic optimal guidance command.

    Formula::

        u = (k_r / tau^2) * ZEM - (k_v / tau) * ZEV + k_a * a_t_perp

    where:
        ZEM = x + xdot * tau
        ZEV = xdot - v_final
    """
    vf = np.zeros(3) if v_final is None else v_final
    zem = x + xdot * tau
    zev = xdot - vf
    u = (k_r / tau**2) * zem - (k_v / tau) * zev
    if target_accel is not None and k_a != 0.0:
        norm_x = np.linalg.norm(x)
        los = x / norm_x if norm_x > 1e-9 else np.array([1.0, 0.0, 0.0])
        a_t_perp = target_accel - float(target_accel @ los) * los
        u = u + k_a * a_t_perp
    return u


class OptimalHomingGuidance(GuidanceLaw):
    """Optimal Guidance Law (OGL) for missile intercept (zero miss distance).

    Energy-optimal intercept with unconstrained terminal velocity corresponds to
    effective navigation gain N' = 3::

        u = (3 / t_go^2) * ZEM = (3 / t_go^2) * (x + xdot * t_go)

    Args:
        t_go_min: Floor on time-to-go to avoid singularity [s].
    """

    name = "Optimal Homing Guidance (OGL)"

    def __init__(self, t_go_min: float = 0.5) -> None:
        self.t_go_min = t_go_min

    def command(self, state: GuidanceState) -> Vec:
        """Compute the optimal intercept command."""
        tau = state.t_go if state.t_final is not None else estimate_t_go(state.range, state.closing_speed)
        tau = max(tau, self.t_go_min)
        return _optimal_command(state.rel_pos, state.rel_vel, tau, k_r=3.0, k_v=0.0)


class ZemZevGuidance(GuidanceLaw):
    """Fixed-time energy-optimal ZEM/ZEV guidance for orbital rendezvous.

    Drives both relative position and relative velocity to zero simultaneously (N'=6,2)::

        u = (6 / tau^2) * ZEM - (2 / tau) * ZEV

    Args:
        t_go_min: Floor on time-to-go to avoid singularity [s].
    """

    name = "ZEM/ZEV (fixed time)"

    def __init__(self, t_go_min: float = 5.0) -> None:
        self.t_go_min = t_go_min

    def command(self, state: GuidanceState) -> Vec:
        """Compute the fixed-time optimal rendezvous command."""
        tau = max(state.t_go, self.t_go_min)
        return _optimal_command(state.rel_pos, state.rel_vel, tau, k_r=6.0, k_v=2.0)


class AdaptiveTgoGuidance(GuidanceLaw):
    """ZEM/ZEV guidance with time-to-go estimated online from range and closing speed.

    Args:
        a_nom: Nominal acceleration of the reference braking maneuver [m/s^2].
        t_go_min: Floor on time-to-go estimate [s].
    """

    name = "ZEM/ZEV (adaptive t_go)"

    def __init__(self, a_nom: float = 0.2, t_go_min: float = 5.0) -> None:
        self.a_nom = a_nom
        self.t_go_min = t_go_min

    def command(self, state: GuidanceState) -> Vec:
        """Compute the adaptive t_go optimal rendezvous command."""
        tau = estimate_t_go(state.range, state.closing_speed, accel_nom=self.a_nom, t_go_min=self.t_go_min)
        return _optimal_command(state.rel_pos, state.rel_vel, tau, k_r=6.0, k_v=2.0)


class TerminalVelocityGuidance(GuidanceLaw):
    """Energy-optimal guidance arriving with a prescribed docking velocity.

    Args:
        v_dock: Desired closing speed at contact [m/s].
        t_go_min: Floor on time-to-go [s].
    """

    name = "Optimal (docking speed)"

    def __init__(self, v_dock: float = 0.5, t_go_min: float = 5.0) -> None:
        self.v_dock = v_dock
        self.t_go_min = t_go_min
        self._axis: Vec | None = None

    def reset(self) -> None:
        """Reset approach axis."""
        self._axis = None

    def command(self, state: GuidanceState) -> Vec:
        """Compute the optimal command arriving at v_final = -v_dock * axis."""
        if self._axis is None:
            self._axis = state.los.copy()
        tau = max(state.t_go, self.t_go_min)
        v_final = -self.v_dock * self._axis
        return _optimal_command(state.rel_pos, state.rel_vel, tau, k_r=6.0, k_v=2.0, v_final=v_final)
