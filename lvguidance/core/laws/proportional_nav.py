"""Proportional Navigation (PN) guidance law family."""

from __future__ import annotations

from typing import Any

import numpy as np

from lvguidance.core.base import GuidanceLaw, GuidanceState, Vec
from lvguidance.core.kinematics import estimate_t_go


class TrueProportionalNavigation(GuidanceLaw):
    """True Proportional Navigation (TPN).

    The acceleration command is applied perpendicular to the Line of Sight (LOS)::

        u = N * V_c * (Omega x los)

    where N is the navigation ratio (typically 3 to 5), V_c is the closing speed,
    Omega is the LOS angular rate vector, and los is the unit LOS vector.

    Args:
        n_gain: Effective navigation ratio N [-].
    """

    name = "True Proportional Navigation"

    def __init__(self, n_gain: float = 3.0) -> None:
        self.n_gain = n_gain

    def command(self, state: GuidanceState) -> Vec:
        """Compute the TPN acceleration command."""
        v_c = max(state.closing_speed, 0.0)
        omega = state.los_rate_vector
        return self.n_gain * v_c * np.cross(omega, state.los)


class PureProportionalNavigation(GuidanceLaw):
    """Pure Proportional Navigation (PPN).

    The acceleration command is applied perpendicular to the missile velocity vector::

        u = N * V_c * (Omega x v_c_hat)

    Args:
        n_gain: Effective navigation ratio N [-].
    """

    name = "Pure Proportional Navigation"

    def __init__(self, n_gain: float = 3.0) -> None:
        self.n_gain = n_gain

    def command(self, state: GuidanceState) -> Vec:
        """Compute the PPN acceleration command."""
        v_c = max(state.closing_speed, 0.0)
        omega = state.los_rate_vector
        if state.chaser_vel is not None and np.linalg.norm(state.chaser_vel) > 1e-3:
            v_hat = state.chaser_vel / np.linalg.norm(state.chaser_vel)
            return self.n_gain * v_c * np.cross(omega, v_hat)
        # Fallback to TPN if missile velocity vector is not provided
        return self.n_gain * v_c * np.cross(omega, state.los)


class AugmentedProportionalNavigation(GuidanceLaw):
    """Augmented Proportional Navigation (APN).

    Compensates for target acceleration normal to the line of sight::

        u = N * V_c * (Omega x los) + 0.5 * N * a_target_perp

    Args:
        n_gain: Effective navigation ratio N [-].
    """

    name = "Augmented Proportional Navigation"

    def __init__(self, n_gain: float = 3.0) -> None:
        self.n_gain = n_gain

    def command(self, state: GuidanceState) -> Vec:
        """Compute the APN acceleration command."""
        v_c = max(state.closing_speed, 0.0)
        omega = state.los_rate_vector
        cmd_tpn = self.n_gain * v_c * np.cross(omega, state.los)

        if state.target_accel is not None:
            # Component of target acceleration perpendicular to LOS
            a_t = state.target_accel
            los = state.los
            a_t_perp = a_t - float(a_t @ los) * los
            cmd_tpn = cmd_tpn + 0.5 * self.n_gain * a_t_perp

        return cmd_tpn


class BiasedProportionalNavigation(GuidanceLaw):
    """Biased Proportional Navigation (BPN / Impact Angle Control Guidance).

    Augments standard PN with an impact-angle bias term to constrain terminal direction::

        u = N * V_c * (Omega x los) + (k_bias * V_c / t_go) * (v_desired_hat - v_hat)_perp

    Args:
        desired_dir: Desired terminal velocity unit vector (defaults to line-of-sight).
        n_gain: Navigation ratio N [-].
        k_bias: Impact angle guidance bias gain [-].
        t_go_min: Floor on time-to-go [s].
    """

    name = "Biased Proportional Navigation (IACG)"

    def __init__(
        self,
        desired_dir: Vec | None = None,
        n_gain: float = 3.0,
        k_bias: float = 1.0,
        t_go_min: float = 0.5,
    ) -> None:
        self.n_gain = n_gain
        self.k_bias = k_bias
        self.t_go_min = t_go_min
        self.desired_dir = (
            desired_dir / np.linalg.norm(desired_dir) if desired_dir is not None else None
        )

    def reset(self, scenario: Any = None) -> None:
        """Reset internal state."""
        pass

    def command(self, state: GuidanceState) -> Vec:
        """Compute BPN command with impact-angle steering."""
        v_c = max(state.closing_speed, 0.0)
        omega = state.los_rate_vector
        cmd = self.n_gain * v_c * np.cross(omega, state.los)

        if state.chaser_vel is not None and np.linalg.norm(state.chaser_vel) > 1e-3:
            v_hat = state.chaser_vel / np.linalg.norm(state.chaser_vel)
            t_go = (
                state.t_go
                if state.t_final is not None
                else estimate_t_go(state.range, state.closing_speed)
            )
            t_go = max(t_go, self.t_go_min)

            # Target direction: if not specified, default to line-of-sight
            target_dir = self.desired_dir if self.desired_dir is not None else state.los
            dir_error = target_dir - v_hat
            dir_error_perp = dir_error - float(dir_error @ v_hat) * v_hat
            # Smoothly fade bias term in terminal seconds to let TPN null miss distance
            fade = min(t_go / 3.0, 1.0)
            bias_accel = fade * (self.k_bias * v_c / t_go) * dir_error_perp
            cmd = cmd + bias_accel

        return cmd
