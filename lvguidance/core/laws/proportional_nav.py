"""Proportional Navigation (PN) guidance law family."""

from __future__ import annotations

import numpy as np

from lvguidance.core.base import GuidanceLaw, GuidanceState, Vec


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
