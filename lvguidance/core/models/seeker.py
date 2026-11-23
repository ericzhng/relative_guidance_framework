"""Seeker sensor model with FOV limits, measurement noise, and tracking filters."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.kinematics import compute_look_angle, compute_los

Vec = NDArray[np.float64]


class AlphaBetaFilter:
    """Discrete alpha-beta tracking filter for smoothing noisy Line-of-Sight measurements.

    Tracks unit LOS vector and estimates its rate of change::

        x_pred = x + dt * v
        v_pred = v
        residual = z - x_pred
        x = x_pred + alpha * residual
        v = v_pred + (beta / dt) * residual

    Args:
        alpha: Position/LOS filter gain (typically 0.4 - 0.8).
        beta: Velocity/LOS rate filter gain (typically 0.1 - 0.4).
    """

    def __init__(self, alpha: float = 0.6, beta: float = 0.2) -> None:
        self.alpha = alpha
        self.beta = beta
        self.x: Vec | None = None
        self.v: Vec = np.zeros(3)

    def reset(self) -> None:
        """Reset internal filter states."""
        self.x = None
        self.v = np.zeros(3)

    def update(self, measured_los: Vec, dt: float) -> tuple[Vec, Vec]:
        """Update filter with a new LOS measurement.

        Args:
            measured_los: Measured unit LOS vector.
            dt: Sample time interval [s].

        Returns:
            Tuple (filtered_los, filtered_los_rate).
        """
        if self.x is None or dt <= 0:
            self.x = measured_los.copy()
            self.v = np.zeros(3)
            return self.x.copy(), self.v.copy()

        # Predict
        x_pred = self.x + dt * self.v
        v_pred = self.v.copy()

        # Update
        residual = measured_los - x_pred
        self.x = x_pred + self.alpha * residual
        norm_x = np.linalg.norm(self.x)
        if norm_x > 1e-9:
            self.x = self.x / norm_x

        self.v = v_pred + (self.beta / dt) * residual
        return self.x.copy(), self.v.copy()


class SeekerModel:
    """Models a gimbaled or strapdown seeker with FOV limits and noise.

    Attributes:
        fov_limit_deg: Maximum permissible look angle [deg].
        noise_std_rad: Standard deviation of additive LOS angular noise [rad].
        bias_rad: Constant angular bias [rad].
    """

    def __init__(
        self,
        fov_limit_deg: float = 60.0,
        noise_std_rad: float = 0.0,
        bias_rad: float = 0.0,
        seed: int | None = None,
    ) -> None:
        self.fov_limit_rad = np.deg2rad(fov_limit_deg)
        self.noise_std_rad = noise_std_rad
        self.bias_rad = bias_rad
        self.rng = np.random.default_rng(seed)
        self.filter = AlphaBetaFilter()

    def reset(self) -> None:
        """Reset internal seeker filter states."""
        self.filter.reset()

    def check_fov(self, chaser_vel: Vec, rel_pos: Vec) -> bool:
        """Check if target remains within the seeker's field of view."""
        look_angle = compute_look_angle(chaser_vel, rel_pos)
        return bool(look_angle <= self.fov_limit_rad)

    def measure_los(self, rel_pos: Vec, dt: float = 0.01, filter_output: bool = False) -> tuple[Vec, Vec]:
        """Measure unit LOS and optionally compute filtered LOS rate.

        Args:
            rel_pos: Target position relative to missile [m].
            dt: Guidance sample step [s].
            filter_output: Whether to return alpha-beta filtered values.

        Returns:
            Tuple (measured_los, estimated_los_rate).
        """
        true_los = compute_los(rel_pos)
        if self.noise_std_rad <= 0.0 and self.bias_rad == 0.0:
            noisy_los = true_los
        else:
            perturbation = self.rng.normal(0.0, self.noise_std_rad, size=3) + self.bias_rad
            noisy = true_los + perturbation
            norm = np.linalg.norm(noisy)
            noisy_los = noisy / norm if norm > 1e-9 else true_los

        if not filter_output:
            return noisy_los, np.zeros(3)

        return self.filter.update(noisy_los, dt)
