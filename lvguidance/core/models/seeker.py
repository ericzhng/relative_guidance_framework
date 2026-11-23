"""Seeker sensor model with FOV limits and measurement noise."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.kinematics import compute_look_angle, compute_los

Vec = NDArray[np.float64]


class SeekerModel:
    """Models a gimbaled or strapdown seeker.

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

    def check_fov(self, chaser_vel: Vec, rel_pos: Vec) -> bool:
        """Check if target remains within the seeker's field of view."""
        look_angle = compute_look_angle(chaser_vel, rel_pos)
        return bool(look_angle <= self.fov_limit_rad)

    def measure_los(self, rel_pos: Vec) -> Vec:
        """Measure unit LOS with optional noise and bias."""
        true_los = compute_los(rel_pos)
        if self.noise_std_rad <= 0.0 and self.bias_rad == 0.0:
            return true_los

        # Add small perturbation perpendicular to true LOS
        perturbation = self.rng.normal(0.0, self.noise_std_rad, size=3) + self.bias_rad
        noisy = true_los + perturbation
        norm = np.linalg.norm(noisy)
        return noisy / norm if norm > 1e-9 else true_los
