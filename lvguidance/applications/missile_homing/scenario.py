"""Scenario configuration for missile homing intercept problems."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from lvguidance.applications.missile_homing.targets import NonManeuveringTarget, TargetManeuver
from lvguidance.core.base import Vec
from lvguidance.core.models.autopilot import AutopilotModel


@dataclass(frozen=True)
class MissileScenario:
    """Engagement scenario for a homing missile intercepting a flying target.

    Attributes:
        name: Name of the engagement scenario.
        target_pos0: Target initial position [m].
        target_vel0: Target initial velocity [m/s].
        missile_pos0: Missile initial position [m].
        missile_speed0: Missile initial speed [m/s].
        heading_error_deg: Initial heading error relative to collision course [deg].
        target_maneuver: Maneuver profile executed by the target.
        max_accel: Missile structural/aerodynamic acceleration limit [m/s^2] (e.g. 200 m/s^2 ~ 20g).
        tau_a: Autopilot lag time constant [s] (e.g. 0.05 to 0.1 s).
        rate_limit: Acceleration slew rate limit [m/s^3] (e.g. 2000 m/s^3).
        dt: Guidance and integration step size [s] (e.g. 0.01 s).
        t_max: Maximum engagement duration [s].
        capture_radius: Intercept threshold (lethal radius) [m].
    """

    name: str = "Missile Intercept"
    target_pos0: Vec = field(default_factory=lambda: np.array([10000.0, 2000.0, 1000.0]))
    # Transonic target velocity
    target_vel0: Vec = field(default_factory=lambda: np.array([-300.0, 50.0, 0.0]))
    missile_pos0: Vec = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    missile_speed0: float = 800.0  # Mach 2.4+ missile
    heading_error_deg: float = 10.0
    target_maneuver: TargetManeuver = field(default_factory=NonManeuveringTarget)
    max_accel: float = 250.0  # ~25 g
    tau_a: float = 0.05  # 50 ms autopilot response time
    rate_limit: float = 2000.0  # 200 g/s slew rate
    dt: float = 0.01  # 100 Hz guidance loop
    t_max: float = 40.0
    capture_radius: float = 1.0

    def initial_states(self) -> tuple[Vec, Vec]:
        """Construct initial 6-states [r, v] for target and missile."""
        target = np.concatenate((self.target_pos0, self.target_vel0))

        # Nominal line-of-sight vector
        los = self.target_pos0 - self.missile_pos0
        los_norm = float(np.linalg.norm(los))
        los_hat = los / los_norm if los_norm > 1e-9 else np.array([1.0, 0.0, 0.0])

        # Introduce initial heading error
        theta = float(np.deg2rad(self.heading_error_deg))
        # Choose a perturbation axis perpendicular to LOS
        axis = np.array([0.0, 0.0, 1.0]) if abs(los_hat[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
        perp = np.cross(los_hat, axis)
        perp = perp / np.linalg.norm(perp)
        missile_vel_dir = los_hat * np.cos(theta) + perp * np.sin(theta)
        missile_vel = self.missile_speed0 * missile_vel_dir

        missile = np.concatenate((self.missile_pos0, missile_vel))
        return target, missile

    def make_autopilot(self) -> AutopilotModel:
        """Create configured autopilot instance."""
        return AutopilotModel(
            tau_a=self.tau_a,
            max_accel=self.max_accel,
            rate_limit=self.rate_limit,
        )
