"""Scenario definition for orbital rendezvous and proximity operations."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from lvguidance.core.astrodynamics.constants import MU_EARTH, R_EARTH
from lvguidance.core.astrodynamics.frames import eci_to_lvlh, lvlh_dcm
from lvguidance.core.base import Vec


def gravity(r: Vec, mu: float = MU_EARTH) -> Vec:
    """Central-body gravitational acceleration vector [m/s^2]."""
    r_norm = float(np.linalg.norm(r))
    return -mu * r / max(r_norm**3, 1e-9)


def circular_state(
    radius: float,
    inclination: float = 0.0,
    phase: float = 0.0,
    mu: float = MU_EARTH,
) -> tuple[Vec, Vec]:
    """Build inertial state [r, v] of a circular orbit."""
    speed = float(np.sqrt(mu / radius))
    ci, si = np.cos(inclination), np.sin(inclination)
    cp, sp = np.cos(phase), np.sin(phase)
    r_hat = np.array([cp, sp * ci, sp * si], dtype=float)
    t_hat = np.array([-sp, cp * ci, cp * si], dtype=float)
    return radius * r_hat, speed * t_hat


@dataclass(frozen=True)
class OrbitalScenario:
    """Orbital rendezvous scenario parameters.

    Supports both inertial launch-vehicle relative scenarios and Hill/LVLH scenarios.

    Attributes:
        name: Label of the scenario.
        target_radius: Circular orbit radius [m].
        target_inclination: Orbit inclination [rad].
        rel_pos0: Initial relative position in inertial frame (target - chaser) [m].
        rel_vel0: Initial relative velocity in inertial frame (target - chaser) [m/s].
        t_final: Nominal arrival time for fixed-time laws [s].
        t_max: Simulation time limit [s].
        max_accel: Vehicle thrust acceleration limit [m/s^2].
        capture_radius: Docking completion capture radius [m].
        dt: Guidance step size [s].
        gravity_feedforward: Whether differential gravity is compensated.
        mu: Gravitational parameter [m^3/s^2].
        goal: Desired final relative state in LVLH [m, m/s].
    """

    name: str = "LEO Rendezvous"
    target_radius: float = R_EARTH + 400e3
    target_inclination: float = float(np.deg2rad(51.6))
    rel_pos0: Vec = field(default_factory=lambda: np.array([6000.0, -25000.0, 4000.0]))
    rel_vel0: Vec = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    t_final: float = 900.0
    t_max: float = 1500.0
    max_accel: float = 0.5
    capture_radius: float = 2.0
    dt: float = 1.0
    gravity_feedforward: bool = True
    mu: float = MU_EARTH
    goal: Vec = field(default_factory=lambda: np.zeros(6))

    @classmethod
    def circular(
        cls,
        altitude: float,
        chaser_rel0: Vec,
        duration: float,
        goal: Vec | None = None,
        max_accel: float = 0.05,
        dt: float = 10.0,
        mu: float = MU_EARTH,
    ) -> OrbitalScenario:
        """Build a scenario around a circular equatorial target orbit from LVLH offset.

        Args:
            altitude: Target altitude above Earth radius [m].
            chaser_rel0: Initial chaser relative state in LVLH [x, y, z, vx, vy, vz].
            duration: Total manoeuvre duration [s].
            goal: Desired final LVLH state; defaults to origin at rest.
            max_accel: Thrust acceleration limit [m/s^2].
            dt: Guidance update step [s].
            mu: Gravitational parameter [m^3/s^2].
        """
        radius = R_EARTH + altitude
        speed = math.sqrt(mu / radius)
        target_r0 = np.array([radius, 0.0, 0.0])
        target_v0 = np.array([0.0, speed, 0.0])

        rot = lvlh_dcm(target_r0, target_v0)
        omega = np.cross(target_r0, target_v0) / float(np.dot(target_r0, target_r0))
        rho = rot.T @ chaser_rel0[:3]
        rho_dot = rot.T @ chaser_rel0[3:]

        # target minus chaser in inertial:
        rel_pos0 = -rho
        rel_vel0 = -(rho_dot + np.cross(omega, rho))

        return cls(
            name=f"Circular {altitude/1e3:.0f}km Rendezvous",
            target_radius=radius,
            target_inclination=0.0,
            rel_pos0=rel_pos0,
            rel_vel0=rel_vel0,
            t_final=duration,
            t_max=duration + 100.0,
            max_accel=max_accel,
            capture_radius=2.0,
            dt=dt,
            mu=mu,
            goal=np.zeros(6) if goal is None else np.asarray(goal, dtype=float),
        )

    @property
    def mean_motion(self) -> float:
        """Target orbit mean motion n = sqrt(mu / a^3) [rad/s]."""
        return float(np.sqrt(self.mu / self.target_radius**3))

    @property
    def duration(self) -> float:
        """Total manoeuvre duration [s]."""
        return self.t_final

    @property
    def target_r0(self) -> Vec:
        """Initial target position vector in inertial frame [m]."""
        r_t, _ = circular_state(self.target_radius, self.target_inclination, mu=self.mu)
        return r_t

    @property
    def target_v0(self) -> Vec:
        """Initial target velocity vector in inertial frame [m/s]."""
        _, v_t = circular_state(self.target_radius, self.target_inclination, mu=self.mu)
        return v_t

    @property
    def chaser_rel0(self) -> Vec:
        """Initial chaser state in target LVLH frame [x, y, z, vx, vy, vz]."""
        r_t, v_t = self.target_r0, self.target_v0
        r_c = r_t - self.rel_pos0
        v_c = v_t - self.rel_vel0
        return eci_to_lvlh(r_t, v_t, r_c, v_c)

    def initial_states(self) -> tuple[Vec, Vec]:
        """Construct inertial 6-states [r, v] for target and chaser."""
        r_t, v_t = self.target_r0, self.target_v0
        target = np.concatenate((r_t, v_t))
        chaser = np.concatenate((r_t - self.rel_pos0, v_t - self.rel_vel0))
        return target, chaser

    def differential_gravity(self, r_target: Vec, r_chaser: Vec) -> Vec:
        """Differential gravity feedforward g(r_t) - g(r_c) [m/s^2]."""
        return gravity(r_target, self.mu) - gravity(r_chaser, self.mu)
