"""Closed-loop simulation engine for guidance laws (supporting RK4 and SciPy solve_ivp)."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.integrate import solve_ivp

from lvguidance.core.astrodynamics.frames import eci_to_lvlh, lvlh_dcm, lvlh_to_eci
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.models.autopilot import AutopilotModel
from lvguidance.core.simulation.result import Burn, SimulationResult
from lvguidance.core.state import GuidanceState

Vec = NDArray[np.float64]
DynamicsFn = Callable[[Vec, Vec, float], Vec]


def propagate_rk4(state: Vec, accel: Vec, dt: float, deriv_fn: Callable[[Vec, Vec], Vec]) -> Vec:
    """Advance state [r, v] by dt using 4th-order Runge-Kutta.

    Args:
        state: 6-vector [r, v].
        accel: Non-gravitational applied acceleration [m/s^2].
        dt: Integration time step [s].
        deriv_fn: Callable f(state, accel) -> d(state)/dt.
    """
    k1 = deriv_fn(state, accel)
    k2 = deriv_fn(state + 0.5 * dt * k1, accel)
    k3 = deriv_fn(state + 0.5 * dt * k2, accel)
    k4 = deriv_fn(state + dt * k3, accel)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_loop(
    law: GuidanceLaw,
    target0: Vec,
    chaser0: Vec,
    target_deriv: Callable[[Vec, Vec], Vec],
    chaser_deriv: Callable[[Vec, Vec], Vec],
    target_accel_fn: Callable[[float, Vec], Vec] | None = None,
    feedforward_fn: Callable[[Vec, Vec], Vec] | None = None,
    autopilot: AutopilotModel | None = None,
    dt: float = 1.0,
    t_max: float = 1500.0,
    t_final: float | None = None,
    capture_radius: float = 2.0,
    stop_on_opening: bool = False,
    substeps: int = 4,
    scenario: Any = None,
) -> SimulationResult:
    """Core closed-loop simulation loop with fixed-step RK4 propagation.

    Supports continuous thrust acceleration and scheduled discrete impulsive burns.
    """
    law.reset(scenario)
    if autopilot is not None:
        autopilot.reset()

    max_accel = autopilot.max_accel if autopilot is not None else 50.0
    target = target0.copy()
    chaser = chaser0.copy()
    t = 0.0

    times, tgt_hist, chs_hist, acc_hist, cmd_hist = [], [], [], [], []
    burns: list[Burn] = []
    min_range = float("inf")

    burn_times = set(np.round(np.asarray(list(law.burn_times()), dtype=float), 4))
    h = dt / substeps

    while True:
        rel_pos = target[:3] - chaser[:3]
        rel_vel = target[3:] - chaser[3:]
        current_range = float(np.linalg.norm(rel_pos))

        # Check termination criteria
        if current_range <= capture_radius or t >= t_max:
            break
        if stop_on_opening and current_range > min_range + 5.0 and min_range < 500.0:
            break
        if current_range < min_range:
            min_range = current_range

        # Apply impulsive burn if scheduled at current time
        t_key = round(t, 4)
        if t_key in burn_times:
            # Check relative state
            state_now = GuidanceState(
                t=t,
                rel_pos=rel_pos,
                rel_vel=rel_vel,
                t_final=t_final,
                target_accel=np.zeros(3),
                chaser_vel=chaser[3:],
            )
            dv = np.asarray(law.burn(t, state_now), dtype=float)
            if np.any(dv):
                # Apply impulsive burn to chaser velocity in inertial frame
                chaser[3:] += dv
                burns.append(Burn(t, dv))
                rel_vel = target[3:] - chaser[3:]

        # Target maneuver acceleration
        a_target = target_accel_fn(t, target) if target_accel_fn is not None else np.zeros(3)

        # Guidance state and continuous command
        state = GuidanceState(
            t=t,
            rel_pos=rel_pos,
            rel_vel=rel_vel,
            t_final=t_final,
            target_accel=a_target,
            chaser_vel=chaser[3:],
        )
        u_cmd = law.command(state)

        # Differential environment compensation (e.g. differential gravity)
        if feedforward_fn is not None:
            u_cmd = u_cmd + feedforward_fn(target[:3], chaser[:3])

        # Autopilot / actuator dynamics
        if autopilot is not None:
            a_actual = autopilot.step(u_cmd, dt)
        else:
            norm_u = float(np.linalg.norm(u_cmd))
            a_actual = u_cmd if norm_u <= max_accel else u_cmd * (max_accel / norm_u)

        times.append(t)
        tgt_hist.append(target.copy())
        chs_hist.append(chaser.copy())
        acc_hist.append(a_actual.copy())
        cmd_hist.append(u_cmd.copy())

        # Substep numerical integration
        for _ in range(substeps):
            target = propagate_rk4(target, a_target, h, target_deriv)
            chaser = propagate_rk4(chaser, a_actual, h, chaser_deriv)

        t += dt

    # Final sample point at termination
    times.append(t)
    tgt_hist.append(target.copy())
    chs_hist.append(chaser.copy())
    acc_hist.append(a_actual.copy() if times else np.zeros(3))
    cmd_hist.append(u_cmd.copy() if times else np.zeros(3))

    switch_time = getattr(law, "switch_time", None)
    events: dict[str, float] = {}
    if switch_time is not None:
        events["handover"] = float(switch_time)

    return SimulationResult(
        law_name=law.name,
        t=np.asarray(times, dtype=np.float64),
        target=np.array(tgt_hist),
        chaser=np.array(chs_hist),
        accel=np.array(acc_hist),
        cmd_accel=np.array(cmd_hist),
        max_accel=max_accel,
        events=events,
        burns=burns,
    )


def _two_body_rhs(_t: float, y: Vec, mu: float, a_lvlh: Vec) -> Vec:
    """Two-body derivative of [r_t, v_t, r_c, v_c] with LVLH thrust on the chaser."""
    r_t, r_c = y[0:3], y[6:9]
    acc_t = -mu * r_t / np.linalg.norm(r_t) ** 3
    acc_c = -mu * r_c / np.linalg.norm(r_c) ** 3
    if np.any(a_lvlh):
        acc_c = acc_c + lvlh_dcm(r_t, y[3:6]).T @ a_lvlh
    return np.concatenate((y[3:6], acc_t, y[9:12], acc_c))


def simulate_ivp(
    law: GuidanceLaw,
    target_r0: Vec,
    target_v0: Vec,
    chaser_rel0: Vec,
    duration: float,
    mu: float,
    sample_dt: float = 10.0,
    rtol: float = 1e-11,
    atol: float = 1e-8,
    scenario: Any = None,
) -> SimulationResult:
    """High-fidelity simulation using SciPy solve_ivp (DOP853) with hybrid actuation.

    Propagates full nonlinear two-body dynamics in ECI with zero-order hold thrust
    in the LVLH frame, accurately applying impulsive burns at scheduled nodes.
    """
    law.reset(scenario)
    dt = law.control_dt or sample_dt
    burn_times_list = list(law.burn_times())
    burn_times = np.asarray(burn_times_list, dtype=float)

    grid = np.unique(
        np.round(np.concatenate((np.arange(0.0, duration, dt), burn_times, [duration])), 6)
    )
    burn_idx = (
        {int(np.argmin(np.abs(grid - tb))) for tb in burn_times}
        if len(burn_times) > 0
        else set()
    )

    r_c, v_c = lvlh_to_eci(target_r0, target_v0, chaser_rel0)
    y = np.concatenate((target_r0, target_v0, r_c, v_c))
    n_nodes = len(grid)

    target_hist = np.zeros((n_nodes, 6))
    chaser_hist = np.zeros((n_nodes, 6))
    accels = np.zeros((n_nodes, 3))
    burns: list[Burn] = []

    for k, t in enumerate(grid):
        r_t, v_t = y[0:3], y[3:6]
        r_c, v_c = y[6:9], y[9:12]
        x_lvlh = eci_to_lvlh(r_t, v_t, r_c, v_c)

        if k in burn_idx:
            state = GuidanceState(
                t=float(t),
                rel_pos=r_t - r_c,
                rel_vel=v_t - v_c,
                t_final=duration,
                chaser_vel=v_c,
                lvlh_state=x_lvlh,
            )
            dv = np.asarray(law.burn(float(t), state), dtype=float)
            if np.any(dv):
                # Add impulsive burn in LVLH to inertial velocity
                y[9:12] += lvlh_dcm(r_t, v_t).T @ dv
                burns.append(Burn(float(t), dv))
                r_c, v_c = y[6:9], y[9:12]
                x_lvlh = eci_to_lvlh(r_t, v_t, r_c, v_c)

        target_hist[k] = np.concatenate((r_t, v_t))
        chaser_hist[k] = np.concatenate((r_c, v_c))

        if k == n_nodes - 1:
            break

        # Compute continuous command
        state = GuidanceState(
            t=float(t),
            rel_pos=r_t - r_c,
            rel_vel=v_t - v_c,
            t_final=duration,
            chaser_vel=v_c,
            lvlh_state=x_lvlh,
        )
        accel_lvlh = np.asarray(law.command(state), dtype=float)
        accels[k] = accel_lvlh

        # Propagate between grid nodes using DOP853
        sol = solve_ivp(
            _two_body_rhs,
            (float(t), float(grid[k + 1])),
            y,
            method="DOP853",
            rtol=rtol,
            atol=atol,
            args=(mu, accel_lvlh),
        )
        y = sol.y[:, -1]

    max_acc = float(np.max(np.linalg.norm(accels, axis=1))) if len(accels) > 0 else 0.5
    return SimulationResult(
        law_name=law.name,
        t=np.asarray(grid, dtype=np.float64),
        target=target_hist,
        chaser=chaser_hist,
        accel=accels,
        cmd_accel=accels,
        max_accel=max(max_acc, 0.05),
        events={},
        burns=burns,
    )
