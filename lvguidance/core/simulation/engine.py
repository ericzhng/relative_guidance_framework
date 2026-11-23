"""Closed-loop simulation engine for guidance laws."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np
from numpy.typing import NDArray

from lvguidance.core.base import GuidanceLaw
from lvguidance.core.models.autopilot import AutopilotModel
from lvguidance.core.simulation.result import SimulationResult
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
) -> SimulationResult:
    """Core closed-loop simulation loop.

    Args:
        law: Guidance law to test.
        target0: Initial target 6-state [r, v].
        chaser0: Initial chaser 6-state [r, v].
        target_deriv: Function computing target state derivative.
        chaser_deriv: Function computing chaser state derivative.
        target_accel_fn: Optional function returning target maneuver acceleration a_T(t, state).
        feedforward_fn: Optional environment feed-forward (e.g., differential gravity).
        autopilot: Autopilot dynamics model (optional).
        dt: Guidance update cycle [s].
        t_max: Simulation cutoff time [s].
        t_final: Planned arrival time [s].
        capture_radius: Stop if range drops below this [m].
        stop_on_opening: Stop if vehicle starts opening after a close approach.
        substeps: RK4 integration substeps per guidance cycle.

    Returns:
        SimulationResult containing recorded trajectories and performance metrics.
    """
    law.reset()
    if autopilot is not None:
        autopilot.reset()

    max_accel = autopilot.max_accel if autopilot is not None else 50.0
    target = target0.copy()
    chaser = chaser0.copy()
    t = 0.0

    times, tgt_hist, chs_hist, acc_hist, cmd_hist = [], [], [], [], []
    min_range = float("inf")

    h = dt / substeps

    while True:
        rel_pos = target[:3] - chaser[:3]
        rel_vel = target[3:] - chaser[3:]
        current_range = float(np.linalg.norm(rel_pos))

        # Check termination criteria
        if current_range <= capture_radius or t >= t_max:
            break
        if stop_on_opening and current_range > min_range + 5.0 and min_range < 500.0:
            # Terminate if opening after an intercept CPA
            break
        if current_range < min_range:
            min_range = current_range

        # Target maneuver acceleration
        a_target = target_accel_fn(t, target) if target_accel_fn is not None else np.zeros(3)

        # Guidance state and command
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
        t=np.array(times),
        target=np.array(tgt_hist),
        chaser=np.array(chs_hist),
        accel=np.array(acc_hist),
        cmd_accel=np.array(cmd_hist),
        max_accel=max_accel,
        events=events,
    )
