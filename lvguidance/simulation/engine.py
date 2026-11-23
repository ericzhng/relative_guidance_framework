"""Closed-loop simulation engine."""

from __future__ import annotations

import numpy as np

from lvguidance.dynamics.orbit import propagate_step
from lvguidance.guidance.base import GuidanceLaw, GuidanceState
from lvguidance.simulation.result import SimulationResult
from lvguidance.simulation.scenario import Scenario


def _saturate(vec: np.ndarray, limit: float) -> np.ndarray:
    """Scale ``vec`` down so that its norm does not exceed ``limit``."""
    norm = float(np.linalg.norm(vec))
    return vec if norm <= limit else vec * (limit / norm)


def simulate(law: GuidanceLaw, scenario: Scenario) -> SimulationResult:
    """Run a guidance law against an orbiting target.

    Both bodies are propagated under two-body gravity. The target is unpowered; the
    chaser applies the saturated guidance command (plus optional differential-gravity
    feed-forward) held constant over each guidance cycle. The run ends when the range
    drops below the capture radius or the simulation time limit is reached.

    Args:
        law: Guidance law under test.
        scenario: Initial conditions and limits.

    Returns:
        Time histories of the run.
    """
    law.reset()
    target, chaser = scenario.initial_states()
    t = 0.0
    times, tgt, chs, acc = [], [], [], []
    zero = np.zeros(3)

    while True:
        rel_pos = target[:3] - chaser[:3]
        rel_vel = target[3:] - chaser[3:]
        state = GuidanceState(t, rel_pos, rel_vel, scenario.t_final)
        u = law.command(state)
        if scenario.gravity_feedforward:
            u = u + scenario.differential_gravity(target[:3], chaser[:3])
        u = _saturate(u, scenario.max_accel)

        times.append(t)
        tgt.append(target.copy())
        chs.append(chaser.copy())
        acc.append(u)

        if np.linalg.norm(rel_pos) <= scenario.capture_radius or t >= scenario.t_max:
            break
        target = propagate_step(target, zero, scenario.dt, mu=scenario.mu)
        chaser = propagate_step(chaser, u, scenario.dt, mu=scenario.mu)
        t += scenario.dt

    switch_time = getattr(law, "switch_time", None)
    return SimulationResult(
        law_name=law.name,
        t=np.array(times),
        target=np.array(tgt),
        chaser=np.array(chs),
        accel=np.array(acc),
        max_accel=scenario.max_accel,
        events={} if switch_time is None else {"handover": float(switch_time)},
    )
