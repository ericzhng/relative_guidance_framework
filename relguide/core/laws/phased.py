"""Multi-phase guidance handover manager."""

from __future__ import annotations

from typing import Any

from relguide.core.base import GuidanceLaw, GuidanceState, Vec


class PhasedGuidance(GuidanceLaw):
    """Executes a midcourse guidance law, then latches to a terminal guidance law.

    The handover occurs once the range drops below ``switch_range``. The switch
    is one-way to prevent chattering.

    Args:
        midcourse: Law active in the midcourse phase.
        terminal: Law active in the terminal phase.
        switch_range: Range threshold for handover [m].
    """

    def __init__(
        self, midcourse: GuidanceLaw, terminal: GuidanceLaw, switch_range: float = 1000.0
    ) -> None:
        self.midcourse = midcourse
        self.terminal = terminal
        self.switch_range = switch_range
        self.name = f"{midcourse.name} + terminal"
        self.switch_time: float | None = None

    def reset(self, scenario: Any = None) -> None:
        """Reset handover state and child guidance laws."""
        self.switch_time = None
        try:
            self.midcourse.reset(scenario)
        except TypeError:
            self.midcourse.reset()
        try:
            self.terminal.reset(scenario)
        except TypeError:
            self.terminal.reset()

    def command(self, state: GuidanceState) -> Vec:
        """Delegate command to the active phase law."""
        if self.switch_time is None and state.range <= self.switch_range:
            self.switch_time = state.t
        active = self.midcourse if self.switch_time is None else self.terminal
        return active.command(state)
