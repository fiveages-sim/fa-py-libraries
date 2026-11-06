"""FSM gating for robot commands (teleop_ws FsmCommandGate)."""

from __future__ import annotations


class FsmCommandGate:
    """Fail closed; require resync after the gate re-opens."""

    def __init__(self, allowed_states):
        self.allowed_states = frozenset(int(state) for state in allowed_states)
        if not self.allowed_states:
            raise ValueError("allowed_fsm_states must not be empty")
        self.state = None
        self.needs_resync = False

    @property
    def is_open(self) -> bool:
        return self.state in self.allowed_states

    @property
    def can_publish(self) -> bool:
        return self.is_open and not self.needs_resync

    def update(self, state: int) -> tuple[bool, bool]:
        was_open = self.is_open
        self.state = int(state)
        is_open = self.is_open
        if is_open and not was_open:
            self.needs_resync = True
        return was_open, is_open

    def mark_resynced(self) -> None:
        if self.is_open:
            self.needs_resync = False
