from __future__ import annotations

import threading
from typing import Literal

SlamState = Literal["mapping", "localization", "calibrating"]

_slam_state_lock = threading.Lock()
_slam_state: SlamState = "mapping"


def get_slam_state() -> SlamState:
    with _slam_state_lock:
        return _slam_state


def set_slam_state(state: SlamState) -> None:
    global _slam_state
    with _slam_state_lock:
        _slam_state = state

