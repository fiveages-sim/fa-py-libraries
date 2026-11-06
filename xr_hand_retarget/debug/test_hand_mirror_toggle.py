#!/usr/bin/env python3
"""Hand mirror latch helpers (no rclpy)."""

from __future__ import annotations

from xr_hand_retarget.mirror_latch import (
    MIRROR_DEBOUNCE_S,
    apply_mirror_toggle,
    mirror_for_controller,
    optical_to_publish_side,
)


def test_debounce_blocks_double_toggle() -> None:
    on, until, ok = apply_mirror_toggle(mirrored=False, skip_until=0.0, now=10.0)
    assert ok and on is True
    on2, until2, ok2 = apply_mirror_toggle(
        mirrored=on, skip_until=until, now=10.0 + MIRROR_DEBOUNCE_S * 0.5
    )
    assert ok2 is False and on2 is True and until2 == until
    on3, _, ok3 = apply_mirror_toggle(
        mirrored=on, skip_until=until, now=until + 1e-6
    )
    assert ok3 and on3 is False
    print("  debounce blocks double toggle")


def test_optical_publish_routing() -> None:
    assert optical_to_publish_side("left", False) == "left"
    assert optical_to_publish_side("right", False) == "right"
    assert optical_to_publish_side("left", True) == "right"
    assert optical_to_publish_side("right", True) == "left"
    print("  optical publish routing swaps when mirrored")


def test_mirror_latch_requires_same_controller() -> None:
    assert mirror_for_controller({"mirror": True, "pid": 1, "start": 2}, None) is False
    assert mirror_for_controller({"mirror": True, "pid": 1, "start": 2}, (1, 2)) is True
    assert mirror_for_controller({"mirror": True, "pid": 1, "start": 2}, (1, 9)) is False
    print("  mirror latch requires same controller")


def main() -> None:
    test_debounce_blocks_double_toggle()
    test_optical_publish_routing()
    test_mirror_latch_requires_same_controller()
    print("all hand mirror toggle OK")


if __name__ == "__main__":
    main()
