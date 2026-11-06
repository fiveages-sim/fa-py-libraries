"""LinkerHand O6 / L6 / O7 backend (family bind)."""

from __future__ import annotations

from xr_hand_retarget.algorithms.curl_o6 import O6Backend

__all__ = ["O6Backend", "LinkerBackend"]

# Alias for make_backend / docs.
LinkerBackend = O6Backend
