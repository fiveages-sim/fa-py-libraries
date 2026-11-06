"""Allow ``python -m xr_hand_retarget`` from the fa-py-libraries repo root.

The project folder and the inner package share the same name. Running from the
repo without letting the inner package win would load this directory as a
namespace package with no ``__main__``.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
root = str(_ROOT)
if sys.path[:1] != [root]:
    sys.path.insert(0, root)

# Drop the namespace package created from this directory.
sys.modules.pop("xr_hand_retarget", None)
for name in list(sys.modules):
    if name.startswith("xr_hand_retarget."):
        del sys.modules[name]

runpy.run_module("xr_hand_retarget.node", run_name="__main__")
