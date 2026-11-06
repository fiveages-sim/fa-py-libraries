"""``python -m xr_hand_retarget.tools.build_o6_cspace --side both``"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    # Delegate to repo tools/ script (same tree when editable install).
    script = Path(__file__).resolve().parents[2] / "tools" / "build_o6_cspace.py"
    if not script.is_file():
        raise SystemExit(f"build script not found: {script}")
    if argv is not None:
        sys.argv = [str(script), *argv]
    else:
        sys.argv[0] = str(script)
    runpy.run_path(str(script), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
