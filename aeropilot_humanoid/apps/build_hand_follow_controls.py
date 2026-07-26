"""Build the hand-following MuJoCo cockpit XML and write its report."""

from __future__ import annotations

import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from aeropilot_humanoid.hand_follow_controller import (  # noqa: E402
    simulate_hand_follow_sequence,
)
from aeropilot_humanoid.vendor_audit import write_json  # noqa: E402


def main() -> None:
    result = simulate_hand_follow_sequence(PROJECT_ROOT)
    write_json(PROJECT_ROOT / "reports/hand_follow_controls_report.json", result)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
