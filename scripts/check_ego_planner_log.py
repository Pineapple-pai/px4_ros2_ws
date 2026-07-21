#!/usr/bin/env python3
"""Fail a regression run when Ego-Planner emits known unsafe planning diagnostics."""

import argparse
import pathlib
import re
import sys


PATTERNS = {
    "drone_in_obstacle": re.compile(r"drone is in obstacle", re.IGNORECASE),
    "initial_control_points_in_obstacle": re.compile(r"First 3 control points in obstacles", re.IGNORECASE),
    "astar_pool_exhausted": re.compile(r"Ran out of pool", re.IGNORECASE),
    "invalid_zero_target_adjustment": re.compile(r"Adjusted local target point from\s*\[0(?:\.0+)?,\s*0(?:\.0+)?,\s*0(?:\.0+)?\]", re.IGNORECASE),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", help="Planner/validation log files to inspect")
    parser.add_argument("--start-line", type=int, default=1, help="Ignore lines before this 1-based line number")
    parser.add_argument("--context", type=int, default=1, help="Context lines printed around each match")
    args = parser.parse_args()

    total = 0
    counts = {name: 0 for name in PATTERNS}
    for raw_path in args.logs:
        path = pathlib.Path(raw_path)
        if not path.exists():
            print(f"LOG_SCAN_MISSING {path}")
            return 2
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        first = max(0, args.start_line - 1)
        for index in range(first, len(lines)):
            matched = [name for name, pattern in PATTERNS.items() if pattern.search(lines[index])]
            if not matched:
                continue
            total += 1
            for name in matched:
                counts[name] += 1
            low = max(first, index - args.context)
            high = min(len(lines), index + args.context + 1)
            print(f"LOG_HAZARD file={path} line={index + 1} kinds={','.join(matched)}")
            for context_index in range(low, high):
                print(f"  {context_index + 1}: {lines[context_index]}")

    print("EGO_LOG_SCAN " + " ".join(f"{name}={count}" for name, count in counts.items()))
    print(f"EGO_LOG_SCAN_RESULT passed={total == 0} hazards={total}")
    return 0 if total == 0 else 3


if __name__ == "__main__":
    sys.exit(main())