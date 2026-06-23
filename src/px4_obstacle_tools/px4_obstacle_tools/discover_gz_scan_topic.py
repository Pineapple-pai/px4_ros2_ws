#!/usr/bin/env python3
"""
Discover active Gazebo Classic scan/lidar topics by probing ``gz topic -l``.

Usage (standalone)::

    python3 discover_gz_scan_topic.py

Or import and call ``find_scan_topics()``.

Returns a list of topic strings sorted by publisher count (most publishers first).
"""

import subprocess
import sys


def find_scan_topics(gz_bin: str = "gz") -> list[str]:
    """Return all topics matching scan/laser/lidar.

    NOTE: PX4's gazebo_lidar_plugin publishes via transport::Node
    which does NOT register as a named Publisher visible via
    ``gz topic -i``.  Therefore we return ALL lidar-related topics
    without filtering by publisher count.
    """
    try:
        result = subprocess.run(
            [gz_bin, "topic", "-l"],
            capture_output=True, text=True, timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
        print(f"ERROR: cannot list gz topics: {e}", file=sys.stderr)
        return []

    all_topics = [t.strip() for t in result.stdout.splitlines() if t.strip()]
    candidates = [t for t in all_topics if any(kw in t.lower()
                  for kw in ("scan", "laser", "lidar", "rplidar"))]
    # Sort alphabetically for deterministic ordering
    return sorted(candidates)


def main() -> None:
    topics = find_scan_topics()
    if not topics:
        print("No active scan/laser/lidar topics found in Gazebo.", file=sys.stderr)
        sys.exit(1)
    for t in topics:
        print(t)


if __name__ == "__main__":
    main()