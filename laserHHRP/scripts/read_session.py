#!/usr/bin/env python3
"""Reads a recorded scan session (folder written by recorder_node).

    read_session.py ~/laser_hhrp/scans/20261011-190838        prints a summary

In Python:
    from read_session import read_frames
    for f in read_frames(folder):      # laser_hhrp/msg/ScanFrame
        f.position_m, f.x_m, f.z_m, f.accel_mps2
"""

from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message

from laser_hhrp.msg import ScanFrame


def read_frames(session: str | Path):
    """Yields every ScanFrame of frames.bin ([uint32 length][serialized ScanFrame] ...)."""
    with open(Path(session) / "frames.bin", "rb") as f:
        while True:
            header = f.read(4)
            if len(header) < 4:
                return
            (length,) = struct.unpack("<I", header)
            yield deserialize_message(f.read(length), ScanFrame)


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    session = Path(sys.argv[1]).expanduser()
    info = json.loads((session / "session.json").read_text())
    frames = list(read_frames(session))
    print(f"{session}: {info['start']} .. {info['stop']}, {len(frames)} frames")
    if frames:
        print(f"  position {frames[0].position_m * 1e3:.2f} .. {frames[-1].position_m * 1e3:.2f} mm, "
              f"{len(frames[0].x_m)} points per profile")
    print(f"  summary: {info['summary']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
