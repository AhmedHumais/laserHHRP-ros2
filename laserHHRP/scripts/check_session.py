#!/usr/bin/env python3
"""Checks the last session recorded by a simulated scan (see timed_scan.py).

    check_session.py ~/laser_hhrp/scans

Frames present, positions never going back, IMU valid, and the frames that show the simulated
rivet head (20 um proud, at 20 mm, radius 2.5 mm) are at the rivet.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from read_session import read_frames  # noqa: E402


def check(scans: Path) -> list[str]:
    sessions = sorted(p for p in scans.iterdir() if (p / "frames.bin").exists())
    if not sessions:
        return [f"{scans}: no recorded session"]
    frames = list(read_frames(sessions[-1]))
    errors = []
    if len(frames) < 100:
        errors.append(f"only {len(frames)} frames")
    for a, b in zip(frames, frames[1:]):
        if b.position_m < a.position_m - 1e-9:
            errors.append(f"position goes back at frame {b.index}")
            break
    with_imu = sum(1 for f in frames if f.imu_valid)
    if with_imu < len(frames) - 5:
        errors.append(f"IMU valid in only {with_imu}/{len(frames)} frames")
    over_rivet = 0
    for f in frames:
        z = f.z_m
        if len(z) and abs((z[0] - z[len(z) // 2]) - 20e-6) < 1e-6:
            over_rivet += 1
            if abs(f.position_m - 0.020) > 0.0025 + 0.0004:
                errors.append(f"rivet seen at {f.position_m * 1e3:.2f} mm (frame {f.index})")
                break
    if over_rivet == 0:
        errors.append("rivet not seen")
    print(f"{sessions[-1].name}: {len(frames)} frames, {over_rivet} over the rivet, {with_imu} with IMU")
    return errors


if __name__ == "__main__":
    problems = check(Path(sys.argv[1] if len(sys.argv) > 1 else "~/laser_hhrp/scans").expanduser())
    for p in problems:
        print("FAIL:", p, file=sys.stderr)
    sys.exit(1 if problems else 0)
