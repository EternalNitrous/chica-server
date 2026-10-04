#!/usr/bin/env python3
"""Compare original and rebuilt calibration traces captured with virtual touch inputs."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT.parents[1]
ORACLE_DIR = Path(os.environ.get("CHICA_ORACLE_DIR", WORKSPACE / "research-private" / "oracle"))
DEFAULT_ORIGINAL = ORACLE_DIR / "controltrace_calibrate_virtualtouch_original.jsonl"
DEFAULT_REBUILT = ORACLE_DIR / "controltrace_calibrate_virtualtouch_rebuilt.jsonl"


def calibration_frames(path: Path) -> list[list[int]]:
    frames: list[list[int]] = []
    in_calibration = False
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if record.get("type") == "mark" and record.get("message") == "send:calibrate":
                in_calibration = True
                frames.clear()
                continue
            if in_calibration and record.get("type") == "servo_values":
                values = record.get("values")
                if isinstance(values, list):
                    frames.append([int(value) for value in values])
    return frames


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("original", nargs="?", type=Path, default=DEFAULT_ORIGINAL)
    parser.add_argument("rebuilt", nargs="?", type=Path, default=DEFAULT_REBUILT)
    args = parser.parse_args()

    original = calibration_frames(args.original)
    rebuilt = calibration_frames(args.rebuilt)
    if not original:
        print(f"no original calibration frames found in {args.original}", file=sys.stderr)
        return 1
    if not rebuilt:
        print(f"no rebuilt calibration frames found in {args.rebuilt}", file=sys.stderr)
        return 1
    if len(original) != len(rebuilt):
        print(f"frame count mismatch: original={len(original)} rebuilt={len(rebuilt)}")
        return 1

    worst = 0
    worst_at: tuple[int, int, int, int] | None = None
    for frame_index, (left, right) in enumerate(zip(original, rebuilt)):
        if len(left) != len(right):
            print(
                f"pulse count mismatch at frame {frame_index}: "
                f"original={len(left)} rebuilt={len(right)}"
            )
            return 1
        for pulse_index, (a, b) in enumerate(zip(left, right)):
            delta = abs(a - b)
            if delta > worst:
                worst = delta
                worst_at = (frame_index, pulse_index, a, b)

    print(f"calibration virtual-touch frames={len(original)} worst_pulse_delta={worst}")
    if worst_at is not None:
        frame, pulse, a, b = worst_at
        print(f"worst at frame={frame} pulse={pulse} original={a} rebuilt={b}")
    return 0 if worst == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
