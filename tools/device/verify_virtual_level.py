#!/usr/bin/env python3
"""Verify level mode with injected virtual orientation through TCP/logcat."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_APK = ROOT / "app" / "build" / "outputs" / "apk" / "debug" / "app-debug.apk"
FLAGS_RE = re.compile(r"FLAGS=(\S+)")


STEPS = [
    "virtualtouch:0.5",
    "torque:0.5",
    "sit:1.5",
    "virtualorientation:0.700,-0.500,0.000:0.2",
    "level:2.0",
    "level:2.0",
]


def capture(apk: Path, out: Path) -> None:
    command = [
        sys.executable,
        str(ROOT / "tools" / "capture" / "capture_control_logcat.py"),
        "--out",
        str(out),
        "--apk",
        str(apk),
        "--package",
        "com.makeyourpet.chicaserver",
    ]
    for step in STEPS:
        command += ["--step", step]
    subprocess.run(command, cwd=ROOT, check=True)


def load(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def flags(rows: list[dict]) -> list[str]:
    out: list[str] = []
    for row in rows:
        if row.get("type") != "tcp":
            continue
        match = FLAGS_RE.search(row.get("line", ""))
        if not match:
            continue
        value = match.group(1)
        if not out or out[-1] != value:
            out.append(value)
    return out


def command_times(rows: list[dict], command: str) -> list[float]:
    times: list[float] = []
    for row in rows:
        if (row.get("type") == "control"
                and row.get("event") == "tcp_recv"
                and row.get("command") == command):
            times.append(float(row.get("time", 0.0)))
    if not times:
        raise AssertionError(f"missing command marker: {command}")
    return times


def servo_values(rows: list[dict], start: float, end: float | None = None) -> list[tuple[int, ...]]:
    out: list[tuple[int, ...]] = []
    for row in rows:
        if row.get("type") != "servo_values":
            continue
        timestamp = float(row.get("time", 0.0))
        if timestamp < start:
            continue
        if end is not None and timestamp >= end:
            continue
        out.append(tuple(row.get("values", [])))
    return out


def max_delta(frames: list[tuple[int, ...]]) -> int:
    if len(frames) < 2:
        return 0
    first = frames[0]
    return max(max(abs(a - b) for a, b in zip(first, frame)) for frame in frames[1:])


def max_delta_from(reference: tuple[int, ...], frames: list[tuple[int, ...]]) -> int:
    if not frames:
        return 0
    return max(max(abs(a - b) for a, b in zip(reference, frame)) for frame in frames)


def verify(path: Path) -> None:
    rows = load(path)
    path_flags = flags(rows)
    if "110001100" not in path_flags:
        raise AssertionError(f"level flag never enabled: {path_flags}")
    if path_flags[-1] != "110000100":
        raise AssertionError(f"level flag did not clear after second level command: {path_flags}")

    level_times = command_times(rows, "level")
    if len(level_times) < 2:
        raise AssertionError(f"expected two level commands, got {len(level_times)}")
    level_on, level_off = level_times[0], level_times[1]
    pre_frames = servo_values(rows, 0.0, level_on)
    active_frames = servo_values(rows, level_on, level_off)
    decay_frames = servo_values(rows, level_off, None)
    if not pre_frames:
        raise AssertionError("no pre-level servo frame found")
    active_delta = max_delta_from(pre_frames[-1], active_frames)
    decay_delta = max_delta(decay_frames)
    if active_delta < 25:
        raise AssertionError("injected orientation produced no level servo motion")
    if decay_delta <= 0:
        raise AssertionError("level decay produced no servo motion after disable")
    print(f"virtual_level flags={path_flags} active_delta={active_delta} decay_delta={decay_delta}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apk", type=Path, default=DEFAULT_APK)
    parser.add_argument("--out", type=Path, default=Path("/tmp/chica-virtual-level/rebuilt_level.jsonl"))
    args = parser.parse_args()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    capture(args.apk, args.out)
    verify(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
