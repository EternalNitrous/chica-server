#!/usr/bin/env python3
"""Compare captured original CHICA_GAIT/CHICA_SERVO traces against the native port."""

from __future__ import annotations

import argparse
import ast
import json
import pathlib
import re
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[2]
PROBE = pathlib.Path("/tmp/chica_gait_probe")
NUMBER_RE = re.compile(r"[-+]?\d*\.\d+(?:e[-+]?\d+)?|[-+]?\d+(?:e[-+]?\d+)?", re.I)
PULSES_RE = re.compile(r"pulses=(\[.*\])")


def build_probe() -> None:
    command = [
        "c++",
        "-std=c++17",
        "-Iapp/src/main/cpp",
        "tools/oracle/gait_probe.cpp",
        "app/src/main/cpp/apk_model.cpp",
        "app/src/main/cpp/pulse_conversion.cpp",
        "-o",
        str(PROBE),
    ]
    subprocess.run(command, cwd=ROOT, check=True)


def parse_walk_target(command: str, crab_mode: bool) -> list[float] | None:
    if not command.startswith("walk") or command.startswith("walkclear"):
        return None
    if ":" not in command:
        return None
    values = [float(value) for value in NUMBER_RE.findall(command.split(":", 1)[1])]
    if len(values) < 3:
        return None
    if crab_mode:
        return [values[1], values[0], 0.0]
    return [values[1], 0.0, values[0]]


def active_leg_complement(disabled_pair: tuple[int, int]) -> list[int]:
    disabled = set(disabled_pair)
    return [leg for leg in (5, 2, 1, 0, 3, 4) if leg not in disabled]


def parse_quad_pair(command: str) -> tuple[int, int] | None:
    if not command.startswith("quad:"):
        return None
    parts = command.split(":", 1)[1].split(",")
    if len(parts) < 2:
        return None
    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        return None


def parse_trace(path: pathlib.Path) -> tuple[list[tuple], list[list[int]], list[int] | None, tuple[int, int] | None]:
    frames: list[tuple] = []
    servos: list[list[int]] = []
    target: list[float] | None = None
    filtered = [0.0, 0.0, 0.0]
    walking = False
    crab_mode = False
    seed_pulses: list[int] | None = None
    latest_servo: list[int] | None = None
    quad_pair: tuple[int, int] | None = None

    for line in path.read_text().splitlines():
        if not line:
            continue
        record = json.loads(line)
        if record.get("type") == "servo_values":
            values = record["values"]
            if not walking:
                latest_servo = values
            elif not servos or servos[-1] != values:
                servos.append(values)
            continue
        if record.get("type") == "device_command":
            command = str(record.get("command", ""))
            if command.startswith("crab"):
                crab_mode = not crab_mode
            next_quad = parse_quad_pair(command)
            if next_quad is not None:
                quad_pair = next_quad
            next_target = parse_walk_target(command, crab_mode)
            if next_target is not None:
                target = next_target
                filtered = [0.0, 0.0, 0.0]
                walking = True
                seed_pulses = latest_servo
            continue
        if not walking:
            continue
        if record.get("type") == "gait" and target is not None:
            if record.get("allow"):
                filtered = [
                    current + ((desired - current) * 0.05)
                    for current, desired in zip(filtered, target)
                ]
            frames.append((
                int(record["gait"]),
                int(record["style"]),
                float(record["dt"]),
                1 if record.get("allow") else 0,
                filtered[0],
                filtered[1],
                filtered[2],
            ))

    return frames, servos, seed_pulses, quad_pair


def run_probe(frames: list[tuple], seed_pulses: list[int] | None, quad_pair: tuple[int, int] | None) -> list[list[int]]:
    command = [str(PROBE)]
    if quad_pair is not None and seed_pulses is not None:
        command += ["--seed-pulses", ",".join(str(value) for value in seed_pulses)]
    if quad_pair is not None:
        active = active_leg_complement(quad_pair)
        command += ["--body-z", "45", "--quad-active", ",".join(str(leg) for leg in active)]
    for frame in frames:
        command += ["--frame", ",".join(str(value) for value in frame)]
    proc = subprocess.run(command, text=True, capture_output=True, check=True)
    pulses: list[list[int]] = []
    for line in proc.stdout.splitlines():
        match = PULSES_RE.search(line)
        if match:
            pulses.append(ast.literal_eval(match.group(1)))
    return pulses


def compare_file(path: pathlib.Path) -> bool:
    frames, servos, seed_pulses, quad_pair = parse_trace(path)
    if not frames:
        print(f"{path.name}: no gait frames")
        return False
    probe = run_probe(frames, seed_pulses, quad_pair)
    if len(servos) != len(probe):
        print(f"{path.name}: frame count mismatch: original={len(servos)} rebuilt={len(probe)}")
        return False
    exact = all(a == b for a, b in zip(servos, probe))
    print(f"{path.name}: frames={len(frames)} exact={exact}")
    for index, (original, rebuilt) in enumerate(zip(servos, probe)):
        if original == rebuilt:
            continue
        diffs = [abs(a - b) for a, b in zip(original, rebuilt)]
        print(f"  frame {index}: max={max(diffs)} sum={sum(diffs)}")
        print(f"    original={original}")
        print(f"    rebuilt ={rebuilt}")
    return exact


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", nargs="+", type=pathlib.Path)
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()

    if not args.skip_build:
        build_probe()

    ok = True
    for trace in args.trace:
        ok = compare_file(trace) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
