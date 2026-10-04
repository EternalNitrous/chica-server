#!/usr/bin/env python3
"""Compare original Frida quad setxy traces against rebuilt logcat traces."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


def load(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def original_values(path: Path) -> tuple[list[list[int]], dict[str, int]]:
    records = load(path)
    start = next(
        record["hostTime"]
        for record in records
        if record.get("type") == "command" and record.get("command", "").startswith("setxy")
    )
    end = next(
        record["hostTime"]
        for record in records
        if record.get("type") == "command" and record.get("command", "").startswith("walkclear")
    )
    values: list[list[int]] = []
    steps: dict[str, int] = {}
    for record in records:
        host_time = record.get("hostTime", 0.0)
        if not (start <= host_time <= end):
            continue
        record_type = record.get("type")
        if record_type in ("set_pose_step", "pose_G_step", "pose_g_step"):
            steps[record_type] = steps.get(record_type, 0) + 1
        elif record_type == "servo_call":
            frame = record["values"]
            if not values or values[-1] != frame:
                values.append(frame)
    return values, steps


def rebuilt_values(path: Path) -> list[list[int]]:
    records = load(path)
    start = None
    end = None
    for record in records:
        if record.get("type") != "mark":
            continue
        message = record.get("message", "")
        if message.startswith("send:setxy:"):
            start = record["time"]
        elif start is not None and message.startswith("send:walkclear"):
            end = record["time"]
            break
    if start is None:
        raise ValueError(f"{path}: missing setxy mark")
    values: list[list[int]] = []
    for record in records:
        if record.get("type") != "servo_values":
            continue
        timestamp = record.get("time", 0.0)
        if timestamp < start or (end is not None and timestamp > end):
            continue
        frame = record["values"]
        if not values or values[-1] != frame:
            values.append(frame)
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    default_oracle = Path(os.environ.get(
        "CHICA_ORACLE_DIR",
        Path(__file__).resolve().parents[4] / "research-private" / "oracle",
    ))
    parser.add_argument("--oracle-dir", type=Path, default=default_oracle)
    parser.add_argument("--pairs", nargs="+", default=["03", "14", "25"])
    args = parser.parse_args()

    ok_all = True
    for pair in args.pairs:
        original_path = args.oracle_dir / f"frida_original_quad_{pair}_setxy_0_09_trace.jsonl"
        rebuilt_path = args.oracle_dir / f"marked_rebuilt_quad_{pair}_setxy_static_trace.jsonl"
        original, steps = original_values(original_path)
        rebuilt = rebuilt_values(rebuilt_path)
        ok = original == rebuilt
        ok_all = ok_all and ok
        print(
            f"{pair}: exact={ok} original_unique={len(original)} "
            f"rebuilt_unique={len(rebuilt)} motion_steps={steps}"
        )
        if not ok:
            for index in range(max(len(original), len(rebuilt))):
                expected = original[index] if index < len(original) else None
                actual = rebuilt[index] if index < len(rebuilt) else None
                print(f"  {index}: expected={expected} actual={actual}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
