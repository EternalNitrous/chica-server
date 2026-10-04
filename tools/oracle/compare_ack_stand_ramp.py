#!/usr/bin/env python3
"""Compare original and rebuilt standing ACK-ramp traces."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT.parents[1]
ORACLE_DIR = Path(os.environ.get("CHICA_ORACLE_DIR", WORKSPACE / "research-private" / "oracle"))
DEFAULT_ORIGINAL = ORACLE_DIR / "controltrace_ack_stand_ramp_virtualtouch_original.jsonl"
DEFAULT_REBUILT = ORACLE_DIR / "controltrace_ack_stand_ramp_virtualtouch_rebuilt.jsonl"
EXPECTED_STANDING_VALUES = [float(value) for value in range(49, 19, -1)]


def load(path: Path) -> list[dict]:
    records: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            records.append(json.loads(line))
    return records


def standing_ack_values(records: list[dict]) -> list[float]:
    return [
        float(record["value"])
        for record in records
        if record.get("type") == "control"
        and record.get("event") == "ack_ramp"
        and record.get("state", {}).get("stand") is True
    ]


def marker_time(records: list[dict], message: str) -> float:
    return max(
        [
            float(record.get("time", 0.0))
            for record in records
            if record.get("type") == "mark" and record.get("message") == message
        ]
        or [0.0]
    )


def post_sit_methods(records: list[dict]) -> set[tuple[str, float]]:
    after_sit = marker_time(records, "send:sit")
    return {
        (str(record.get("method")), float(record.get("duration", 0.0)))
        for record in records
        if record.get("type") == "anim" and float(record.get("time", 0.0)) > after_sit
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("original", nargs="?", type=Path, default=DEFAULT_ORIGINAL)
    parser.add_argument("rebuilt", nargs="?", type=Path, default=DEFAULT_REBUILT)
    args = parser.parse_args()

    original = load(args.original)
    rebuilt = load(args.rebuilt)
    original_values = standing_ack_values(original)
    rebuilt_values = standing_ack_values(rebuilt)
    original_methods = post_sit_methods(original)
    rebuilt_methods = post_sit_methods(rebuilt)

    print(f"standing_ack_values_original={original_values}")
    print(f"standing_ack_values_rebuilt={rebuilt_values}")
    print(f"post_sit_anim_methods_original={sorted(original_methods)}")
    print(f"post_sit_anim_methods_rebuilt={sorted(rebuilt_methods)}")

    ok = True
    if original_values != EXPECTED_STANDING_VALUES:
        print("original standing ACK values differ from expected 49..20", file=sys.stderr)
        ok = False
    if rebuilt_values != EXPECTED_STANDING_VALUES:
        print("rebuilt standing ACK values differ from expected 49..20", file=sys.stderr)
        ok = False
    if original_values != rebuilt_values:
        print("standing ACK values differ", file=sys.stderr)
        ok = False
    if any(method == "M" for method, _ in rebuilt_methods):
        print("rebuilt emitted ACK pose-ramp animations after sit", file=sys.stderr)
        ok = False
    if any(method == "M" for method, _ in original_methods):
        print("original emitted ACK pose-ramp animations after sit", file=sys.stderr)
        ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
