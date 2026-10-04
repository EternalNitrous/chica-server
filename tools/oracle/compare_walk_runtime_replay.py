#!/usr/bin/env python3
"""Replay instrumented runtime gait logs through the native port and compare pulses.

The instrumented APK logs CHICA_GAIT immediately after the servo write for a
runtime walk frame. Pairing each gait record with the preceding CHICA_SERVO
record gives the pulse output for that frame without wall-clock alignment.
Replay uses the per-frame filtered command logged with CHICA_GAIT, so changes
between walk vectors and the stop ramp are preserved. That command is rounded
in the log, so a one-microsecond delta is reported as bounded rather than exact.
"""

from __future__ import annotations

import argparse
import ast
import contextlib
import io
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
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-Iapp/src/main/cpp",
            "tools/oracle/gait_probe.cpp",
            "app/src/main/cpp/apk_model.cpp",
            "app/src/main/cpp/pulse_conversion.cpp",
            "-o",
            str(PROBE),
        ],
        cwd=ROOT,
        check=True,
    )


def parse_vector(text: str) -> list[float]:
    values = [float(value) for value in NUMBER_RE.findall(text)]
    if len(values) != 3:
        raise ValueError(f"expected 3 values in gait cmd, got {text!r}")
    return values


def parse_trace(path: pathlib.Path) -> tuple[list[tuple], list[list[int]]]:
    records = [
        json.loads(line)
        for line in path.read_text().splitlines()
        if line.strip()
    ]
    records = [record for record in records if "time" in record]
    records.sort(key=lambda record: record["time"])

    frames: list[tuple] = []
    observed: list[list[int]] = []
    latest_servo: list[int] | None = None
    for record in records:
        record_type = record.get("type")
        if record_type == "servo_values":
            latest_servo = record["values"]
            continue
        if record_type != "gait":
            continue
        if latest_servo is None:
            raise ValueError(f"{path}: gait record before any servo output")
        printed = parse_vector(str(record.get("cmd", "")))
        forward, left, turn = printed
        frames.append((
            int(record["gait"]),
            int(record["style"]),
            float(record["dt"]),
            1 if record.get("allow") else 0,
            forward,
            left,
            turn,
        ))
        observed.append(latest_servo)
    return frames, observed


def run_probe(frames: list[tuple]) -> list[list[int]]:
    command = [str(PROBE)]
    for frame in frames:
        command += ["--frame", ",".join(str(value) for value in frame)]
    proc = subprocess.run(command, text=True, capture_output=True, check=True)
    pulses: list[list[int]] = []
    for line in proc.stdout.splitlines():
        match = PULSES_RE.search(line)
        if match:
            pulses.append(ast.literal_eval(match.group(1)))
    return pulses


def compare_file(path: pathlib.Path) -> str:
    frames, observed = parse_trace(path)
    if not frames:
        print(f"{path.name}: no runtime gait frames")
        return "FAIL"
    total_frames = len(frames)
    # A walk-only probe cannot replay sit/home/mode animations interleaved with
    # gait steps: those operations overwrite body/feet/layers in the live app.
    # Check the preceding uncontaminated prefix, and retain BOUNDED coverage
    # for the rest instead of diagnosing missing inputs as a gait mismatch.
    records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    gait_times = sorted(float(row["time"]) for row in records if row.get("type") == "gait")
    interleaved = [
        float(row["time"]) for row in records
        if row.get("type") == "anim" and "time" in row
        and gait_times[0] <= float(row["time"]) <= gait_times[-1]
    ]
    unmodeled_after = min(interleaved) if interleaved else None
    if unmodeled_after is not None:
        prefix_count = sum(time < unmodeled_after for time in gait_times)
        frames, observed = frames[:prefix_count], observed[:prefix_count]
        if not frames:
            print(f"{path.name}: status=BOUNDED input_frames={total_frames} replay_frames=0; concurrent pose animation is not represented in gait-only inputs")
            return "BOUNDED"
    expected = run_probe(frames)
    differing_frames = 0
    differing_channels = 0
    max_delta = 0
    first_diffs: list[tuple[int, int, int, int]] = []
    for index, (actual, rebuilt) in enumerate(zip(observed, expected)):
        channel_deltas = [abs(a - b) for a, b in zip(actual, rebuilt)]
        frame_max = max(channel_deltas)
        if frame_max == 0:
            continue
        differing_frames += 1
        differing_channels += sum(delta != 0 for delta in channel_deltas)
        max_delta = max(max_delta, frame_max)
        if len(first_diffs) < 5:
            channel = channel_deltas.index(frame_max)
            first_diffs.append((index, channel, actual[channel], rebuilt[channel]))
    if len(observed) != len(expected):
        status = "FAIL"
    elif differing_frames == 0:
        status = "BOUNDED" if unmodeled_after is not None else "PASS"
    elif max_delta <= 1:
        # The compact runtime trace rounds its command vector before logging it.
        # A one-microsecond delta cannot establish exact parity from that input.
        status = "BOUNDED"
    else:
        status = "FAIL"
    print(
        f"{path.name}: status={status} input_frames={total_frames} replay_frames={len(frames)} "
        f"app_outputs={len(observed)} replay_outputs={len(expected)} "
        f"differing_frames={differing_frames} differing_channels={differing_channels} "
        f"max_pwm_delta={max_delta}"
    )
    if unmodeled_after is not None:
        print("  gait prefix checked; remaining frames require the interleaved pose-animation inputs, unavailable in this replay format")
    for frame, channel, actual, rebuilt in first_diffs:
        print(f"  frame={frame} pin={channel}: app={actual} replay={rebuilt}")
    return status


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", nargs="+", type=pathlib.Path)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument(
        "--summary", action="store_true",
        help="print aggregate results instead of per-capture frame diagnostics",
    )
    args = parser.parse_args()

    if not args.skip_build:
        build_probe()

    statuses: list[str] = []
    summaries: list[tuple[str, str, str]] = []
    for trace in args.trace:
        if args.summary:
            captured = io.StringIO()
            with contextlib.redirect_stdout(captured):
                status = compare_file(trace)
            headline = next(
                (line for line in captured.getvalue().splitlines() if "status=" in line),
                "",
            )
            summaries.append((status, trace.name, headline))
            statuses.append(status)
        else:
            statuses.append(compare_file(trace))
    if args.summary:
        counts = {name: statuses.count(name) for name in ("PASS", "BOUNDED", "FAIL")}
        frame_counts = []
        max_deltas = []
        for _, _, headline in summaries:
            frames = re.search(r"input_frames=(\d+)", headline)
            delta = re.search(r"max_pwm_delta=(\d+)", headline)
            if frames:
                frame_counts.append(int(frames.group(1)))
            if delta:
                max_deltas.append(int(delta.group(1)))
        print(
            f"saved rebuild walk replay captures={len(statuses)} "
            f"exact={counts['PASS']} bounded={counts['BOUNDED']} "
            f"failed={counts['FAIL']} frames={sum(frame_counts)} "
            f"worst_pwm_delta={max(max_deltas, default=0)}us"
        )
        for status, name, headline in summaries:
            if status == "FAIL":
                print(f"FAIL {name}: {headline}")
    if "FAIL" in statuses:
        return 1
    if "BOUNDED" in statuses:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
