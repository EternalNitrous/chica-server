#!/usr/bin/env python3
"""Replay CHICA_GT APK poses through the native IK and PWM conversion path."""

from __future__ import annotations

import argparse
import ast
import contextlib
import io
import json
import os
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT.parents[1]
CAPTURE_DIR = Path(os.environ.get("CHICA_CAPTURE_DIR", WORKSPACE / "research-private" / "server-apk-analysis"))
PROBE = ROOT / "build" / "gt_kinematics_probe"
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
SEQ_RE = re.compile(r"^sq=(\d+)\s+ph=(" + NUMBER + r")\s+")
VEC_RE = re.compile(r"xyz=\[([^\]]+)\],\s*uvw=\[([^\]]+)\]")
FEET_RE = re.compile(r"\b([LR][123])=\[\s*(" + NUMBER + r"),\s*(" + NUMBER + r"),\s*(" + NUMBER + r")\s*\]")
PULSE_RE = re.compile(r"^sq=(\d+)\s+pu=\[([^\]]+)\]")
LEG_INDEX = {"L1": 0, "L2": 1, "L3": 2, "R1": 3, "R2": 4, "R3": 5}


def parse_vec(text: str) -> list[float]:
    values = [float(value) for value in re.findall(NUMBER, text)]
    if len(values) != 3:
        raise ValueError(f"expected a 3-value vector, got {text!r}")
    return values


def parse_vec_with_uncertainty(text: str) -> tuple[list[float], list[float]]:
    tokens = re.findall(NUMBER, text)
    if len(tokens) != 3:
        raise ValueError(f"expected a 3-value vector, got {text!r}")
    values = [float(token) for token in tokens]
    uncertainties = []
    for token in tokens:
        mantissa, _, exponent_text = token.lower().partition("e")
        decimals = len(mantissa.partition(".")[2])
        exponent = int(exponent_text) if exponent_text else 0
        uncertainties.append(0.5 * (10.0 ** (exponent - decimals)))
    return values, uncertainties


def parse_capture(path: Path) -> tuple[list[dict], dict[str, int]]:
    frames: dict[int, dict] = {}
    pose_ids: set[int] = set()
    pulse_ids: set[int] = set()
    duplicate_pose_sequences: set[int] = set()
    duplicate_pulse_sequences: set[int] = set()
    counts = {"pose_headers": 0, "pulse_rows": 0, "duplicate_pose_ids": 0, "duplicate_pulse_ids": 0}

    for line_number, raw in enumerate(path.read_text(errors="replace").splitlines(), 1):
        try:
            record = json.loads(raw)
        except json.JSONDecodeError as error:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {error}") from error
        if record.get("type") != "log" or record.get("tag") != "CHICA_GT":
            continue
        message = str(record.get("message", ""))

        pose_match = SEQ_RE.match(message)
        if pose_match and "layer=" in message and "body=" in message:
            sequence = int(pose_match.group(1))
            counts["pose_headers"] += 1
            if sequence in pose_ids:
                counts["duplicate_pose_ids"] += 1
                duplicate_pose_sequences.add(sequence)
            pose_ids.add(sequence)
            layer_match = re.search(r"layer=\[" + VEC_RE.pattern + r"\]", message)
            body_match = re.search(r"body=\[" + VEC_RE.pattern + r"\]", message)
            if layer_match is None or body_match is None:
                raise ValueError(f"{path}:{line_number}: malformed pose header for sq={sequence}")
            body_xyz, body_xyz_uncertainty = parse_vec_with_uncertainty(body_match.group(1))
            body_uvw, body_uvw_uncertainty = parse_vec_with_uncertainty(body_match.group(2))
            layer_xyz, layer_xyz_uncertainty = parse_vec_with_uncertainty(layer_match.group(1))
            layer_uvw, layer_uvw_uncertainty = parse_vec_with_uncertainty(layer_match.group(2))
            frame = {
                "seq": sequence,
                "time": float(record.get("time", 0.0)),
                "phase": float(pose_match.group(2)),
                "body": body_xyz + body_uvw,
                "layer": layer_xyz + layer_uvw,
                "pose_uncertainty": (
                    body_xyz_uncertainty + body_uvw_uncertainty
                    + layer_xyz_uncertainty + layer_uvw_uncertainty
                ),
                "feet": [None] * 6,
                "feet_uncertainty": [None] * 6,
            }
            existing = frames.get(sequence)
            if existing is not None:
                existing.update(frame)
            else:
                frames[sequence] = frame
            continue

        pulse_match = PULSE_RE.match(message)
        if pulse_match:
            sequence = int(pulse_match.group(1))
            counts["pulse_rows"] += 1
            if sequence in pulse_ids:
                counts["duplicate_pulse_ids"] += 1
                duplicate_pulse_sequences.add(sequence)
            pulse_ids.add(sequence)
            frame = frames.setdefault(sequence, {"seq": sequence, "feet": [None] * 6})
            frame["pulses"] = [int(value) for value in re.findall(r"[-+]?\d+", pulse_match.group(2))]
            continue

        for match in FEET_RE.finditer(message):
            if not frames:
                continue
            sequence = max(frames)
            frame = frames[sequence]
            leg = LEG_INDEX[match.group(1)]
            values = [match.group(i) for i in (2, 3, 4)]
            parsed, uncertainty = parse_vec_with_uncertainty(",".join(values))
            frame["feet"][leg] = parsed
            frame["feet_uncertainty"][leg] = uncertainty

        if frames and "active=" in message and "angles=" in message:
            frame = frames[max(frames)]
            active = re.search(r"active=\[([^\]]+)\]", message)
            angles = re.search(r"angles=(\[.*\])$", message)
            if active is None or angles is None:
                raise ValueError(f"{path}:{line_number}: malformed enabled-leg/angle metadata")
            frame["active"] = [value.strip() == "true" for value in active[1].split(",")]
            frame["angles"] = ast.literal_eval(angles[1])
            if len(frame["active"]) != 6 or len(frame["angles"]) != 6 or any(len(row) != 3 for row in frame["angles"]):
                raise ValueError(f"{path}:{line_number}: wrong enabled-leg/angle dimensions")

    complete: list[dict] = []
    missing_pose = missing_pulses = missing_feet = bad_pulse_width = 0
    for sequence in sorted(frames):
        frame = frames[sequence]
        if "body" not in frame or "layer" not in frame or "phase" not in frame:
            missing_pose += 1
            continue
        if "pulses" not in frame:
            missing_pulses += 1
            continue
        if len(frame["pulses"]) != 18:
            bad_pulse_width += 1
            continue
        if any(foot is None for foot in frame["feet"]):
            missing_feet += 1
            continue
        if any(value is None for value in frame["feet_uncertainty"]):
            missing_feet += 1
            continue
        frame["uncertainty"] = frame["pose_uncertainty"] + [
            value
            for leg_uncertainty in frame["feet_uncertainty"]
            for value in leg_uncertainty
        ]
        frame["feet"] = [foot for foot in frame["feet"] if foot is not None]
        complete.append(frame)
    counts.update({
        "sequence_ids": len(frames),
        "complete_frames": len(complete),
        "missing_pose": missing_pose,
        "missing_pulses": missing_pulses,
        "missing_feet": missing_feet,
        "bad_pulse_width": bad_pulse_width,
        "duplicate_pose_sequence_ids": sorted(duplicate_pose_sequences),
        "duplicate_pulse_sequence_ids": sorted(duplicate_pulse_sequences),
    })
    return complete, counts


def compile_probe() -> None:
    PROBE.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "c++", "-std=c++17", "-O2", "-Iapp/src/main/cpp",
            "tools/oracle/gt_kinematics_probe.cpp",
            "app/src/main/cpp/apk_model.cpp",
            "app/src/main/cpp/pulse_conversion.cpp",
            "-o", str(PROBE),
        ],
        cwd=ROOT,
        check=True,
    )


def run_probe(frames: list[dict]) -> dict[int, tuple[bool, list[int]]]:
    rows: list[str] = []
    for frame in frames:
        values = [frame["seq"], *frame["body"], *frame["layer"], *[coord for foot in frame["feet"] for coord in foot]]
        if "active" in frame and "angles" in frame:
            values += [int(value) for value in frame["active"]]
            values += [angle for leg in frame["angles"] for angle in leg]
        rows.append(",".join(format(value, ".17g") if isinstance(value, float) else str(value) for value in values))
    result = subprocess.run(
        [str(PROBE)], input="\n".join(rows) + "\n", text=True,
        capture_output=True, cwd=ROOT, check=True,
    )
    output: dict[int, tuple[bool, list[int]]] = {}
    for line in result.stdout.splitlines():
        values = [int(value) for value in line.split(",")]
        if len(values) != 20:
            raise ValueError(f"native probe returned malformed row: {line!r}")
        output[values[0]] = (values[1] == 1, values[2:])
    return output


def quantization_pwm_bounds(
    frames: list[dict], predicted: dict[int, tuple[bool, list[int]]]
) -> tuple[dict[int, list[int]], set[int]]:
    """Estimate PWM uncertainty from each CHICA_GT value's printed precision.

    The probe is evaluated at each input's +/- half-last-digit endpoints. The
    per-coordinate PWM deltas are summed, giving a conservative first-order
    envelope for the rounded pose/foot values. This is only used to decide
    whether a small observed delta can be explained by log quantization; it
    never upgrades a nonzero delta to an exact pass.
    """
    variants: list[dict] = []
    owners: dict[int, tuple[int, int, float]] = {}
    next_sequence = 1_000_000_000
    for frame in frames:
        valid, center = predicted[frame["seq"]]
        if not valid or max(abs(a - b) for a, b in zip(frame["pulses"], center)) <= 1:
            continue
        values = frame["body"] + frame["layer"] + [
            coordinate for foot in frame["feet"] for coordinate in foot
        ]
        uncertainties = frame.get("uncertainty", [])
        if len(uncertainties) != len(values):
            continue
        for coordinate, uncertainty in enumerate(uncertainties):
            if uncertainty <= 0.0:
                continue
            for direction in (-1.0, 1.0):
                changed = values.copy()
                changed[coordinate] += direction * uncertainty
                variant = {
                    "seq": next_sequence,
                    "body": changed[:6],
                    "layer": changed[6:12],
                    "feet": [changed[12 + leg * 3:15 + leg * 3] for leg in range(6)],
                }
                for key in ("active", "angles"):
                    if key in frame:
                        variant[key] = frame[key]
                owners[next_sequence] = (frame["seq"], coordinate, direction)
                variants.append(variant)
                next_sequence += 1

    if not variants:
        return {}, set()
    results = run_probe(variants)
    bounds: dict[int, list[int]] = {}
    invalid: set[int] = set()
    for frame in frames:
        if any(owner[0] == frame["seq"] for owner in owners.values()):
            bounds[frame["seq"]] = [0] * 18
    for sequence, (frame_sequence, coordinate, direction) in owners.items():
        if direction > 0.0:
            continue
        first, second = results.get(sequence), results.get(sequence + 1)
        if first is None or second is None or not first[0] or not second[0]:
            invalid.add(frame_sequence)
            continue
        center = predicted[frame_sequence][1]
        channel_deltas = [
            max(abs(a - value), abs(b - value))
            for a, b, value in zip(first[1], second[1], center)
        ]
        bounds[frame_sequence] = [
            total + delta for total, delta in zip(bounds[frame_sequence], channel_deltas)
        ]
    for frame_sequence in bounds:
        bounds[frame_sequence] = [value + 1 for value in bounds[frame_sequence]]
    return bounds, invalid


def compare(path: Path) -> str:
    frames, counts = parse_capture(path)
    if not frames:
        print(f"{path.name}: status=BOUNDED no complete CHICA_GT pose/pulse frames; capture_counts={counts}")
        return "BOUNDED"
    # Legacy captures did not record the enabled mask or retained angles.
    # Replaying parked legs as enabled is invalid evidence, not an IK failure.
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    has_quad = any(
        row.get("type") == "command" and str(row.get("command", "")).startswith("quad")
        or row.get("type") == "tcp" and re.search(r"FLAGS=....4", str(row.get("line", "")))
        for row in rows
    )
    if has_quad and any("active" not in frame or "angles" not in frame for frame in frames):
        print(f"{path.name}: status=BOUNDED frames={len(frames)} quadruped capture lacks enabled-mask/retained-angle metadata; PWM replay unavailable")
        return "BOUNDED"
    predicted = run_probe(frames)
    ambiguous_sequences = set(counts["duplicate_pose_sequence_ids"])
    ambiguous_sequences.update(counts["duplicate_pulse_sequence_ids"])
    comparison_frames = [frame for frame in frames if frame["seq"] not in ambiguous_sequences]
    quantization_bounds, quantization_invalid = quantization_pwm_bounds(comparison_frames, predicted)
    mismatches: list[tuple[int, int, list[int], list[int]]] = []
    invalid: list[int] = []
    unexplained: list[tuple[int, int, int, int]] = []
    changed_channels = 0
    max_delta = 0
    total_delta = 0
    for frame in comparison_frames:
        valid, pulses = predicted[frame["seq"]]
        if not valid:
            invalid.append(frame["seq"])
        else:
            frame_deltas = [abs(a - b) for a, b in zip(frame["pulses"], pulses)]
            frame_max = max(frame_deltas)
            max_delta = max(max_delta, frame_max)
            total_delta += sum(frame_deltas)
            changed_channels += sum(delta != 0 for delta in frame_deltas)
            if frame_max:
                mismatches.append((frame["seq"], frame_max, frame["pulses"], pulses))
                frame_bounds = quantization_bounds.get(frame["seq"])
                if frame_bounds is not None:
                    unexplained.extend(
                        (frame["seq"], channel, delta, frame_bounds[channel])
                        for channel, delta in enumerate(frame_deltas)
                        if delta > frame_bounds[channel]
                    )

    complete_ok = (
        counts["complete_frames"] == counts["pose_headers"]
        and counts["missing_pose"] == 0
        and counts["missing_pulses"] == 0
        and counts["missing_feet"] == 0
        and counts["bad_pulse_width"] == 0
        and counts["duplicate_pose_ids"] == 0
        and counts["duplicate_pulse_ids"] == 0
    )
    unbounded = [
        frame["seq"] for frame in comparison_frames
        if max(abs(a - b) for a, b in zip(frame["pulses"], predicted[frame["seq"]][1])) > 1
        and frame["seq"] not in quantization_bounds
    ]
    if invalid or unexplained:
        status = "FAIL"
    elif complete_ok and max_delta == 0:
        status = "PASS"
    else:
        # Missing log records and PWM deltas explainable by displayed numeric
        # precision are evidence limits, not proof of an app mismatch.
        status = "BOUNDED"
    total_values = len(comparison_frames) * 18
    mean_delta = total_delta / total_values if total_values else 0.0
    print(
        f"{path.name}: status={status} frames={len(comparison_frames)} "
        f"ambiguous_duplicate_frames_skipped={len(ambiguous_sequences)} "
        f"complete_capture={complete_ok} differing_frames={len(mismatches)} "
        f"differing_channels={changed_channels}/{total_values} "
        f"max_pwm_delta={max_delta} mean_abs_pwm_delta={mean_delta:.4f}us"
    )
    if not complete_ok:
        print(f"  capture_counts={counts}")
    if quantization_bounds:
        print(
            f"  quantization_checked_frames={len(quantization_bounds)} "
            f"unexplained_channels={len(unexplained)} unbounded_frames={len(unbounded)} "
            f"invalid_uncertainty_frames={len(quantization_invalid)}"
        )
    for sequence in invalid[:3]:
        print(f"  sq={sequence}: native IK rejected the captured pose")
    for sequence, channel, delta, bound in unexplained[:5]:
        print(f"  sq={sequence} pin={channel}: pwm_delta={delta}us exceeds logged_precision_bound={bound}us")
    if max_delta > 1:
        max_examples = [item for item in mismatches if item[1] == max_delta]
        for sequence, delta, original, rebuilt in max_examples[:3]:
            changed = [
                (index, a, b)
                for index, (a, b) in enumerate(zip(original, rebuilt))
                if a != b
            ]
            print(f"  sq={sequence}: max_pwm_delta={delta} changed_pin_values={changed}")
    return status


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", nargs="*", type=Path, help="original APK CHICA_GT JSONL captures")
    parser.add_argument("--capture-dir", type=Path, default=CAPTURE_DIR)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument(
        "--summary", action="store_true",
        help="print aggregate results instead of one line per capture",
    )
    args = parser.parse_args()
    traces = args.traces or sorted(args.capture_dir.glob("gt_jsonl_*.jsonl"))
    if not traces:
        print(f"SKIP: no gt_jsonl_*.jsonl captures found in {args.capture_dir}")
        return 0
    if not args.skip_build:
        compile_probe()
    missing = [path for path in traces if not path.is_file()]
    if missing:
        for path in missing:
            print(f"FAIL: capture does not exist: {path}")
        return 1
    statuses: list[str] = []
    summaries: list[tuple[str, str, str]] = []
    for path in traces:
        if args.summary:
            captured = io.StringIO()
            with contextlib.redirect_stdout(captured):
                status = compare(path)
            lines = captured.getvalue().splitlines()
            headline = next((line for line in lines if "status=" in line), "")
            summaries.append((status, path.name, headline))
            statuses.append(status)
        else:
            statuses.append(compare(path))
    if args.summary:
        counts = {name: statuses.count(name) for name in ("PASS", "BOUNDED", "FAIL")}
        print(
            f"original pose-to-PWM captures={len(statuses)} exact={counts['PASS']} "
            f"bounded={counts['BOUNDED']} failed={counts['FAIL']}"
        )
        comparable = []
        for _, _, headline in summaries:
            match = re.search(r"max_pwm_delta=(\d+).*mean_abs_pwm_delta=([0-9.]+)us", headline)
            if match:
                comparable.append((int(match.group(1)), float(match.group(2))))
        if comparable:
            print(
                f"PWM replay over {len(comparable)} captures: "
                f"worst_delta={max(item[0] for item in comparable)}us "
                f"worst_mean_abs_delta={max(item[1] for item in comparable):.4f}us"
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
