#!/usr/bin/env python3
"""Compare original APK and rebuild captures, segmented at every command.

The applications run in separate emulator sessions. Their frame clocks and
sampling rates are therefore not identical; output deltas are diagnostic and
cannot by themselves establish an exact runtime mismatch. Command and FLAGS
parity are checked exactly, and every captured command interval is summarized
separately so transition or stop-ramp differences remain visible. A second
comparison fits one constant timing offset per command interval, then compares
the interpolated pose and PWM trajectories. This distinguishes update/transport
jitter from a different motion path; it does not erase or certify command
latency.
"""

from __future__ import annotations

import argparse
import bisect
import contextlib
import io
import json
import math
import statistics
import sys
from pathlib import Path

from compare_gt_kinematics import parse_capture


FLAGS_MARKER = "FLAGS="
MOTION_PREFIXES = ("walk", "set", "quad", "bounce", "jump")


def load(path: Path) -> list[dict]:
    records = []
    for line_number, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as error:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {error}") from error
    return records


def command_events(records: list[dict]) -> list[dict]:
    host_events = [
        {
            "command": str(record["command"]),
            "time": float(record["hostTime"]),
            "marker_id": record.get("markerId"),
        }
        for record in records
        if record.get("type") == "command" and record.get("hostTime") is not None
    ]
    host_events.sort(key=lambda event: event["time"])

    # Command timestamps use the host clock; frames and markers use logcat's
    # clock. New captures keep adb marker writes off the control loop and record
    # the host time when adb returns, allowing us to estimate the clock offset
    # without mistaking adb's launch latency for command latency.
    markers_by_id: dict[int, dict] = {}
    legacy_markers = []
    for record in sorted(records, key=lambda item: item.get("time", 0.0)):
        if record.get("type") != "mark":
            continue
        message = str(record.get("message", ""))
        if message.startswith("send#"):
            marker_id_text, _, command = message[len("send#"):].partition(":")
            try:
                marker_id = int(marker_id_text)
            except ValueError:
                continue
            markers_by_id[marker_id] = {
                "command": command,
                "time": float(record["time"]),
            }
        elif message.startswith("send:"):
            legacy_markers.append({
                "command": message[len("send:"):],
                "time": float(record["time"]),
            })

    marker_hosts = {
        int(record["markerId"]): record
        for record in records
        if record.get("type") == "marker_host" and record.get("markerId") is not None
    }

    receives = [
        {"command": str(record["command"]), "time": float(record["time"])}
        for record in sorted(records, key=lambda item: item.get("time", 0.0))
        if record.get("type") == "control"
        and record.get("event") == "tcp_recv"
        and record.get("command") != "ack"
    ]

    marker_index = 0
    matched_markers: list[dict | None] = []
    for event in host_events:
        if event["marker_id"] is not None:
            matched_markers.append(markers_by_id.get(int(event["marker_id"])))
            continue
        match_index = next(
            (index for index in range(marker_index, len(legacy_markers))
             if legacy_markers[index]["command"] == event["command"]),
            None,
        )
        if match_index is None:
            matched_markers.append(None)
            continue
        matched_markers.append(legacy_markers[match_index])
        marker_index = match_index + 1
    clock_offsets = []
    for event, marker in zip(host_events, matched_markers):
        marker_id = event["marker_id"]
        marker_host = marker_hosts.get(int(marker_id)) if marker_id is not None else None
        if marker is not None and marker_host is not None:
            clock_offsets.append(marker["time"] - float(marker_host["hostEnd"]))
    has_return_calibration = bool(clock_offsets)
    if not clock_offsets:
        clock_offsets = [
            marker["time"] - event["time"]
            for event, marker in zip(host_events, matched_markers)
            if marker is not None
        ]
    clock_offset = statistics.median(clock_offsets) if clock_offsets else 0.0

    receive_index = 0
    events = []
    for event, marker in zip(host_events, matched_markers):
        mapped = {
            **event,
            "host_time": event["time"],
            "time": event["time"] + clock_offset,
            "time_source": (
                "host+marker_return_offset" if has_return_calibration
                else "host+median_logcat_offset" if clock_offsets
                else "host"
            ),
            "clock_offset_s": clock_offset,
        }
        if marker is not None:
            mapped["marker_time"] = marker["time"]
            mapped["marker_residual_ms"] = (marker["time"] - mapped["time"]) * 1000.0
        receive_match = next(
            (index for index in range(receive_index, len(receives))
             if receives[index]["command"] == event["command"]),
            None,
        )
        if receive_match is not None:
            mapped["receive_time"] = receives[receive_match]["time"]
            mapped["receive_delay_ms"] = (receives[receive_match]["time"] - mapped["time"]) * 1000.0
            receive_index = receive_match + 1
        events.append(mapped)
    return events


def flag_transitions(records: list[dict]) -> list[str]:
    transitions: list[str] = []
    previous: str | None = None
    for record in sorted(records, key=lambda item: item.get("hostTime", item.get("time", 0.0))):
        if record.get("type") != "tcp":
            continue
        line = str(record.get("line", ""))
        if FLAGS_MARKER not in line:
            continue
        value = line.split(FLAGS_MARKER, 1)[1].split("|", 1)[0]
        if value != previous:
            transitions.append(value)
            previous = value
    return transitions


def command_replies(records: list[dict]) -> dict | None:
    """Return command READY/BUSY outcomes when the capture has request IDs.

    New captures number every TCP request, including ACKs, and pair each reply
    with its request in stream order. Older captures do not record ACK sends,
    so guessing which status line answered a command would be unsafe.
    """
    commands = sorted(
        (record for record in records
         if record.get("type") == "command" and record.get("requestSequence") is not None),
        key=lambda record: int(record["requestSequence"]),
    )
    sends = sorted(
        (record for record in records
         if record.get("type") == "tcp_send" and record.get("requestSequence") is not None),
        key=lambda record: int(record["requestSequence"]),
    )
    if not commands or not sends:
        return None

    replies = {
        int(record["requestSequence"]): str(record.get("line", ""))
        for record in records
        if record.get("type") == "tcp" and record.get("requestSequence") is not None
    }
    outcomes = []
    for command in commands:
        sequence = int(command["requestSequence"])
        line = replies.get(sequence)
        if line is None:
            disposition = None
            flags = None
        else:
            disposition = (
                "busy" if line.startswith("busy:")
                else "ready" if line.startswith("ready:")
                else "other"
            )
            flags = None
            if FLAGS_MARKER in line:
                flags = line.split(FLAGS_MARKER, 1)[1].split("|", 1)[0]
        outcomes.append({
            "command": str(command["command"]),
            "request_sequence": sequence,
            "disposition": disposition,
            "flags": flags,
        })

    expected = list(range(1, len(sends) + 1))
    send_sequences = [int(record["requestSequence"]) for record in sends]
    complete_replies = (
        send_sequences == expected
        and all(sequence in replies for sequence in expected)
        and all(int(record.get("responseSequence", -1)) == int(record["requestSequence"])
                for record in records
                if record.get("type") == "tcp" and record.get("requestSequence") is not None)
    )
    return {
        "outcomes": outcomes,
        "complete": complete_replies,
        "all_requests": len(sends),
        "reply_count": len(replies),
        "rejected": [item["command"] for item in outcomes if item["disposition"] == "busy"],
    }


def rebuild_busy_commands(records: list[dict]) -> list[str]:
    """List commands the instrumented rebuild received while its gate was busy."""
    return [
        str(record.get("command", ""))
        for record in sorted(records, key=lambda item: item.get("time", 0.0))
        if record.get("type") == "control"
        and record.get("event") == "tcp_recv"
        and record.get("command") != "ack"
        and bool(record.get("state", {}).get("busy"))
    ]


def first_motion_event(events: list[dict]) -> dict | None:
    for event in events:
        command = event["command"]
        if command.startswith(MOTION_PREFIXES) and not command.startswith(("walkclear", "setclear")):
            return event
    return None


def rebuild_frames(records: list[dict]) -> list[dict]:
    frames = []
    frame_records = [record for record in records if record.get("type") == "frame"]
    if frame_records:
        for record in frame_records:
            pulses = record.get("pulses")
            trace = record.get("trace", {})
            body = trace.get("body")
            layer = trace.get("layer")
            if len(pulses or []) != 18 or len(body or []) != 6 or len(layer or []) != 6:
                continue
            frames.append({
                "time": float(record["time"]),
                "pulses": [int(value) for value in pulses],
                "body": [float(value) for value in body],
                "layer": [float(value) for value in layer],
            })
    else:
        # Older instrumented captures have CHICA_GAIT plus the preceding
        # CHICA_SERVO row, rather than a combined CHICA_FRAME record.
        latest_pulses = None
        for record in sorted(records, key=lambda item: item.get("time", 0.0)):
            if record.get("type") == "servo_values":
                values = record.get("values")
                latest_pulses = values if len(values or []) == 18 else None
                continue
            if record.get("type") != "gait" or latest_pulses is None:
                continue
            body = record.get("body")
            layer = record.get("layer")
            if len(body or []) != 6 or len(layer or []) != 6:
                continue
            frames.append({
                "time": float(record["time"]),
                "pulses": [int(value) for value in latest_pulses],
                "body": [float(value) for value in body],
                "layer": [float(value) for value in layer],
            })
        if not frames:
            # The oldest rebuild logs contain PWM rows only. Keep those useful
            # for comparing commands such as setdive/setrotate, but do not
            # pretend pose and animation-layer parity was observed.
            frames = [
                {
                    "time": float(record["time"]),
                    "pulses": [int(value) for value in record["values"]],
                    "body": None,
                    "layer": None,
                }
                for record in records
                if record.get("type") == "servo_values"
                and len(record.get("values") or []) == 18
            ]
    return sorted(frames, key=lambda frame: frame["time"])


def command_segments(
    frames: list[dict],
    events: list[dict],
    planned_durations: list[float] | None = None,
) -> list[dict]:
    """Partition output by command, excluding the capture tool's post-step tail."""
    if not frames or not events:
        return []
    segments = []
    capture_end = frames[-1]["time"]
    for index, event in enumerate(events):
        start = event["time"]
        end = events[index + 1]["time"] if index + 1 < len(events) else capture_end
        if planned_durations is not None and index < len(planned_durations):
            end = min(end, start + planned_durations[index])
        if end < start:
            continue
        selected = [frame for frame in frames if start <= frame["time"] <= end]
        segments.append({
            "command": event["command"],
            "duration": max(0.0, end - start),
            "frames": [
                {**frame, "elapsed": frame["time"] - start}
                for frame in selected
            ],
        })
    return segments


def capture_plan(meta: dict | None) -> tuple[tuple, list[float]] | None:
    """Return comparable capture inputs and per-command durations."""
    if meta is None or not isinstance(meta.get("steps"), list):
        return None
    steps = meta["steps"]
    probe_offsets = meta.get("statusProbeAfterMs")
    if probe_offsets is None:
        probe_offsets = ()
    elif not isinstance(probe_offsets, (list, tuple)):
        probe_offsets = (probe_offsets,)
    signature = (
        tuple((str(step.get("command", "")), float(step.get("delayMs", 0.0))) for step in steps),
        meta.get("heartbeatEnabled"),
        meta.get("emptyLineKeepalive", False),
        meta.get("ackIntervalMs"),
        meta.get("waitReadyBeforeCommand"),
        meta.get("readySettleMs"),
        meta.get("settleBeforeCommandMs"),
        tuple(probe_offsets),
    )
    return signature, [float(step.get("delayMs", 0.0)) / 1000.0 for step in steps]


def commands_match_plan(meta: dict | None, commands: list[str]) -> bool:
    if meta is None or not isinstance(meta.get("steps"), list):
        return False
    return commands == [str(step.get("command", "")) for step in meta["steps"]]


def _max_or_zero(values: list[float]) -> float:
    return max(values) if values else 0.0


def _interpolate(frames: list[dict], times: list[float], elapsed: float, key: str) -> list[float]:
    slot = bisect.bisect_left(times, elapsed)
    if slot <= 0:
        return list(frames[0][key])
    if slot >= len(frames):
        return list(frames[-1][key])
    left, right = frames[slot - 1], frames[slot]
    span = right["elapsed"] - left["elapsed"]
    amount = 0.0 if span <= 0.0 else (elapsed - left["elapsed"]) / span
    return [a + (b - a) * amount for a, b in zip(left[key], right[key])]


def timing_normalized_comparison(original: dict, rebuilt: dict, overlap: float) -> dict | None:
    """Fit one bounded time offset, then compare interpolated output curves."""
    original_frames = [frame for frame in original["frames"] if frame["elapsed"] <= overlap]
    rebuilt_frames = [frame for frame in rebuilt["frames"] if frame["elapsed"] <= overlap]
    if not original_frames or not rebuilt_frames:
        return None
    pose_keys = ("body", "layer")
    if any(any(frame.get(key) is None for frame in original_frames + rebuilt_frames) for key in pose_keys):
        return None
    if not any(
        max(frame[key][component] for frame in original_frames)
        - min(frame[key][component] for frame in original_frames) > 1e-6
        for key in pose_keys
        for component in range(len(original_frames[0][key]))
    ):
        return None

    rebuilt_times = [frame["elapsed"] for frame in rebuilt_frames]
    min_coverage = max(1, math.ceil(min(len(original_frames), len(rebuilt_frames)) * 0.80))
    best: tuple[float, int, int] | None = None
    for shift_ms in range(-100, 101):
        shift = shift_ms / 1000.0
        paired = [
            frame for frame in original_frames
            if rebuilt_times[0] <= frame["elapsed"] + shift <= rebuilt_times[-1]
        ]
        if len(paired) < min_coverage:
            continue
        squared_error = 0.0
        scalar_count = 0
        for frame in paired:
            sample_time = frame["elapsed"] + shift
            for key in pose_keys:
                estimate = _interpolate(rebuilt_frames, rebuilt_times, sample_time, key)
                deltas = [a - b for a, b in zip(frame[key], estimate)]
                squared_error += sum(delta * delta for delta in deltas)
                scalar_count += len(deltas)
        candidate = (squared_error / scalar_count, shift_ms, len(paired))
        if best is None or candidate[0] < best[0] - 1e-12 or (
            abs(candidate[0] - best[0]) <= 1e-12 and abs(candidate[1]) < abs(best[1])
        ):
            best = candidate
    if best is None:
        return None

    _, shift_ms, _ = best
    deltas_by_key = {
        "body_xyz": [], "body_angles": [],
        "layer_xyz": [], "layer_angles": [], "pulses": [],
    }
    for frame in original_frames:
        sample_time = frame["elapsed"] + shift_ms / 1000.0
        if not rebuilt_times[0] <= sample_time <= rebuilt_times[-1]:
            continue
        for key in ("body", "layer"):
            estimate = _interpolate(rebuilt_frames, rebuilt_times, sample_time, key)
            prefix = "body" if key == "body" else "layer"
            deltas_by_key[prefix + "_xyz"].extend(
                abs(a - b) for a, b in zip(frame[key][:3], estimate[:3])
            )
            deltas_by_key[prefix + "_angles"].extend(
                abs(a - b) for a, b in zip(frame[key][3:], estimate[3:])
            )
        estimate = _interpolate(rebuilt_frames, rebuilt_times, sample_time, "pulses")
        deltas_by_key["pulses"].extend(
            abs(a - b) for a, b in zip(frame["pulses"], estimate)
        )

    def stats(values: list[float]) -> dict[str, float]:
        return {
            "rms": math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0,
            "max": max(values, default=0.0),
            "mean": statistics.mean(values) if values else 0.0,
        }

    return {
        "shift_ms": shift_ms,
        "coverage": best[2],
        "body_xyz": stats(deltas_by_key["body_xyz"]),
        "body_angles": stats(deltas_by_key["body_angles"]),
        "layer_xyz": stats(deltas_by_key["layer_xyz"]),
        "layer_angles": stats(deltas_by_key["layer_angles"]),
        "pulses": stats(deltas_by_key["pulses"]),
        "at_shift_limit": abs(shift_ms) == 100,
    }


def compare_segment(original: dict, rebuilt: dict) -> tuple[dict, bool]:
    original_frames = original["frames"]
    rebuilt_frames = rebuilt["frames"]
    overlap = min(original["duration"], rebuilt["duration"])

    terminal_original = original_frames[-1] if original_frames else None
    terminal_rebuilt = rebuilt_frames[-1] if rebuilt_frames else None
    terminal_pwm_delta = None
    terminal_body_xyz_max = None
    terminal_layer_xyz_max = None
    if terminal_original is not None and terminal_rebuilt is not None:
        terminal_pwm_delta = max(
            abs(a - b)
            for a, b in zip(terminal_original["pulses"], terminal_rebuilt["pulses"])
        )
        if terminal_original.get("body") is not None and terminal_rebuilt.get("body") is not None:
            terminal_body_xyz_max = max(
                abs(a - b)
                for a, b in zip(terminal_original["body"][:3], terminal_rebuilt["body"][:3])
            )
        if terminal_original.get("layer") is not None and terminal_rebuilt.get("layer") is not None:
            terminal_layer_xyz_max = max(
                abs(a - b)
                for a, b in zip(terminal_original["layer"][:3], terminal_rebuilt["layer"][:3])
            )

    original_common = [frame for frame in original_frames if frame["elapsed"] <= overlap]
    rebuilt_common = [frame for frame in rebuilt_frames if frame["elapsed"] <= overlap]
    rebuilt_times = [frame["elapsed"] for frame in rebuilt_common]
    offsets: list[float] = []
    pulse_deltas: list[int] = []
    body_xyz: list[float] = []
    body_angles: list[float] = []
    layer_xyz: list[float] = []
    layer_angles: list[float] = []
    peak_pwm: tuple[int, float, int, int, int] | None = None
    peak_pose: tuple[float, float, str, float, float] | None = None
    exact_frames = 0
    used_rebuilt: set[int] = set()

    for original_frame in original_common:
        if not rebuilt_common:
            break
        elapsed = original_frame["elapsed"]
        slot = bisect.bisect_left(rebuilt_times, elapsed)
        candidates = [index for index in (slot - 1, slot) if 0 <= index < len(rebuilt_common)]
        selected_index = min(candidates, key=lambda index: abs(rebuilt_times[index] - elapsed))
        rebuilt_frame = rebuilt_common[selected_index]
        used_rebuilt.add(selected_index)
        offsets.append(abs(rebuilt_frame["elapsed"] - elapsed) * 1000.0)

        deltas = [abs(a - b) for a, b in zip(original_frame["pulses"], rebuilt_frame["pulses"])]
        pulse_deltas.extend(deltas)
        exact_frames += all(delta == 0 for delta in deltas)
        if deltas:
            channel = max(range(len(deltas)), key=deltas.__getitem__)
            candidate = (
                deltas[channel], elapsed, channel,
                original_frame["pulses"][channel], rebuilt_frame["pulses"][channel],
            )
            if peak_pwm is None or candidate[0] > peak_pwm[0]:
                peak_pwm = candidate
        if original_frame.get("body") is not None and rebuilt_frame.get("body") is not None:
            for component, (a, b) in enumerate(zip(original_frame["body"], rebuilt_frame["body"])):
                delta = abs(a - b)
                (body_xyz if component < 3 else body_angles).append(delta)
                candidate = (
                    delta, elapsed, f"body[{component}]", a, b,
                )
                if peak_pose is None or candidate[0] > peak_pose[0]:
                    peak_pose = candidate
        if original_frame.get("layer") is not None and rebuilt_frame.get("layer") is not None:
            for component, (a, b) in enumerate(zip(original_frame["layer"], rebuilt_frame["layer"])):
                delta = abs(a - b)
                (layer_xyz if component < 3 else layer_angles).append(delta)
                candidate = (
                    delta, elapsed, f"layer[{component}]", a, b,
                )
                if peak_pose is None or candidate[0] > peak_pose[0]:
                    peak_pose = candidate

    pose_comparable = bool(body_xyz or body_angles) and bool(layer_xyz or layer_angles)
    exact = (
        bool(offsets)
        and len(original_frames) == len(rebuilt_frames)
        and len(original_common) == len(rebuilt_common)
        and len(used_rebuilt) == len(rebuilt_common)
        and pose_comparable
        and all(offset == 0.0 for offset in offsets)
        and all(delta == 0 for delta in pulse_deltas)
        and all(delta == 0.0 for delta in body_xyz + body_angles + layer_xyz + layer_angles)
    )
    summary = {
        "duration_original": original["duration"],
        "duration_rebuilt": rebuilt["duration"],
        "overlap": overlap,
        "frames_original": len(original_frames),
        "frames_rebuilt": len(rebuilt_frames),
        "frames_compared": len(offsets),
        "rebuilt_frames_used": len(used_rebuilt),
        "offsets": offsets,
        "pulse_deltas": pulse_deltas,
        "pulse_exact_frames": exact_frames,
        "body_xyz_max": _max_or_zero(body_xyz),
        "body_angle_max": _max_or_zero(body_angles),
        "layer_xyz_max": _max_or_zero(layer_xyz),
        "layer_angle_max": _max_or_zero(layer_angles),
        "pose_comparable": pose_comparable,
        "peak_pwm": peak_pwm,
        "peak_pose": peak_pose,
        "terminal_pwm_delta": terminal_pwm_delta,
        "terminal_body_xyz_max": terminal_body_xyz_max,
        "terminal_layer_xyz_max": terminal_layer_xyz_max,
        "timing_normalized": timing_normalized_comparison(original, rebuilt, overlap),
        "exact": exact,
    }
    return summary, exact


def capture_is_complete(counts: dict[str, int]) -> bool:
    return (
        counts.get("complete_frames", 0) == counts.get("pose_headers", -1)
        and counts.get("missing_pose", 0) == 0
        and counts.get("missing_pulses", 0) == 0
        and counts.get("missing_feet", 0) == 0
        and counts.get("bad_pulse_width", 0) == 0
        and counts.get("duplicate_pose_ids", 0) == 0
        and counts.get("duplicate_pulse_ids", 0) == 0
    )


def capture_meta(records: list[dict]) -> dict | None:
    return next((record for record in records if record.get("type") == "capture_meta"), None)


def quiet_control_capture_pair(original: dict | None, rebuilt: dict | None) -> bool:
    if not original or not rebuilt:
        return False
    if original.get("heartbeatEnabled") is not False or rebuilt.get("heartbeatEnabled") is not False:
        return False
    if original.get("waitReadyBeforeCommand") or rebuilt.get("waitReadyBeforeCommand"):
        return False
    for key in ("settleBeforeCommandMs", "statusProbeAfterMs", "steps"):
        if original.get(key) != rebuilt.get(key):
            return False
    steps = original.get("steps")
    if not isinstance(steps, list) or len(steps) != 1:
        return False
    settle_ms = float(original.get("settleBeforeCommandMs", 0.0))
    # Require a generous quiet interval after the single command. Status probes
    # are sent at fixed points while it runs; a BUSY reply does not enqueue ACK
    # behavior, and probing stops at the first READY reply.
    return all(float(step.get("delayMs", 0.0)) + settle_ms >= 1500.0 for step in steps)


def status_probe_outcomes(records: list[dict]) -> dict | None:
    probes = sorted(
        (record for record in records if record.get("type") == "status_probe"),
        key=lambda record: int(record["requestSequence"]),
    )
    if not probes:
        return None
    replies = {
        int(record["requestSequence"]): record
        for record in records
        if record.get("type") == "tcp" and record.get("requestSequence") is not None
    }
    command_records = sorted(
        (record for record in records
         if record.get("type") == "command" and record.get("requestSequence") is not None),
        key=lambda record: int(record["requestSequence"]),
    )
    commands = {int(record["requestSequence"]): record for record in command_records}
    command_indices = {
        int(record["requestSequence"]): index
        for index, record in enumerate(command_records)
    }
    sends = {
        int(record["requestSequence"]): record
        for record in records
        if record.get("type") == "tcp_send" and record.get("requestSequence") is not None
    }
    outcomes = []
    for probe in probes:
        sequence = int(probe["requestSequence"])
        reply = replies.get(sequence)
        line = str(reply.get("line", "")) if reply else None
        flags = line.split(FLAGS_MARKER, 1)[1].split("|", 1)[0] if line and FLAGS_MARKER in line else None
        disposition = (
            "busy" if line and line.startswith("busy:")
            else "ready" if line and line.startswith("ready:")
            else "other" if line else None
        )
        parent = int(probe.get("afterCommandSequence", -1))
        command = commands.get(parent)
        send = sends.get(sequence)
        command_host_time = command.get("hostTime") if command else None
        send_host_time = send.get("hostTime") if send else None
        response_host_time = reply.get("hostTime") if reply else None
        outcomes.append({
            "command": str(command.get("command", "")) if command else None,
            "command_index": command_indices.get(parent),
            "offset_ms": float(probe.get("offsetMs", 0.0)),
            "actual_offset_ms": (
                (float(send_host_time) - float(command_host_time)) * 1000.0
                if send_host_time is not None and command_host_time is not None
                else float(probe.get("offsetMs", 0.0))
            ),
            "disposition": disposition,
            "flags": flags,
            "response_delay_ms": (
                (float(response_host_time) - float(send_host_time)) * 1000.0
                if response_host_time is not None and send_host_time is not None
                else None
            ),
        })
    expected = [int(probe["requestSequence"]) for probe in probes]
    return {
        "complete": all(sequence in replies for sequence in expected),
        "outcomes": outcomes,
    }


def readiness_windows(probes: dict | None) -> dict[int, dict]:
    """Bound command completion using the last BUSY and first READY probe."""
    grouped: dict[int, list[dict]] = {}
    if probes is not None:
        for outcome in probes["outcomes"]:
            command_index = outcome.get("command_index")
            if command_index is not None:
                grouped.setdefault(int(command_index), []).append(outcome)

    windows = {}
    for command_index, outcomes in grouped.items():
        busy = [item for item in outcomes if item["disposition"] == "busy"]
        ready = [item for item in outcomes if item["disposition"] == "ready"]
        windows[command_index] = {
            "last_busy_ms": busy[-1]["actual_offset_ms"] if busy else None,
            "first_ready_ms": ready[0]["actual_offset_ms"] if ready else None,
            "right_censored_after_ms": (
                busy[-1]["actual_offset_ms"] if busy and not ready else None
            ),
            "ready_flags": ready[0]["flags"] if ready else None,
        }
    return windows


def trim_segment_before_ready_probe(segment: dict, cutoff_ms: float | None) -> None:
    if cutoff_ms is None:
        return
    cutoff_s = max(0.0, cutoff_ms / 1000.0)
    segment["frames"] = [
        frame for frame in segment["frames"] if frame["elapsed"] < cutoff_s
    ]
    segment["duration"] = min(segment["duration"], cutoff_s)


def gated_trajectory_divergence(
    segment_results: list[tuple[str, dict | None, bool | None]],
    *,
    controlled_inputs: bool,
) -> list[tuple[int, str, dict]]:
    """Flag large residual path differences after timing alignment.

    This stricter diagnosis is only enabled for captures that gate commands on
    READY and pair every request/reply. Independent frame scheduling can still
    leave small residuals, so only large pose or PWM differences qualify.
    """
    if not controlled_inputs:
        return []
    divergent = []
    for index, (command, result, _) in enumerate(segment_results):
        if result is None:
            continue
        normalized = result["timing_normalized"]
        if normalized is None or normalized["at_shift_limit"]:
            continue
        large_pose_delta = (
            normalized["layer_xyz"]["max"] > 10.0
            or normalized["layer_angles"]["max"] > 5.0
            or normalized["body_xyz"]["max"] > 10.0
            or normalized["body_angles"]["max"] > 5.0
        )
        large_pwm_delta = normalized["pulses"]["mean"] > 25.0
        if large_pose_delta or large_pwm_delta:
            divergent.append((index, command, normalized))
    return divergent


def summarize(original_path: Path, rebuilt_path: Path) -> int:
    original_records = load(original_path)
    rebuilt_records = load(rebuilt_path)
    original_events = command_events(original_records)
    rebuilt_events = command_events(rebuilt_records)
    original_commands = [event["command"] for event in original_events]
    rebuilt_commands = [event["command"] for event in rebuilt_events]
    original_flags = flag_transitions(original_records)
    rebuilt_flags = flag_transitions(rebuilt_records)
    original_replies = command_replies(original_records)
    rebuilt_replies = command_replies(rebuilt_records)
    original_meta = capture_meta(original_records)
    rebuilt_meta = capture_meta(rebuilt_records)
    original_plan = capture_plan(original_meta)
    rebuilt_plan = capture_plan(rebuilt_meta)
    capture_plan_exact = (
        original_plan is not None
        and rebuilt_plan is not None
        and original_plan[0] == rebuilt_plan[0]
    )
    strict_capture = quiet_control_capture_pair(original_meta, rebuilt_meta)
    original_probes = status_probe_outcomes(original_records)
    rebuilt_probes = status_probe_outcomes(rebuilt_records)
    original_windows = readiness_windows(original_probes)
    rebuilt_windows = readiness_windows(rebuilt_probes)
    probes_exact: bool | None = None
    probes_complete = False
    if original_probes is not None and rebuilt_probes is not None:
        probes_complete = original_probes["complete"] and rebuilt_probes["complete"]
        if probes_complete:
            def probe_signature(probes: dict) -> list[tuple]:
                return [
                    (
                        item["command_index"], item["offset_ms"],
                        item["disposition"], item["flags"],
                    )
                    for item in probes["outcomes"]
                ]

            probes_exact = probe_signature(original_probes) == probe_signature(rebuilt_probes)
    reply_exact: bool | None = None
    reply_complete = False
    reply_mismatches: list[tuple[int, dict, dict]] = []
    reply_flag_mismatches: list[tuple[int, dict, dict]] = []
    if original_replies is not None and rebuilt_replies is not None:
        reply_complete = original_replies["complete"] and rebuilt_replies["complete"]
        if reply_complete:
            original_reply_effects = [
                (item["command"], item["disposition"])
                for item in original_replies["outcomes"]
            ]
            rebuilt_reply_effects = [
                (item["command"], item["disposition"])
                for item in rebuilt_replies["outcomes"]
            ]
            reply_exact = original_reply_effects == rebuilt_reply_effects
            reply_mismatches = [
                (index, original_item, rebuilt_item)
                for index, (original_item, rebuilt_item) in enumerate(zip(
                    original_replies["outcomes"], rebuilt_replies["outcomes"]
                ))
                if (original_item["command"], original_item["disposition"])
                != (rebuilt_item["command"], rebuilt_item["disposition"])
            ]
            reply_flag_mismatches = [
                (index, original_item, rebuilt_item)
                for index, (original_item, rebuilt_item) in enumerate(zip(
                    original_replies["outcomes"], rebuilt_replies["outcomes"]
                ))
                if original_item["flags"] != rebuilt_item["flags"]
            ]
    original_motion = first_motion_event(original_events)
    rebuilt_motion = first_motion_event(rebuilt_events)
    original_gt, capture_counts = parse_capture(original_path)
    original_frames = [
        {
            **frame,
            "time": float(frame["time"]),
        }
        for frame in original_gt
    ]
    rebuilt_frames = rebuild_frames(rebuilt_records)

    commands_exact = original_commands == rebuilt_commands
    command_inputs_match_plan = (
        commands_match_plan(original_meta, original_commands)
        and commands_match_plan(rebuilt_meta, rebuilt_commands)
    )
    flags_exact = original_flags == rebuilt_flags
    trigger_exact = (
        (original_motion is None and rebuilt_motion is None)
        or (
            original_motion is not None
            and rebuilt_motion is not None
            and original_motion["command"] == rebuilt_motion["command"]
        )
    )
    original_segments = command_segments(
        original_frames, original_events,
        original_plan[1] if original_plan is not None else None,
    )
    rebuilt_segments = command_segments(
        rebuilt_frames, rebuilt_events,
        rebuilt_plan[1] if rebuilt_plan is not None else None,
    )
    control_divergence_index = min(
        (index for index, _, _ in reply_mismatches),
        default=None,
    )
    if not original_events or not rebuilt_events:
        print(
            f"BOUNDED: command stream missing; original={len(original_events)} "
            f"rebuilt={len(rebuilt_events)}"
        )
        return 2
    if not original_frames or not rebuilt_frames or not original_segments or not rebuilt_segments:
        print(
            f"BOUNDED: command captures exist, but frame outputs are unavailable; "
            f"original_frames={len(original_frames)} rebuilt_frames={len(rebuilt_frames)} "
            f"capture_counts={capture_counts}"
        )
        return 2

    paired_count = min(len(original_segments), len(rebuilt_segments))
    segment_results = []
    for index, (original, rebuilt) in enumerate(zip(
        original_segments[:paired_count], rebuilt_segments[:paired_count]
    )):
        if control_divergence_index is not None and index >= control_divergence_index:
            segment_results.append((original["command"], None, None))
            continue
        first_ready_offsets = [
            windows[index]["first_ready_ms"]
            for windows in (original_windows, rebuilt_windows)
            if index in windows and windows[index]["first_ready_ms"] is not None
        ]
        cutoff_ms = min(first_ready_offsets) if first_ready_offsets else None
        trim_segment_before_ready_probe(original, cutoff_ms)
        trim_segment_before_ready_probe(rebuilt, cutoff_ms)
        summary, exact = compare_segment(original, rebuilt)
        summary["probe_cutoff_ms"] = cutoff_ms
        segment_results.append((original["command"], summary, exact))

    all_outputs_exact = (
        len(original_segments) == len(rebuilt_segments)
        and len(segment_results) == len(original_segments)
        and commands_exact
        and command_inputs_match_plan
        and control_divergence_index is None
        and all(exact for _, _, exact in segment_results)
        and capture_plan_exact
        and capture_is_complete(capture_counts)
        and reply_exact is True
        and flags_exact
        and (probes_exact is True if original_probes is not None or rebuilt_probes is not None else True)
    )
    # Repeated ACKs are not an idle query: each accepted ACK advances the
    # original app's ACK-ramp behavior. Only paired quiet captures use strict
    # READY/BUSY, status-probe, and FLAGS parity checks. ACK-gated and ordinary
    # heartbeat stress captures remain useful diagnostics but cannot establish
    # an isolated command mismatch.
    ready_flags_mismatch = strict_capture and any(
        original_windows[index].get("ready_flags") is not None
        and rebuilt_windows[index].get("ready_flags") is not None
        and original_windows[index]["ready_flags"] != rebuilt_windows[index]["ready_flags"]
        for index in original_windows.keys() & rebuilt_windows.keys()
    )
    strict_control_mismatch = strict_capture and capture_plan_exact and command_inputs_match_plan and (
        reply_exact is False or ready_flags_mismatch
    )
    gated_profile = (
        capture_plan_exact
        and bool(original_meta and original_meta.get("waitReadyBeforeCommand"))
        and bool(rebuilt_meta and rebuilt_meta.get("waitReadyBeforeCommand"))
        and reply_complete
        and command_inputs_match_plan
    )
    gated_control_mismatch = gated_profile and reply_exact is False
    controlled_inputs = (
        gated_profile
        and reply_exact is True
        and flags_exact
        and capture_is_complete(capture_counts)
        and not reply_flag_mismatches
    )
    trajectory_divergences = gated_trajectory_divergence(
        segment_results, controlled_inputs=controlled_inputs,
    )
    if strict_control_mismatch or gated_control_mismatch or trajectory_divergences:
        status = "FAIL"
    else:
        # Per-request FLAGS and READY/BUSY observations are sampled at async
        # boundaries. Keep timeline differences visible as BOUNDED evidence;
        # only command disposition or settled READY FLAGS differences are a
        # hard control-path failure.
        status = "PASS" if all_outputs_exact else "BOUNDED"

    print(
        f"{original_path.name} vs {rebuilt_path.name}: status={status} "
        f"commands_exact={commands_exact} flags_exact={flags_exact} "
        f"command_replies_exact={reply_exact if reply_complete else 'unavailable/incomplete'} "
        f"status_probes_exact={probes_exact if probes_complete else 'unavailable/incomplete'} "
        f"capture_plan_exact={capture_plan_exact} "
        f"command_inputs_match_plan={command_inputs_match_plan} "
        f"controlled_inputs={controlled_inputs} "
        f"large_trajectory_divergences={len(trajectory_divergences)} "
        f"control_check={'strict-quiet' if strict_capture else 'stress-diagnostic'} "
        f"reply_flags_snapshot_mismatches={len(reply_flag_mismatches)} "
        f"original_gt_complete={capture_is_complete(capture_counts)} "
        f"motion_trigger={original_motion['command'] if original_motion else None!r}"
    )
    print(
        f"  commands={len(original_commands)}/{len(rebuilt_commands)} "
        f"FLAGS_transitions={len(original_flags)}/{len(rebuilt_flags)} "
        f"frames={len(original_frames)}/{len(rebuilt_frames)} "
        f"command_clock=original:{sum(e['time_source'].startswith('host+') for e in original_events)}/"
        f"{len(original_events)} rebuilt:{sum(e['time_source'].startswith('host+') for e in rebuilt_events)}/"
        f"{len(rebuilt_events)} calibrated "
        f"segments={len(original_segments)}/{len(rebuilt_segments)}"
    )
    if original_events and rebuilt_events and "clock_offset_s" in original_events[0] and "clock_offset_s" in rebuilt_events[0]:
        residuals = [
            abs(event["marker_residual_ms"])
            for event in original_events + rebuilt_events
            if "marker_residual_ms" in event
        ]
        marker_p95 = sorted(residuals)[min(len(residuals) - 1, math.ceil(0.95 * len(residuals)) - 1)] if residuals else 0.0
        print(
            f"  host_to_logcat_offset_ms="
            f"original:{original_events[0]['clock_offset_s'] * 1000.0:+.1f} "
            f"rebuilt:{rebuilt_events[0]['clock_offset_s'] * 1000.0:+.1f} "
            f"abs_marker_residual_ms=p95:{marker_p95:.1f} max:{max(residuals, default=0.0):.1f}"
        )
    receive_offsets = [
        event["receive_delay_ms"]
        for event in rebuilt_events if "receive_delay_ms" in event
    ]
    if receive_offsets:
        ordered_receive = sorted(receive_offsets)
        p95_index = min(len(ordered_receive) - 1, math.ceil(0.95 * len(ordered_receive)) - 1)
        print(
            f"  rebuild_tcp_rx_minus_send_ms="
            f"median:{statistics.median(receive_offsets):.1f} "
            f"p95:{ordered_receive[p95_index]:.1f} max:{max(receive_offsets):.1f}"
        )

    if original_replies is not None and rebuilt_replies is not None:
        original_busy = original_replies["rejected"]
        rebuilt_busy = rebuilt_replies["rejected"]
        print(
            f"  command_replies: requests={original_replies['all_requests']}/"
            f"{rebuilt_replies['all_requests']} replies={original_replies['reply_count']}/"
            f"{rebuilt_replies['reply_count']} rejected_busy="
            f"original:{original_busy!r} rebuild:{rebuilt_busy!r}"
        )
        for index, original_item, rebuilt_item in reply_mismatches:
            print(
                f"    reply_mismatch[{index}] {original_item['command']!r}: "
                f"original={original_item['disposition']} "
                f"rebuild={rebuilt_item['disposition']}"
            )
        for index, original_item, rebuilt_item in reply_flag_mismatches:
            print(
                f"    reply_flags_snapshot[{index}] {original_item['command']!r}: "
                f"original={original_item['flags']} rebuild={rebuilt_item['flags']} "
                "(sampled during asynchronous command dispatch)"
            )
    else:
        busy_commands = rebuild_busy_commands(rebuilt_records)
        if busy_commands:
            print(
                f"  rebuild_received_while_busy={busy_commands!r} "
                "(instrumented rebuild only; original reply pairing unavailable)"
            )

    if original_probes is not None or rebuilt_probes is not None:
        original_probe_rows = original_probes["outcomes"] if original_probes else []
        rebuilt_probe_rows = rebuilt_probes["outcomes"] if rebuilt_probes else []
        print(
            f"  status_probes: complete={probes_complete} "
            f"original={original_probe_rows!r} rebuilt={rebuilt_probe_rows!r}"
        )
        print(
            f"  readiness_windows_ms: original={original_windows!r} "
            f"rebuilt={rebuilt_windows!r}"
        )

    for index, (command, result, exact) in enumerate(segment_results):
        if result is None:
            print(
                f"  [{index}] {command}: UNCOMPARABLE "
                f"(command stream diverged at request {control_divergence_index})"
            )
            continue
        offsets = result["offsets"]
        deltas = result["pulse_deltas"]
        offset_p95 = sorted(offsets)[math.ceil(0.95 * len(offsets)) - 1] if offsets else 0.0
        offset_median = statistics.median(offsets) if offsets else 0.0
        offset_max = max(offsets, default=0.0)
        mean_pwm_delta = statistics.mean(deltas) if deltas else 0.0
        label = "PASS" if exact else "BOUNDED"
        print(
            f"  [{index}] {command}: {label} "
            f"duration={result['duration_original']:.3f}/{result['duration_rebuilt']:.3f}s "
            f"overlap={result['overlap']:.3f}s "
            f"frames={result['frames_original']}/{result['frames_rebuilt']} "
            f"compared={result['frames_compared']} unique_rebuild={result['rebuilt_frames_used']}"
        )
        if result["probe_cutoff_ms"] is not None:
            print(f"      trajectory_clipped_before_accepted_ack={result['probe_cutoff_ms']:.1f}ms")
        pose_summary = (
            f"body_xyz_max={result['body_xyz_max']:.3f}mm "
            f"body_angle_max={result['body_angle_max']:.3f}deg "
            f"layer_xyz_max={result['layer_xyz_max']:.3f}mm "
            f"layer_angle_max={result['layer_angle_max']:.3f}deg"
            if result["pose_comparable"] else "pose_deltas=unavailable"
        )
        print(
            f"      nearest_offset_ms=median:{offset_median:.2f} "
            f"p95:{offset_p95:.2f} max:{offset_max:.2f} "
            f"PWM_max_delta={max(deltas, default=0)} PWM_mean_delta={mean_pwm_delta:.3f}us "
            f"{pose_summary}"
        )
        if result["terminal_pwm_delta"] is not None:
            terminal_pose = (
                f"terminal_body_xyz_max={result['terminal_body_xyz_max']:.3f}mm "
                f"terminal_layer_xyz_max={result['terminal_layer_xyz_max']:.3f}mm"
                if result["terminal_body_xyz_max"] is not None
                and result["terminal_layer_xyz_max"] is not None
                else "terminal_pose=unavailable"
            )
            print(
                f"      terminal_pwm_max_delta={result['terminal_pwm_delta']}us "
                f"{terminal_pose}"
            )
        normalized = result["timing_normalized"]
        if normalized is not None:
            body_xyz = normalized["body_xyz"]
            body_angles = normalized["body_angles"]
            layer_xyz = normalized["layer_xyz"]
            layer_angles = normalized["layer_angles"]
            pwm = normalized["pulses"]
            print(
                f"      timing_normalized: rebuild_shift={normalized['shift_ms']:+d}ms "
                f"coverage={normalized['coverage']} "
                f"body_xyz_rms/max={body_xyz['rms']:.3f}/{body_xyz['max']:.3f}mm "
                f"body_angle_rms/max={body_angles['rms']:.3f}/{body_angles['max']:.3f}deg "
                f"layer_xyz_rms/max={layer_xyz['rms']:.3f}/{layer_xyz['max']:.3f}mm "
                f"layer_angle_rms/max={layer_angles['rms']:.3f}/{layer_angles['max']:.3f}deg "
                f"PWM_MAE/max={pwm['mean']:.2f}/{pwm['max']:.2f}us"
                f"{' shift_limit' if normalized['at_shift_limit'] else ''}"
            )
            if controlled_inputs:
                path_label = (
                    "large-divergence"
                    if any(item[0] == index for item in trajectory_divergences)
                    else "no-large-divergence"
                )
                print(f"      controlled_path_check={path_label}")
        peak_pwm = result["peak_pwm"]
        if peak_pwm is not None:
            delta, elapsed, channel, original, rebuilt = peak_pwm
            print(
                f"      peak_PWM_at={elapsed:.3f}s output={channel} "
                f"original={original} rebuilt={rebuilt} delta={delta}us"
            )
        peak_pose = result["peak_pose"]
        if peak_pose is not None:
            delta, elapsed, component, original, rebuilt = peak_pose
            print(
                f"      peak_pose_at={elapsed:.3f}s component={component} "
                f"original={original:.3f} rebuilt={rebuilt:.3f} delta={delta:.3f}"
            )

    if not capture_is_complete(capture_counts):
        print(f"  original_capture_counts={capture_counts}")
    if not commands_exact:
        print(f"  commands_original={original_commands}")
        print(f"  commands_rebuilt={rebuilt_commands}")
    if not flags_exact:
        print(f"  flags_original={original_flags}")
        print(f"  flags_rebuilt={rebuilt_flags}")
    return 0 if status == "PASS" else (2 if status == "BOUNDED" else 1)


def discover_pairs(capture_dir: Path) -> list[tuple[Path, Path]]:
    originals = {
        path.name[len("gt_jsonl_"):-len(".jsonl")]: path
        for path in capture_dir.glob("gt_jsonl_*.jsonl")
    }
    rebuilt: dict[str, Path] = {}
    for prefix in ("rebuild_", "rebuild_jsonl_"):
        for path in capture_dir.glob(f"{prefix}*.jsonl"):
            key = path.name[len(prefix):-len(".jsonl")]
            rebuilt.setdefault(key, path)
    return [(originals[key], rebuilt[key]) for key in sorted(originals.keys() & rebuilt.keys())]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("original", nargs="?", type=Path)
    parser.add_argument("rebuilt", nargs="?", type=Path)
    parser.add_argument("--capture-dir", type=Path, help="discover matching gt_jsonl_/rebuild_jsonl_ pairs")
    parser.add_argument(
        "--summary", action="store_true",
        help="print aggregate component parity; omit per-command frame diagnostics",
    )
    args = parser.parse_args()

    if bool(args.original) != bool(args.rebuilt):
        parser.error("provide both positional captures or neither")
    pairs = (
        [(args.original, args.rebuilt)]
        if args.original and args.rebuilt
        else discover_pairs(args.capture_dir) if args.capture_dir else []
    )
    if not pairs:
        print("SKIP: no matching original/rebuild runtime capture pairs")
        return 3

    statuses = []
    pair_summaries: list[tuple[int, str]] = []
    for original, rebuilt in pairs:
        missing = [path for path in (original, rebuilt) if not path.is_file()]
        if missing:
            for path in missing:
                print(f"FAIL: missing capture: {path}")
            statuses.append(1)
            continue
        if args.summary:
            captured = io.StringIO()
            with contextlib.redirect_stdout(captured):
                status = summarize(original, rebuilt)
            transcript = captured.getvalue().splitlines()
            headline = next((line for line in transcript if "status=" in line), "")
            pair_summaries.append((status, headline))
            statuses.append(status)
        else:
            statuses.append(summarize(original, rebuilt))

    if args.summary:
        labels = {0: "PASS", 1: "FAIL", 2: "BOUNDED"}
        counts = {name: sum(status == code for status in statuses) for code, name in labels.items()}
        print(
            f"runtime capture pairs={len(statuses)} exact={counts['PASS']} "
            f"bounded={counts['BOUNDED']} failed={counts['FAIL']}"
        )
        component_names = (
            ("commands_exact", "command_sequences"),
            ("flags_exact", "FLAGS_transitions"),
            ("command_replies_exact", "command_replies"),
            ("status_probes_exact", "status_probes"),
            ("capture_plan_exact", "capture_plans"),
        )
        for key, label in component_names:
            known = [
                "True" in headline.split(key + "=", 1)[1].split()[0]
                for _, headline in pair_summaries
                if key + "=" in headline
                and headline.split(key + "=", 1)[1].split()[0] not in (
                    "unavailable/incomplete", "None",
                )
            ]
            print(f"{label} exact={sum(known)}/{len(known)} comparable")
        failures = [headline for status, headline in pair_summaries if status == 1 and headline]
        for headline in failures[:5]:
            print("FAIL: " + headline)

    if 1 in statuses:
        return 1
    if 2 in statuses:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
