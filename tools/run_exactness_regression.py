#!/usr/bin/env python3
"""Run available APK-grounded, native differential, and device checks."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
DEFAULT_ORACLE_DIR = Path(os.environ.get("CHICA_ORACLE_DIR", WORKSPACE / "research-private" / "oracle"))
DEFAULT_CAPTURE_DIR = Path(os.environ.get(
    "CHICA_CAPTURE_DIR", WORKSPACE / "research-private" / "server-apk-analysis"
))


@dataclass(frozen=True)
class Check:
    label: str
    command: tuple[str, ...]
    references: tuple[Path, ...] = ()


def check(label: str, *command: str, references: tuple[Path, ...] = ()) -> Check:
    return Check(label, tuple(command), references)


def run_check(item: Check, require_all: bool, verbose: bool = False) -> str:
    print(f"\n== {item.label} ==", flush=True)
    missing = [path for path in item.references if not path.is_file()]
    if missing:
        for path in missing:
            print(f"missing reference: {path}")
        status = "FAIL" if require_all else "SKIP"
        print(f"{item.label}: {status} (reference data unavailable)")
        return status

    command = item.command
    if not verbose and item.label in {
        "original-apk-pose-to-pwm",
        "original-vs-rebuild-runtime-captures",
        "saved-rebuild-walk-replay",
    }:
        command = (*command, "--summary")
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    if result.stdout:
        print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="" if result.stderr.endswith("\n") else "\n")
    if result.returncode == 0:
        status = "PASS"
    elif result.returncode == 3:
        status = "FAIL" if require_all else "SKIP"
        print(f"{item.label}: no matching runtime capture pairs ({status})")
    elif result.returncode == 2:
        status = "BOUNDED"
        print(f"{item.label}: comparison ran, but its captured precision does not prove exactness")
    else:
        status = "FAIL"
        print(f"{item.label}: command failed with exit code {result.returncode}")
    return status


def native_reference_differential(capture_dir: Path) -> str:
    source = capture_dir / "oracle" / "oracle.cpp"
    if not source.is_file():
        print(f"\n== native-reference-differential ==\nSKIP: no local oracle source at {source}")
        return "SKIP"

    binary = ROOT / "build" / "native_reference_differential"
    command = [
        "c++", "-std=c++17", "-O2", "-Iapp/src/main/cpp",
        str(source), "app/src/main/cpp/apk_model.cpp",
        "app/src/main/cpp/pulse_conversion.cpp", "-o", str(binary),
    ]
    print("\n== native-gait-vs-apk-transliteration ==", flush=True)
    print(
        "evidence scope: the local oracle scenarios compare apk_model against an "
        "independent transliteration of the APK gait; this is not a direct "
        "original-APK runtime comparison",
        flush=True,
    )
    compiled = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    if compiled.stdout:
        print(compiled.stdout, end="" if compiled.stdout.endswith("\n") else "\n")
    if compiled.stderr:
        print(compiled.stderr, file=sys.stderr, end="" if compiled.stderr.endswith("\n") else "\n")
    if compiled.returncode != 0:
        print(f"native reference compile failed with exit code {compiled.returncode}")
        return "FAIL"
    result = subprocess.run([str(binary)], cwd=ROOT, text=True, capture_output=True)
    if result.stdout:
        print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="" if result.stderr.endswith("\n") else "\n")
    if result.returncode != 0:
        print(f"native reference differential failed with exit code {result.returncode}")
        return "FAIL"
    return "PASS"


def runtime_command_coverage(capture_dir: Path) -> None:
    """List source command routes present in paired original/rebuild captures."""
    controller = next((ROOT / "app/src/main/java/com").glob("*/chicaserver/control/ChicaController.java"), None)
    if controller is None or not controller.is_file():
        return
    source = controller.read_text(errors="replace")
    prefixes = set()
    for match in re.finditer(r'(?m)^[^\n]*command\.startsWith\("([^"]+)"[^\n]*$', source):
        # Fixture injection routes exist only in rebuilt developer/test builds;
        # they are not original app commands and cannot be black-box parity cases.
        if "DEVELOPER_FIXTURES" in match.group(0):
            continue
        prefixes.add(match.group(1))
    prefixes = sorted(prefixes, key=lambda s: (-len(s), s))
    originals = {
        path.name[len("gt_jsonl_"):-len(".jsonl")]: path
        for path in capture_dir.glob("gt_jsonl_*.jsonl")
    }
    rebuilt = {}
    for prefix in ("rebuild_", "rebuild_jsonl_"):
        for path in capture_dir.glob(f"{prefix}*.jsonl"):
            rebuilt.setdefault(path.name[len(prefix):-len(".jsonl")], path)
    pairs = [(originals[key], rebuilt[key]) for key in sorted(originals.keys() & rebuilt.keys())]
    observed: dict[str, set[str]] = {prefix: set() for prefix in prefixes}
    paired_sequences = 0
    pair_count = 0
    matched_plans = 0
    mismatched_pairs: list[tuple[str, list[str], list[str]]] = []

    def records(path: Path) -> list[dict]:
        out = []
        for line in path.read_text(errors="replace").splitlines():
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        return out

    def commands(rows: list[dict]) -> list[str]:
        return [str(row["command"]) for row in rows if row.get("type") == "command"]

    for original_path, rebuilt_path in pairs:
        original_rows, rebuilt_rows = records(original_path), records(rebuilt_path)
        original_commands, rebuilt_commands = commands(original_rows), commands(rebuilt_rows)
        pair_count += 1
        matched = original_commands == rebuilt_commands
        if not matched:
            mismatched_pairs.append((original_path.name, original_commands, rebuilt_commands))
        if matched:
            paired_sequences += 1
        original_meta = next((r for r in original_rows if r.get("type") == "capture_meta"), {})
        rebuilt_meta = next((r for r in rebuilt_rows if r.get("type") == "capture_meta"), {})
        plan_keys = (
            "steps", "heartbeatEnabled", "ackIntervalMs", "waitReadyBeforeCommand",
            "readySettleMs", "settleBeforeCommandMs", "statusProbeAfterMs",
            "emptyLineKeepalive",
        )
        same_plan = all(original_meta.get(key) == rebuilt_meta.get(key) for key in plan_keys)
        if matched and same_plan:
            matched_plans += 1
            for prefix in prefixes:
                if any(command.startswith(prefix) for command in original_commands):
                    observed[prefix].add(original_path.name)

    print("\n== runtime-command-route coverage ==")
    print(
        f"paired captures={pair_count} identical command sequences="
        f"{paired_sequences}/{pair_count} identical capture plans="
        f"{matched_plans}/{pair_count}"
    )
    for filename, original_commands, rebuilt_commands in mismatched_pairs[:3]:
        print(
            f"  sequence mismatch {filename}: original={original_commands!r} "
            f"rebuild={rebuilt_commands!r}"
        )
    covered = [prefix for prefix in prefixes if observed[prefix]]
    missing = [prefix for prefix in prefixes if not observed[prefix]]
    intentionally_not_replayed = [prefix for prefix in ("restart", "reboot") if prefix in missing]
    missing = [prefix for prefix in missing if prefix not in intentionally_not_replayed]
    print("captured routes (presence only; exactness is reported by each comparator):")
    print("  " + (", ".join(covered) if covered else "none"))
    print("source routes with no paired capture:")
    print("  " + (", ".join(missing) if missing else "none"))
    if intentionally_not_replayed:
        print("system actions NOT TESTED (would invoke restart/reboot actions):")
        print("  " + ", ".join(intentionally_not_replayed))


def build_checks(oracle_dir: Path, capture_dir: Path) -> list[Check]:
    py = sys.executable
    rebuilt_walk_captures = tuple(sorted({
        *capture_dir.glob("rebuild_jsonl_*walk*.jsonl"),
        *capture_dir.glob("rebuild_walk*.jsonl"),
    }))
    checks = [
        check(
            "runtime-timing-contract",
            py, "tools/oracle/verify_runtime_timing_contract.py",
        ),
        check("set-pose-apk-formulae", py, "tools/oracle/verify_set_pose_model.py"),
        check("set-worker-java-jni", py, "tools/oracle/verify_set_worker_jni.py"),
        check(
            "original-apk-pose-to-pwm",
            py, "tools/oracle/compare_gt_kinematics.py", "--capture-dir", str(capture_dir),
        ),
        check(
            "original-vs-rebuild-runtime-captures",
            py, "tools/oracle/compare_apk_runtime_capture.py",
            "--capture-dir", str(capture_dir),
        ),
        check(
            "saved-rebuild-walk-replay",
            py, "tools/oracle/compare_walk_runtime_replay.py", *map(str, rebuilt_walk_captures),
            references=rebuilt_walk_captures,
        ),
        check(
            "gait-oracles",
            py, "tools/oracle/compare_gait_oracle.py",
            *(
                str(oracle_dir / name)
                for name in (
                    "api35_walk3_025_050_3_gaittrace.jsonl",
                    "api35_walk2_025_050_2_gaittrace.jsonl",
                    "api35_walk1_025_050_1_gaittrace.jsonl",
                    "api35_walk15_025_050_4_gaittrace.jsonl",
                    "api35_walk25_025_050_5_gaittrace.jsonl",
                    "api35_walkwave_025_050_6_gaittrace.jsonl",
                    "api35_crab_walk3_025_050_3_gaittrace.jsonl",
                )
            ),
            references=tuple(
                oracle_dir / name
                for name in (
                    "api35_walk3_025_050_3_gaittrace.jsonl",
                    "api35_walk2_025_050_2_gaittrace.jsonl",
                    "api35_walk1_025_050_1_gaittrace.jsonl",
                    "api35_walk15_025_050_4_gaittrace.jsonl",
                    "api35_walk25_025_050_5_gaittrace.jsonl",
                    "api35_walkwave_025_050_6_gaittrace.jsonl",
                    "api35_crab_walk3_025_050_3_gaittrace.jsonl",
                )
            ),
        ),
        check(
            "quad-runtime",
            py, "tools/oracle/compare_quad_runtime.py", "--oracle-dir", str(oracle_dir),
            references=tuple(
                oracle_dir / f"{prefix}{pair}{suffix}"
                for pair in ("03", "14", "25")
                for prefix, suffix in (
                    ("frida_original_quad_", "_setxy_0_09_trace.jsonl"),
                    ("marked_rebuilt_quad_", "_setxy_static_trace.jsonl"),
                )
            ),
        ),
        check(
            "walk-runtime-replay",
            py, "tools/oracle/compare_walk_runtime_replay.py",
            str(oracle_dir / "controltrace_walk_runtime_virtualtouch_original.jsonl"),
            str(oracle_dir / "controltrace_walkclear_dense_virtualtouch_original.jsonl"),
            references=(
                oracle_dir / "controltrace_walk_runtime_virtualtouch_original.jsonl",
                oracle_dir / "controltrace_walkclear_dense_virtualtouch_original.jsonl",
            ),
        ),
    ]

    animation_traces = (
        ("animation-torque-sit", "controltrace_torque_sit_virtualhw_anim.jsonl", "auto"),
        ("animation-quad-14", "controltrace_quad_14_keep_autositoff_logcat_anim.jsonl", "auto"),
        ("animation-quad-03", "controltrace_quad_03_verify_logcat_anim.jsonl", "auto"),
        ("animation-quad-25", "controltrace_quad_25_verify_logcat_anim.jsonl", "auto"),
        ("animation-bounce-jump", "controltrace_bounce_jump_autositoff_virtualhw_anim.jsonl", "impulse"),
    )
    checks.extend(
        check(
            label,
            py, "tools/oracle/replay_animation_trace.py", str(oracle_dir / filename),
            "--scenario", scenario, "--strict",
            references=(oracle_dir / filename,),
        )
        for label, filename, scenario in animation_traces
    )
    checks.extend(
        [
            check(
                "calibration-virtual-touch",
                py, "tools/oracle/compare_calibration_virtualtouch.py",
                str(oracle_dir / "controltrace_calibrate_virtualtouch_original.jsonl"),
                str(oracle_dir / "controltrace_calibrate_virtualtouch_rebuilt.jsonl"),
                references=(
                    oracle_dir / "controltrace_calibrate_virtualtouch_original.jsonl",
                    oracle_dir / "controltrace_calibrate_virtualtouch_rebuilt.jsonl",
                ),
            ),
            check(
                "ack-stand-ramp",
                py, "tools/oracle/compare_ack_stand_ramp.py",
                str(oracle_dir / "controltrace_ack_stand_ramp_virtualtouch_original.jsonl"),
                str(oracle_dir / "controltrace_ack_stand_ramp_virtualtouch_rebuilt.jsonl"),
                references=(
                    oracle_dir / "controltrace_ack_stand_ramp_virtualtouch_original.jsonl",
                    oracle_dir / "controltrace_ack_stand_ramp_virtualtouch_rebuilt.jsonl",
                ),
            ),
            check("no-hardware-servo", py, "tools/device/verify_no_hardware_servo.py"),
            check("servo2040-protocol", py, "tools/device/verify_servo2040_protocol.py"),
            check("pololu-protocol", py, "tools/device/verify_pololu_protocol.py"),
        ]
    )
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--oracle-dir", type=Path, default=DEFAULT_ORACLE_DIR)
    parser.add_argument("--capture-dir", type=Path, default=DEFAULT_CAPTURE_DIR)
    parser.add_argument(
        "--require-all-oracles", action="store_true",
        help="treat missing optional oracle fixtures as failures (default: report SKIP)",
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="show per-capture and per-command diagnostics",
    )
    args = parser.parse_args()
    oracle_dir = args.oracle_dir.expanduser().resolve()
    capture_dir = args.capture_dir.expanduser().resolve()

    (ROOT / "build").mkdir(exist_ok=True)
    animation_build = run_check(
        check(
            "build-animation-replay",
            "c++", "-std=c++17", "-O2", "-Iapp/src/main/cpp",
            "tools/oracle/animation_replay.cpp",
            "app/src/main/cpp/apk_model.cpp",
            "app/src/main/cpp/pulse_conversion.cpp",
            "-o", "build/animation_replay",
        ),
        require_all=True,
    )

    statuses = [animation_build]
    statuses.append(native_reference_differential(capture_dir))
    for item in build_checks(oracle_dir, capture_dir):
        if item.label == "original-apk-pose-to-pwm" and not list(capture_dir.glob("gt_jsonl_*.jsonl")):
            print(f"\n== {item.label} ==\nSKIP: no original APK JSONL captures in {capture_dir}")
            statuses.append("FAIL" if args.require_all_oracles else "SKIP")
            continue
        if item.label == "saved-rebuild-walk-replay" and not item.references:
            print(f"\n== {item.label} ==\nSKIP: no saved instrumented walk captures in {capture_dir}")
            statuses.append("FAIL" if args.require_all_oracles else "SKIP")
            continue
        if item.label.startswith("animation-") and animation_build != "PASS":
            print(f"\n== {item.label} ==\nSKIP: animation replay binary did not build")
            statuses.append("SKIP")
            continue
        statuses.append(run_check(item, args.require_all_oracles, args.verbose))

    runtime_command_coverage(capture_dir)

    counts = {name: statuses.count(name) for name in ("PASS", "BOUNDED", "SKIP", "FAIL")}
    print("\n== regression summary ==")
    print(" ".join(f"{name}={count}" for name, count in counts.items()))
    if counts["FAIL"]:
        print("result=FAIL (one or more available checks failed)")
        return 1
    if counts["BOUNDED"] or counts["SKIP"]:
        print("result=INCOMPLETE (some checks are bounded or lack oracle fixtures)")
        return 2
    print("result=PASS (all configured checks were exact)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
