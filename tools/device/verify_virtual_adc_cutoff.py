#!/usr/bin/env python3
"""Verify virtual ADC warning/cutoff behavior through the TCP protocol.

This requires a running Android emulator. It installs the rebuilt APK by
default, injects virtual ADC values, and checks that sustained low voltage or
high current clears the relay flag through the normal controller/status path.
"""

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
V_RE = re.compile(r"\|V=\s*([0-9.]+|---)")
I_RE = re.compile(r"\|I=\s*([0-9.]+|---)")


SCENARIOS = {
    "low_voltage": [
        "virtualtouch:0.5",
        "torque:0.5",
        "virtualadc:0.000,7.400:4.5",
    ],
    "high_current": [
        "virtualtouch:0.5",
        "torque:0.5",
        "virtualadc:7.400,20.000:4.5",
    ],
}


def run_capture(apk: Path, outdir: Path, name: str, steps: list[str]) -> Path:
    out = outdir / f"rebuilt_{name}.jsonl"
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
    for step in steps:
        command += ["--step", step]
    last_error: subprocess.CalledProcessError | None = None
    for attempt in range(1, 4):
        subprocess.run(["adb", "forward", "--remove", "tcp:18711"], cwd=ROOT,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            subprocess.run(command, cwd=ROOT, check=True)
            return out
        except subprocess.CalledProcessError as error:
            last_error = error
            print(f"{name}: capture attempt {attempt} failed", file=sys.stderr)
    if last_error is not None:
        raise last_error
    return out


def load(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def tcp_lines(rows: list[dict]) -> list[str]:
    return [row.get("line", "") for row in rows if row.get("type") == "tcp"]


def flags(lines: list[str]) -> list[str]:
    out: list[str] = []
    for line in lines:
        match = FLAGS_RE.search(line)
        if not match:
            continue
        value = match.group(1)
        if not out or out[-1] != value:
            out.append(value)
    return out


def numeric_values(lines: list[str], regex: re.Pattern[str]) -> list[float]:
    out: list[float] = []
    for line in lines:
        match = regex.search(line)
        if not match or match.group(1) == "---":
            continue
        out.append(float(match.group(1)))
    return out


def verify(path: Path, scenario: str) -> None:
    lines = tcp_lines(load(path))
    path_flags = flags(lines)
    if "100000100" not in path_flags:
        raise AssertionError(f"{scenario}: relay never turned on: {path_flags}")
    if not path_flags or path_flags[-1] != "000000100":
        raise AssertionError(f"{scenario}: relay did not cut off: {path_flags}")

    voltages = numeric_values(lines, V_RE)
    currents = numeric_values(lines, I_RE)
    if scenario == "low_voltage":
        if not any(value <= 0.05 for value in voltages):
            raise AssertionError(f"{scenario}: injected low voltage not observed: {voltages}")
    elif scenario == "high_current":
        if not any(value >= 19.95 for value in currents):
            raise AssertionError(f"{scenario}: injected high current not observed: {currents}")
    print(f"{scenario}: flags={path_flags} cutoff=true")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apk", type=Path, default=DEFAULT_APK)
    parser.add_argument("--outdir", type=Path, default=Path("/tmp/chica-adc-cutoff"))
    args = parser.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)
    for name, steps in SCENARIOS.items():
        path = run_capture(args.apk, args.outdir, name, steps)
        verify(path, name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
