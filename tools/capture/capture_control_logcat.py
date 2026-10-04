#!/usr/bin/env python3
"""Capture Chica TCP control/logcat traces for arbitrary command sequences."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import queue
import re
import socket
import subprocess
import threading
import time
from pathlib import Path


PACKAGE = "com.makeyourpet.chicaserver"
PORT = 18711
LOG_RE = re.compile(r"^\s*(\d+\.\d+)\s+\d+\s+\d+\s+\w\s+([^:]+):\s?(.*)$")


def adb(*args: str, check: bool = True) -> str:
    proc = subprocess.run(["adb", *args], text=True, capture_output=True, check=check)
    return proc.stdout.strip()


def install_apk(apk: Path | None, package: str = PACKAGE) -> None:
    if apk is None:
        return
    proc = subprocess.run(["adb", "install", "-r", str(apk)], text=True, capture_output=True)
    if proc.returncode == 0:
        return
    adb("uninstall", package, check=False)
    subprocess.run(["adb", "install", str(apk)], check=True)


def start_app(package: str = PACKAGE) -> None:
    adb("shell", "am", "force-stop", package, check=False)
    time.sleep(0.3)
    adb("logcat", "-G", "32M", check=False)
    adb("logcat", "-c", check=False)
    adb("shell", "monkey", "-p", package, "-c", "android.intent.category.LAUNCHER", "1")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if adb("shell", "pidof", package, check=False):
            return
        time.sleep(0.25)
    raise RuntimeError(f"{package} did not start")


def parse_log_line(raw: str) -> dict | None:
    match = LOG_RE.match(raw.rstrip())
    if not match:
        return None
    timestamp = float(match.group(1))
    tag = match.group(2).strip()
    message = match.group(3).strip()
    if tag == "CHICA_SERVO":
        try:
            return {"type": "servo_values", "time": timestamp, "values": ast.literal_eval(message)}
        except (SyntaxError, ValueError):
            return {"type": "log", "time": timestamp, "tag": tag, "message": message}
    if message.startswith("{"):
        try:
            record = json.loads(message)
            record.setdefault("time", timestamp)
            if tag == "CHICA_GAIT":
                record.setdefault("type", "gait")
            elif tag == "CHICA_CONTROL":
                record.setdefault("type", "control")
            elif tag == "CHICA_COMMAND":
                record.setdefault("type", "device_command")
            elif tag == "CHICA_FRAME":
                record.setdefault("type", "frame")
            elif tag == "CHICA_STATE":
                record.setdefault("type", "state")
            elif tag == "CHICA_ANIM":
                record.setdefault("type", "anim")
            return record
        except json.JSONDecodeError:
            pass
    if tag == "CHICA_MARK":
        return {"type": "mark", "time": timestamp, "message": message}
    return {"type": "log", "time": timestamp, "tag": tag, "message": message}


def start_logcat(records: queue.Queue[dict]) -> tuple[subprocess.Popen, threading.Thread]:
    proc = subprocess.Popen(
        [
            "adb",
            "logcat",
            "-v",
            "epoch",
            "CHICA_ANIM:I",
            "CHICA_GT:D",
            "CHICA_GAIT:I",
            "CHICA_SERVO:I",
            "CHICA_FRAME:I",
            "CHICA_CONTROL:I",
            "CHICA_STATE:I",
            "CHICA_COMMAND:I",
            "CHICA_MARK:I",
            "*:S",
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=1,
    )

    def reader() -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            record = parse_log_line(line)
            if record is not None:
                records.put(record)

    reader_thread = threading.Thread(target=reader, daemon=True)
    reader_thread.start()
    return proc, reader_thread


def connect_tcp() -> tuple[socket.socket, list[dict]]:
    adb("forward", f"tcp:{PORT}", f"tcp:{PORT}")
    deadline = time.time() + 12.0
    last_error: OSError | None = None
    while time.time() < deadline:
        try:
            sock = socket.create_connection(("127.0.0.1", PORT), timeout=2.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(0.05)
            try:
                greeting = sock.recv(8192)
            except socket.timeout:
                greeting = b""
            records = [
                {
                    "type": "tcp",
                    "line": line,
                    "hostTime": time.time(),
                    "responseSequence": 0,
                    "requestSequence": None,
                    "greeting": True,
                }
                for line in greeting.decode(errors="replace").splitlines()
            ]
            if any(record["line"].startswith("ready:") for record in records):
                return sock, records
            sock.close()
        except OSError as error:
            last_error = error
        time.sleep(0.25)
    raise RuntimeError(f"TCP server did not become ready: {last_error}")


def drain_tcp(
    sock: socket.socket,
    records: list[dict],
    response_sequence: list[int],
    request_sequence: list[int],
) -> None:
    timeout = sock.gettimeout()
    sock.setblocking(False)
    try:
        while True:
            try:
                data = sock.recv(8192)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            if not data:
                return
            for line in data.decode(errors="replace").splitlines():
                response_sequence[0] += 1
                sequence = response_sequence[0]
                records.append({
                    "type": "tcp",
                    "line": line,
                    "hostTime": time.time(),
                    "responseSequence": sequence,
                    "requestSequence": sequence if sequence <= request_sequence[0] else None,
                })
    finally:
        sock.settimeout(timeout)


def parse_steps(raw_steps: list[str]) -> list[tuple[str, float]]:
    steps: list[tuple[str, float]] = []
    for raw in raw_steps:
        if ":" in raw:
            command, delay_text = raw.rsplit(":", 1)
            try:
                steps.append((command, float(delay_text)))
                continue
            except ValueError:
                pass
        steps.append((raw, 0.5))
    return steps


def drive_tcp(
    steps: list[tuple[str, float]],
    ack_interval: float,
    wait_ready_before_command: bool,
    ready_settle_ms: float,
    no_heartbeat: bool,
    settle_before_command_ms: float,
    status_probe_offsets_ms: list[float],
    empty_line_keepalive: bool = False,
) -> list[dict]:
    sock, records = connect_tcp()
    response_sequence = [0]
    request_sequence = [0]
    drain_tcp(sock, records, response_sequence, request_sequence)
    marker_queue: queue.Queue[tuple[int, str] | None] = queue.Queue()
    marker_sequence = [0]

    def write_markers() -> None:
        while True:
            item = marker_queue.get()
            if item is None:
                return
            marker_id, command = item
            started = time.time()
            adb("shell", "log", "-t", "CHICA_MARK", f"send#{marker_id}:{command}", check=False)
            records.append({
                "type": "marker_host",
                "markerId": marker_id,
                "command": command,
                "hostStart": started,
                "hostEnd": time.time(),
            })

    marker_thread = threading.Thread(target=write_markers, daemon=True)
    marker_thread.start()

    def send_line(line: str, origin: str, parent_command_sequence: int | None = None) -> float:
        request_sequence[0] += 1
        sequence = request_sequence[0]
        send_time = time.time()
        sock.sendall((line + "\n").encode())
        record = {
            "type": "tcp_send",
            "line": line,
            "origin": origin,
            "requestSequence": sequence,
            "hostTime": send_time,
        }
        if parent_command_sequence is not None:
            record["parentCommandSequence"] = parent_command_sequence
        records.append(record)
        return send_time

    last_ack = 0.0
    last_empty_keepalive = time.monotonic()

    def keep_connection_alive() -> None:
        nonlocal last_empty_keepalive
        if empty_line_keepalive and time.monotonic() - last_empty_keepalive >= 0.25:
            # Original TCP readLine() times out after 1000 ms. Empty lines take
            # the unmatched-command path and produce a reply without invoking
            # the pose-changing ACK handler. Keep them in request/reply ordering.
            send_line("", "empty_keepalive")
            last_empty_keepalive = time.monotonic()

    def wait_for_ready() -> None:
        nonlocal last_ack
        deadline = time.monotonic() + 20.0
        consecutive_ready = 0
        while time.monotonic() < deadline:
            send_line("ack", "ready_gate")
            sequence = request_sequence[0]
            last_ack = time.monotonic()
            response_deadline = time.monotonic() + 2.0
            response = None
            while time.monotonic() < response_deadline:
                drain_tcp(sock, records, response_sequence, request_sequence)
                response = next((
                    record["line"] for record in reversed(records)
                    if record.get("type") == "tcp"
                    and record.get("requestSequence") == sequence
                ), None)
                if response is not None:
                    break
                time.sleep(0.002)
            if response is None:
                raise RuntimeError(f"no TCP response for READY gate request {sequence}")
            if response.startswith("ready:"):
                consecutive_ready += 1
            else:
                consecutive_ready = 0
            if consecutive_ready >= 2:
                # Each READY ACK reply follows a 100 ms server wait. The second
                # successful probe confirms the first ACK worker released before
                # the next one; let that final worker finish before the command.
                quiet_until = time.monotonic() + max(ready_settle_ms / 1000.0, ack_interval)
                while time.monotonic() < quiet_until:
                    drain_tcp(sock, records, response_sequence, request_sequence)
                    time.sleep(0.002)
                return
        raise RuntimeError("server did not return READY for 20 seconds")

    try:
        for step_index, (command, delay) in enumerate(steps):
            if step_index and settle_before_command_ms > 0:
                settle_until = time.monotonic() + settle_before_command_ms / 1000.0
                while time.monotonic() < settle_until:
                    keep_connection_alive()
                    drain_tcp(sock, records, response_sequence, request_sequence)
                    time.sleep(min(0.005, max(0.0, settle_until - time.monotonic())))
            drain_tcp(sock, records, response_sequence, request_sequence)
            if wait_ready_before_command:
                wait_for_ready()
            try:
                send_time = send_line(command, "command")
            except OSError as error:
                records.append({
                    "type": "capture_warning",
                    "message": f"TCP send failed for command {command!r}: {error}",
                    "hostTime": time.time(),
                })
                break
            command_sequence = request_sequence[0]
            marker_sequence[0] += 1
            marker_id = marker_sequence[0]
            records.append({
                "type": "command",
                "command": command,
                "hostTime": send_time,
                "requestSequence": request_sequence[0],
                "markerId": marker_id,
            })
            marker_queue.put((marker_id, command))
            deadline = time.monotonic() + delay
            command_start = time.monotonic()
            probe_deadlines = [command_start + offset / 1000.0 for offset in status_probe_offsets_ms]
            probe_index = 0
            probes_stopped = False
            probe_failed = False
            while time.monotonic() < deadline:
                now = time.monotonic()
                try:
                    keep_connection_alive()
                except OSError as error:
                    records.append({
                        "type": "capture_warning",
                        "message": f"TCP empty-line keepalive failed: {error}",
                        "hostTime": time.time(),
                    })
                    probe_failed = True
                    break
                if (
                    not no_heartbeat
                    and now - last_ack >= ack_interval
                ):
                    try:
                        send_line("ack", "ack")
                    except OSError as error:
                        records.append({
                            "type": "capture_warning",
                            "message": f"TCP heartbeat send failed: {error}",
                            "hostTime": time.time(),
                        })
                        probe_failed = True
                        break
                    last_ack = now
                if not probes_stopped and probe_index < len(probe_deadlines) and now >= probe_deadlines[probe_index]:
                    offset_ms = status_probe_offsets_ms[probe_index]
                    try:
                        send_line("ack", "status_probe", command_sequence)
                    except OSError as error:
                        records.append({
                            "type": "capture_warning",
                            "message": f"TCP status-probe send failed: {error}",
                            "hostTime": time.time(),
                        })
                        probe_failed = True
                        break
                    probe_sequence = request_sequence[0]
                    records.append({
                        "type": "status_probe",
                        "afterCommandSequence": command_sequence,
                        "requestSequence": probe_sequence,
                        "probeIndex": probe_index,
                        "offsetMs": offset_ms,
                        "hostTime": time.time(),
                    })
                    probe_index += 1
                    probe_response_deadline = time.monotonic() + 2.0
                    probe_response = None
                    while time.monotonic() < probe_response_deadline:
                        drain_tcp(sock, records, response_sequence, request_sequence)
                        probe_response = next((
                            record.get("line") for record in reversed(records)
                            if record.get("type") == "tcp"
                            and record.get("requestSequence") == probe_sequence
                        ), None)
                        if probe_response is not None:
                            break
                        time.sleep(0.002)
                    if probe_response is None:
                        records.append({
                            "type": "capture_warning",
                            "message": f"no TCP response for status probe {probe_sequence}",
                            "requestSequence": probe_sequence,
                            "responsesSeen": response_sequence[0],
                            "hostTime": time.time(),
                        })
                        probe_failed = True
                        break
                    if probe_response.startswith("ready:"):
                        # A READY ACK is accepted and advances ACK-ramp behavior;
                        # stop probing so the harness does not queue more work.
                        probes_stopped = True
                drain_tcp(sock, records, response_sequence, request_sequence)
                time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))
            if probe_failed:
                break
        drain_tcp(sock, records, response_sequence, request_sequence)
        response_deadline = time.monotonic() + 5.0
        while response_sequence[0] < request_sequence[0] and time.monotonic() < response_deadline:
            drain_tcp(sock, records, response_sequence, request_sequence)
            time.sleep(0.005)
        if response_sequence[0] < request_sequence[0]:
            records.append({
                "type": "capture_warning",
                "message": "TCP replies remained outstanding at capture end",
                "outstanding": request_sequence[0] - response_sequence[0],
                "hostTime": time.time(),
            })
    finally:
        marker_queue.put(None)
        marker_thread.join()
        sock.close()
    return records


def flush_queue(records: queue.Queue[dict]) -> list[dict]:
    out: list[dict] = []
    while True:
        try:
            out.append(records.get_nowait())
        except queue.Empty:
            return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--apk", type=Path)
    parser.add_argument("--step", action="append", required=True)
    parser.add_argument("--ack-interval", type=float, default=0.10)
    parser.add_argument(
        "--wait-ready-before-command", action="store_true",
        help="legacy ACK-based gate; accepted probes advance ACK-ramp behavior",
    )
    parser.add_argument(
        "--ready-settle-ms", type=float, default=250.0,
        help="quiet time after two ACK-based READY probes (each probe may advance ACK behavior)",
    )
    parser.add_argument(
        "--no-heartbeat", action="store_true",
        help="do not send periodic ACKs during command delays; useful for isolated command captures",
    )
    parser.add_argument(
        "--empty-line-keepalive", action="store_true",
        help="with --no-heartbeat, send empty lines every 250 ms to preserve the original TCP connection",
    )
    parser.add_argument(
        "--settle-before-command-ms", type=float, default=0.0,
        help="quiet wait before each command after the first; does not send ACK probes",
    )
    parser.add_argument(
        "--status-probe-after-ms", type=float, action="append", default=[],
        help="send one ACK status probe at this offset after each command; repeat to bracket BUSY release",
    )
    parser.add_argument("--package", default=PACKAGE,
                        help="installed package to drive")
    args = parser.parse_args()
    if args.empty_line_keepalive and (not args.no_heartbeat or args.wait_ready_before_command):
        parser.error("empty-line keepalives require --no-heartbeat and no ACK-based READY gate")

    steps = parse_steps(args.step)
    if args.settle_before_command_ms < 0 or args.ready_settle_ms < 0:
        parser.error("settle durations must be non-negative")
    if (
        any(offset < 0 for offset in args.status_probe_after_ms)
        or args.status_probe_after_ms != sorted(set(args.status_probe_after_ms))
        or any(offset >= delay * 1000.0 for offset in args.status_probe_after_ms for _, delay in steps)
    ):
        parser.error("status probe offsets must be unique, increasing, and before every command delay ends")

    # Record the installed artifact before another build can replace its path.
    apk_sha256 = hashlib.sha256(args.apk.read_bytes()).hexdigest() if args.apk else None
    install_apk(args.apk, args.package)
    start_app(args.package)
    log_records: queue.Queue[dict] = queue.Queue()
    proc, reader_thread = start_logcat(log_records)
    time.sleep(3.0)
    tcp_records = drive_tcp(
        steps, args.ack_interval, args.wait_ready_before_command, args.ready_settle_ms,
        args.no_heartbeat, args.settle_before_command_ms, args.status_probe_after_ms,
        args.empty_line_keepalive,
    )
    tcp_records.insert(0, {
        "type": "capture_meta",
        "schema": "chica-runtime-capture-v2",
        "package": args.package,
        "apkPath": str(args.apk.resolve()) if args.apk else None,
        "apkSha256": apk_sha256,
        "waitReadyBeforeCommand": args.wait_ready_before_command,
        "readySettleMs": args.ready_settle_ms,
        "ackIntervalMs": args.ack_interval * 1000.0,
        "heartbeatEnabled": not args.no_heartbeat,
        "emptyLineKeepalive": args.empty_line_keepalive,
        "settleBeforeCommandMs": args.settle_before_command_ms,
        "statusProbeAfterMs": args.status_probe_after_ms or None,
        "steps": [{"command": command, "delayMs": delay * 1000.0} for command, delay in steps],
        "hostTime": time.time(),
    })
    time.sleep(1.0)
    proc.terminate()
    try:
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)
    reader_thread.join(timeout=2.0)
    if reader_thread.is_alive():
        raise RuntimeError("logcat reader did not drain after logcat exited")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8") as out:
        for record in tcp_records + flush_queue(log_records):
            out.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
