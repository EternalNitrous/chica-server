#!/usr/bin/env python3
"""Capture original APK motion primitive traces with Frida.

The output is JSONL. It intentionally keeps original obfuscated method names
(`z0.o.g`, `p3.a.M`, etc.) so the trace can drive a structural compat port
before any semantic renaming.
"""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterable

import frida


PACKAGE = "com.makeyourpet.chicaserver"
PORT = 18711


SCRIPT = r"""
function now() {
  return Date.now();
}

function emit(kind, data) {
  send({ kind: kind, time_ms: now(), data: data });
}

function jstr(value) {
  if (value === null || value === undefined) return null;
  try { return value.toString(); } catch (_) { return String(value); }
}

function className(value) {
  if (value === null || value === undefined) return null;
  try { return value.getClass().getName().toString(); } catch (_) { return null; }
}

Java.perform(function () {
  var Z0M = Java.use("z0.m");
  var Z0N = Java.use("z0.n");
  var P3A = Java.use("p3.a");

  function vec(v) {
    if (v === null || v === undefined) return null;
    return {
      x: Number(v.f7144a.value),
      y: Number(v.f7145b.value),
      z: Number(v.f7146c.value)
    };
  }

  function intArray(arr) {
    if (arr === null || arr === undefined) return null;
    var out = [];
    for (var i = 0; i < arr.length; i++) out.push(Number(arr[i]));
    return out;
  }

  function boolArray(arr) {
    if (arr === null || arr === undefined) return null;
    var out = [];
    for (var i = 0; i < arr.length; i++) out.push(Boolean(arr[i]));
    return out;
  }

  function vecArray(arr) {
    if (arr === null || arr === undefined) return null;
    var out = [];
    for (var i = 0; i < arr.length; i++) out.push(vec(arr[i]));
    return out;
  }

  function pose(p) {
    if (p === null || p === undefined) return null;
    try {
      return {
        kind: "p3.a",
        selector: Number(p.f5897d.value),
        xyz: vec(p.f5898e.value),
        uvw: vec(p.f5899f.value)
      };
    } catch (e) {
      return {
        kind: "p3.a",
        selector: safeNumberField(p, "f5897d"),
        f5898eClass: className(safeField(p, "f5898e")),
        f5899fClass: className(safeField(p, "f5899f")),
        text: jstr(p)
      };
    }
  }

  function layers(p) {
    if (p === null || p === undefined) return null;
    var out = [];
    try {
      var arr = Java.cast(p.f5898e.value, Java.use("[Lp3.a;"));
      for (var i = 0; i < arr.length; i++) out.push(pose(arr[i]));
      return out;
    } catch (e) {
      return { error: jstr(e), text: jstr(p) };
    }
  }

  function stateN(n) {
    if (n === null || n === undefined) return null;
    var feet = [];
    try {
      var arr = n.f7148b.value;
      for (var i = 0; i < arr.length; i++) feet.push(vec(arr[i]));
      return {
        body: pose(n.f7147a.value),
        feet: feet,
        text: jstr(n)
      };
    } catch (e) {
      return { error: jstr(e), text: jstr(n) };
    }
  }

  function safeField(obj, name) {
    try { return obj[name].value; } catch (_) { return null; }
  }

  function safeNumberField(obj, name) {
    try { return Number(obj[name].value); } catch (_) { return null; }
  }

  function robotState(z0a) {
    if (z0a === null || z0a === undefined) return null;
    try {
      var snapshot = Z0N.$new();
      z0a.e(snapshot);
      return {
        activeOrder: intArray(z0a.b()),
        activeMask: boolArray(z0a.c()),
        inactiveOrder: intArray(z0a.f7056f.value),
        layers: layers(z0a.d()),
        state: stateN(snapshot)
      };
    } catch (e) {
      return { error: jstr(e), className: className(z0a), text: jstr(z0a) };
    }
  }

  function motionThis(obj) {
    if (obj === null || obj === undefined) return null;
    var selector = safeNumberField(obj, "f5897d");
    var left = safeField(obj, "f5898e");
    var right = safeField(obj, "f5899f");
    var out = {
      selector: selector,
      f5898eClass: className(left),
      f5899fClass: className(right)
    };
    if (className(left) === "z0.a") {
      out.robot = robotState(left);
    } else if (selector === 6) {
      out.pose = pose(obj);
    } else if (selector === 7) {
      out.layers = layers(obj);
    } else {
      out.text = jstr(obj);
    }
    return out;
  }

  function z0oState(obj) {
    if (obj === null || obj === undefined) return null;
    return {
      torque: Boolean(obj.f7149a.value),
      block: Boolean(obj.f7150b.value),
      calib: Boolean(obj.f7151c.value),
      standing: Boolean(obj.f7152d.value),
      autosit: Boolean(obj.f7153e.value),
      level: Boolean(obj.f7154f.value),
      bodyActive: Boolean(obj.f7155g.value),
      keep: Boolean(obj.f7157i.value),
      walkActive: Boolean(obj.f7158j.value),
      gaitMode: Number(obj.f7159k.value),
      style: Number(obj.l.value),
      crab: Boolean(obj.f7160m.value),
      mode: Number(obj.f7161n.value)
    };
  }

  function argValue(arg) {
    if (arg === null || arg === undefined) return null;
    var c = className(arg);
    if (c === "z0.n") return stateN(arg);
    if (c === "z0.m") return vec(arg);
    if (c === "p3.a") return pose(arg);
    if (c === "[I") return intArray(arg);
    if (c === "[Z") return boolArray(arg);
    if (c === "[Lz0.m;") return vecArray(arg);
    if (typeof arg === "number" || typeof arg === "boolean" || typeof arg === "string") return arg;
    return { className: c, text: jstr(arg) };
  }

  function hookAll(clazzName, methodName, label, mapper) {
    try {
      var C = Java.use(clazzName);
      C[methodName].overloads.forEach(function (overload) {
        overload.implementation = function () {
          var args = Array.prototype.slice.call(arguments);
          var before = null;
          try { before = mapper.call(this, "before", args, null); } catch (e) { before = { error: jstr(e) }; }
          emit(label + ".before", before);
          var result = overload.apply(this, args);
          var after = null;
          try { after = mapper.call(this, "after", args, result); } catch (e) { after = { error: jstr(e) }; }
          emit(label + ".after", after);
          return result;
        };
      });
      emit("hooked", { class: clazzName, method: methodName, overloads: C[methodName].overloads.length });
    } catch (e) {
      emit("hook_error", { class: clazzName, method: methodName, error: jstr(e) });
    }
  }

  function mapP3(phase, args, result) {
    return {
      phase: phase,
      self: motionThis(this),
      args: args.map(argValue),
      result: argValue(result)
    };
  }

  function mapZ0O(phase, args, result) {
    return {
      phase: phase,
      state: z0oState(this),
      args: args.map(argValue),
      result: argValue(result)
    };
  }

  [
    ["p3.a", "M", "p3.a.M"],
    ["p3.a", "k", "p3.a.k"],
    ["p3.a", "l", "p3.a.l"],
    ["p3.a", "p", "p3.a.p"],
    ["p3.a", "A", "p3.a.A"],
    ["p3.a", "B", "p3.a.B"],
    ["p3.a", "C", "p3.a.C"],
    ["p3.a", "G", "p3.a.G"]
  ].forEach(function (spec) { hookAll(spec[0], spec[1], spec[2], mapP3); });

  [
    ["z0.o", "a", "z0.o.a"],
    ["z0.o", "c", "z0.o.c"],
    ["z0.o", "d", "z0.o.d"],
    ["z0.o", "e", "z0.o.e"],
    ["z0.o", "f", "z0.o.f"],
    ["z0.o", "g", "z0.o.g"],
    ["z0.o", "i", "z0.o.i"]
  ].forEach(function (spec) { hookAll(spec[0], spec[1], spec[2], mapZ0O); });

  function hookServo(clazzName, methodName, label) {
    hookAll(clazzName, methodName, label, function (phase, args, result) {
      if (phase !== "before") return { phase: phase };
      return { phase: phase, pulses: intArray(args[0]) };
    });
  }
  hookServo("e4.g", "l", "e4.g.l");
  hookServo("e4.f", "l", "e4.f.l");

  emit("ready", { package: "com.makeyourpet.chicaserver" });
});
"""


def adb(*args: str, check: bool = True) -> str:
    proc = subprocess.run(["adb", *args], text=True, capture_output=True, check=check)
    return proc.stdout.strip()


def ensure_app_running(restart: bool) -> None:
    if restart:
        adb("shell", "am", "force-stop", PACKAGE, check=False)
        time.sleep(0.25)
    elif adb("shell", "pidof", PACKAGE, check=False):
        return
    adb("shell", "monkey", "-p", PACKAGE, "-c", "android.intent.category.LAUNCHER", "1")
    deadline = time.time() + 8.0
    while time.time() < deadline:
        if adb("shell", "pidof", PACKAGE, check=False):
            return
        time.sleep(0.25)
    raise RuntimeError(f"{PACKAGE} did not start")


def resolve_pid(device: frida.core.Device, package: str) -> int:
    for app in device.enumerate_applications():
        if app.identifier == package and app.pid:
            return app.pid
    for process in device.enumerate_processes():
        if process.name == package or process.name == "Chica Server":
            return process.pid
    raise RuntimeError(f"unable to find running process for {package}")


def tcp_replay(steps: Iterable[tuple[str, float]], ack_interval: float) -> list[dict]:
    adb("forward", f"tcp:{PORT}", f"tcp:{PORT}")
    records: list[dict] = []
    sock = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        try:
            candidate = socket.create_connection(("127.0.0.1", PORT), timeout=2.0)
            candidate.settimeout(0.1)
            greeting = candidate.recv(8192)
            sock = candidate
            for line in greeting.decode(errors="replace").splitlines():
                records.append({"type": "tcp", "line": line, "hostTime": time.time()})
            break
        except OSError:
            time.sleep(0.25)
    if sock is None:
        raise RuntimeError("TCP server did not become ready")

    def drain() -> None:
        while True:
            try:
                data = sock.recv(8192)
            except socket.timeout:
                return
            except OSError:
                return
            if not data:
                return
            for line in data.decode(errors="replace").splitlines():
                records.append({"type": "tcp", "line": line, "hostTime": time.time()})

    last_ack = 0.0
    for command, delay in steps:
        drain()
        records.append({"type": "command", "command": command, "hostTime": time.time()})
        sock.sendall((command + "\n").encode())
        end = time.time() + delay
        while time.time() < end:
            now = time.time()
            if now - last_ack >= ack_interval:
                sock.sendall(b"ack\n")
                last_ack = now
            drain()
            time.sleep(0.01)
    drain()
    sock.close()
    return records


def parse_steps(raw_steps: list[str]) -> list[tuple[str, float]]:
    out: list[tuple[str, float]] = []
    for raw in raw_steps:
        if ":" in raw:
            command, delay_text = raw.rsplit(":", 1)
            try:
                out.append((command, float(delay_text)))
                continue
            except ValueError:
                pass
        out.append((raw, 0.5))
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parents[4]
        / "research-private" / "oracle" / "animation_primitives.jsonl",
    )
    parser.add_argument("--step", action="append", default=[])
    parser.add_argument("--ack-interval", type=float, default=0.10)
    parser.add_argument("--no-restart", action="store_true")
    args = parser.parse_args()

    steps = parse_steps(args.step) if args.step else [
        ("torque", 0.8),
        ("sit", 2.5),
        ("home", 1.5),
        ("race", 3.0),
        ("offroad", 4.0),
    ]

    ensure_app_running(restart=not args.no_restart)
    device = frida.get_usb_device(timeout=8)
    session = device.attach(resolve_pid(device, PACKAGE))
    script = session.create_script(SCRIPT)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8") as out:
        def write(record: dict) -> None:
            out.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
            out.flush()

        def on_message(message, data) -> None:
            write({"type": "frida", "message": message})
            if message.get("type") == "send":
                payload = message.get("payload", {})
                print(json.dumps(payload, separators=(",", ":"), sort_keys=True))
            else:
                print(json.dumps(message, sort_keys=True), file=sys.stderr)

        script.on("message", on_message)
        script.load()
        time.sleep(1.0)
        for record in tcp_replay(steps, args.ack_interval):
            write(record)
        time.sleep(2.0)

    session.detach()
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
