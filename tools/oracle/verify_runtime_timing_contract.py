#!/usr/bin/env python3
"""Guard source-backed Android runtime timing that native replay cannot cover."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONTROLLER = ROOT / "app/src/main/java/com/makeyourpet/chicaserver/control/ChicaController.java"
ACTIVITY = ROOT / "app/src/main/java/com/makeyourpet/chicaserver/FullscreenActivity.java"
BACKEND = ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware/ServoBackend.java"
HARDWARE = ROOT / "app/src/main/java/com/makeyourpet/chicaserver/hardware"
CONFIG = ROOT / "app/src/main/assets/config-2040.txt"
EXPECTED_SPEEDS = {
    "MODE_STANDARD": 1.0,
    "MODE_RACE": 2.0,
    "MODE_OFFROAD": 0.6,
    "MODE_QUADRUPED": 0.8,
}


def mode_speeds(config: str) -> dict[str, float]:
    speeds: dict[str, float] = {}
    for raw in config.splitlines():
        line = raw.strip()
        if not line.startswith("MODE_"):
            continue
        fields = line.split()
        if len(fields) < 8:
            raise AssertionError(f"malformed mode line: {line}")
        speeds[fields[0]] = float(fields[7])
    return speeds


def constructor_body(source: str) -> str:
    match = re.search(
        r"public ChicaController\(ServoBackend servoBackend, String config\) \{(?P<body>.*?)\n    \}",
        source,
        re.DOTALL,
    )
    if not match:
        raise AssertionError("ChicaController constructor not found")
    return match.group("body")


def section(source: str, start: str, end: str) -> str:
    if start not in source or end not in source:
        raise AssertionError(f"source section not found: {start}")
    return source.split(start, 1)[1].split(end, 1)[0]


def main() -> int:
    source = CONTROLLER.read_text(encoding="utf-8")
    activity = ACTIVITY.read_text(encoding="utf-8")
    speeds = mode_speeds(CONFIG.read_text(encoding="utf-8"))
    for mode, expected in EXPECTED_SPEEDS.items():
        actual = speeds.get(mode)
        if actual != expected:
            raise AssertionError(f"{mode} speed={actual}, expected {expected}")

    gait_clock = re.compile(
        r"double dtMs = Math\.max\(0\.0d, now - lastStepMillis\)\s*"
        r"\*\s*currentMode\(\)\.speed;"
    )
    if not gait_clock.search(source):
        raise AssertionError("walk frame dt is not scaled by currentMode().speed")

    constructor = constructor_body(source)
    if "enterOriginalStartupPose();" not in constructor:
        raise AssertionError("startup pose call missing")
    after_startup = constructor.split("enterOriginalStartupPose();", 1)[1]
    before_monitor = after_startup.split("startOriginalHardwareMonitor();", 1)[0]
    if "setOriginalStanding(true" in before_monitor:
        raise AssertionError("startup incorrectly forces the robot to stand")

    publish = section(
        source,
        "private void publishOriginalFrame()",
        "private void requestOriginalWalkStop",
    )
    if "servoBackend.stageServoPulses(lastPulses);" not in publish:
        raise AssertionError("pose frames do not stage the latest servo target")
    if "setServoPulses" in publish or "flushServoPulses" in publish:
        raise AssertionError("pose worker performs synchronous servo I/O")

    gait_step = section(source, "private boolean stepOriginalGait()", "private void startOriginalWalkWorkerLocked")
    first_publish = gait_step.index("publishOriginalFrame();")
    first_trace = gait_step.index('Log.i("CHICA_GAIT"')
    if first_publish > first_trace:
        raise AssertionError("gait trace is emitted before its matching staged servo frame")

    heartbeat = section(
        source,
        "private void startOriginalHardwareMonitor()",
        "private static ModeParams[] buildModes",
    )
    heartbeat_contract = [
        "Thread.sleep(1L);",
        "now - lastHeartbeat > 7L",
        "servoBackend.flushServoPulses();",
        "(heartbeatCount % 2) == 0",
        "servoBackend.refreshTelemetry();",
        "servoBackend.isConnected() && !servoBackend.hasFault()",
        "bps = communicatedHeartbeats;",
    ]
    for statement in heartbeat_contract:
        if statement not in heartbeat:
            raise AssertionError(f"hardware heartbeat contract missing: {statement}")
    if "synchronized (servoBackend)" not in heartbeat:
        raise AssertionError("servo send and telemetry poll do not share the original backend lock")
    if heartbeat.index("flushServoPulses") > heartbeat.index("refreshTelemetry"):
        raise AssertionError("heartbeat must send servos before polling telemetry")

    walk_command = section(
        source,
        '} else if (command.startsWith("walk")) {',
        '} else if (command.startsWith("clear")) {',
    )
    if "boolean startingWalkWorker = !walkWorkerRunning;" not in walk_command:
        raise AssertionError("walk target updates do not preserve the active gait worker")
    worker_start = section(walk_command, "if (startingWalkWorker) {", "startOriginalWalkWorkerLocked();")
    if "filteredWalk =" not in worker_start or "lastStepMillis =" not in worker_start:
        raise AssertionError("gait filter/clock initialization is not limited to worker startup")
    if "gaitEngine.beginWalkSession();" not in worker_start:
        raise AssertionError("new walk worker does not reset its source-local phase/anchors")

    # d.n0 replaces f.h with a complete target; z0.o.b(true, ...) leaves an
    # already-running B/G worker and its dive/flex form unchanged. These guards
    # supplement paired APK entry captures; they do not prove trajectory parity.
    set_command = section(source, "private void applySetCommand(String command)",
                          "private void resetSetControls()")
    before_mapping = set_command.split('if (command.startsWith("setxy:"))', 1)[0]
    for axis in ("lastPrimaryX", "lastPrimaryY", "lastSecondaryX", "lastSecondaryY",
                 "lastTertiaryX", "lastTertiaryY"):
        if f"{axis} = 0.0d;" not in before_mapping:
            raise AssertionError(f"set command retains an omitted target axis: {axis}")
    latched_mode = re.search(
        r"if \(!setWorkerRunning\) \{(?P<body>.*?)\n        \}",
        set_command, re.DOTALL,
    )
    if latched_mode is None:
        raise AssertionError("set command does not latch its worker motion type")
    assignments = re.findall(r"setSweepMode\s*=\s*[012]\s*;", set_command)
    guarded_assignments = re.findall(r"setSweepMode\s*=\s*[012]\s*;", latched_mode["body"])
    if len(assignments) != 3 or assignments != guarded_assignments:
        raise AssertionError("active set command can replace its worker motion type")
    set_worker = section(source, "private void startOriginalSetWorkerLocked()",
                         "private void startOriginalQuadSetWorkerLocked()")
    for statement in (
        "final boolean sweepRoutine = setSweepMode != 0;",
        "final int workerSweepMode = sweepRoutine ? (setSweepMode == 1 ? 1 : 2) : 0;",
        "final boolean keepStaticPose = keepMode;",
        "double[] workerState = new double[7];",
        "double[] filteredSetPoseTarget = new double[6];",
        "advanceOriginalSetPoseFrame(workerSweepMode, workerState, filteredSetPoseTarget, dtMs);",
        "sleepOriginalPoseStep(10.0d);",
        "filteredSetPoseTarget[i] *= 0.9d;",
        "localSetTargetMagnitude(filteredSetPoseTarget) < 0.01d",
        "gaitEngine.keepSetPose();",
        "if (fadeLayer) publishOriginalLayerFade();",
    ):
        if statement not in set_worker:
            raise AssertionError(f"source set worker contract missing: {statement}")
    if "setPoseQuietFrames" in source or "setPoseLayerMagnitude" in set_worker:
        raise AssertionError("set release still stops on an invented output-layer rule")
    clear_set = section(source, "private void resetSetControls()", "private boolean originalMotionBusy()")
    if "setWorkerRunning = false;" not in clear_set:
        raise AssertionError("setclear does not release the motion flag immediately")
    if "if (!keepMode) publishOriginalLayerFade();" not in source:
        raise AssertionError("turning keep off does not fade the saved pose")
    motion_busy = section(source, "private boolean originalMotionBusy()", "private boolean isQuadOutputMode()")
    if "return activeWalk || setWorkerRunning;" not in motion_busy:
        raise AssertionError("motion busy differs from source flags during walkclear")
    sit = section(source, "private boolean setOriginalStanding", "private void setOriginalBlockMode")
    if "if (originalMotionBusy()) return false;" not in sit:
        raise AssertionError("sit is not guarded against active source motion flags")
    calibration = section(source, "private void runOriginalCalibrate()", "private static String animationTraceJson")
    for statement in ("gaitEngine.beginCalibration();", "while (true)",
                      "contacted[leg] = !Double.isNaN(touches[leg]) && touches[leg] > 0.5d;"):
        if statement not in calibration:
            raise AssertionError(f"source calibration contract missing: {statement}")

    if "ORIGINAL_STOP_VECTOR_THRESHOLD = 0.2d;" not in source:
        raise AssertionError("walk stop threshold differs from z0.e")
    gait_step = section(source, "private boolean stepOriginalGait()", "private void startOriginalWalkWorkerLocked")
    for statement in (
        "boolean transitioningToStop = pendingWalkClear;",
        "filteredWalk = scaleWalk(filteredWalk, 0.9d);",
        "!gaitEngine.hasActiveWalkAnchors()",
        "walkMagnitude(filteredWalk) > ORIGINAL_STOP_VECTOR_THRESHOLD",
    ):
        if statement not in gait_step:
            raise AssertionError(f"walk stop state machine missing: {statement}")
    if "if (!relayStatus || !standing" in gait_step:
        raise AssertionError("draining gait worker is incorrectly cancelled by sit/torque flags")
    ack_ramp = section(source, "private void handleAckRampLocked", "private void enterOriginalHomePose")
    if "publishOriginalHomePose(50.0d - ackCount);" not in ack_ramp:
        raise AssertionError("ACK home correction z0.o.g(50-ackCount) is missing")
    layer_fade = section(source, "private void publishOriginalLayerFade()", "private static double originalFrameDt")
    for statement in (
        "gaitEngine.beginLayerFadeContext()",
        "0.05000000074505806d",
        "currentMode().speed * 0.1d",
        "gaitEngine.finishLayerFadeContext(fadeContext)",
    ):
        if statement not in layer_fade:
            raise AssertionError(f"post-walk p3.a.p layer fade missing: {statement}")
    worker = section(source, "private void startOriginalWalkWorkerLocked()", "private void publishOriginalLayerFade()")
    if "keepMode && modeIndex == 4" not in worker or "if (!preserveKeptQuadrupedPose)" not in worker:
        raise AssertionError("kept quadruped gait does not preserve the source p3.a.p exception")

    backend = BACKEND.read_text(encoding="utf-8")
    if "void stageServoPulses(int[] pulses);" not in backend:
        raise AssertionError("backend has no non-I/O servo staging operation")
    if "void flushServoPulses();" not in backend:
        raise AssertionError("backend has no heartbeat servo flush operation")

    if "DEVELOPER_FIXTURES || BuildConfig.MOTION_TRACE_ENABLED;" not in source:
        raise AssertionError("hot-loop motion tracing is not disabled in the source-accurate build")

    refresh_worker = section(activity, "private final Runnable refreshStatus", "@Override\n    protected void attachBaseContext")
    if "refreshNow();" not in refresh_worker or "postDelayed(this, 83);" not in refresh_worker:
        raise AssertionError("UI state is not refreshed on the original 83ms timer")
    block_listener = section(activity, "findViewById(R.id.buttonBlock).setOnClickListener", "findViewById(R.id.buttonTorque)")
    if "refreshNow();" in block_listener:
        raise AssertionError("Block listener reads state before the asynchronous command completes")
    for tag in ("CHICA_SERVO", "CHICA_GAIT"):
        lines = [line.strip() for line in source.splitlines() if f'Log.i("{tag}"' in line]
        if not lines or any("MOTION_TRACE_ENABLED" not in line for line in lines):
            raise AssertionError(f"{tag} logging is not guarded outside developer fixtures")
    guarded_anim_logs = re.findall(
        r"if \(MOTION_TRACE_ENABLED\) \{\s*Log\.i\(\"CHICA_ANIM\"",
        source,
    )
    if len(guarded_anim_logs) != source.count('Log.i("CHICA_ANIM"'):
        raise AssertionError("CHICA_ANIM logging is not guarded outside developer fixtures")

    usb = (HARDWARE / "UsbSerialServoBackend.java").read_text(encoding="utf-8")
    if "port.write(frame, protocol == BoardProtocol.SERVO2040 ? 0 : TIMEOUT_MS);" not in usb:
        raise AssertionError("Servo2040 USB writes do not use the original zero timeout")
    for name in ("UsbSerialServoBackend.java", "SocketServo2040Backend.java", "TtyServo2040Backend.java"):
        backend_source = (HARDWARE / name).read_text(encoding="utf-8")
        if "if (stagedServoFrame == null) stagedServoFrame = new byte[39];" not in backend_source:
            raise AssertionError(f"{name} does not reuse its staged packet buffer")
        if "Pulses(pulses, stagedServoFrame);" not in backend_source:
            raise AssertionError(f"{name} allocates while staging servo pulses")

    rendered = ", ".join(f"{name}={speeds[name]:g}" for name in EXPECTED_SPEEDS)
    print(
        "runtime source contract passed (not a timing measurement) "
        f"({rendered}; startup=seated; servo heartbeat=>7ms; telemetry=every-other; "
        "connected-bps; ui=83ms; persistent-walk-worker; latched-set-worker; complete-set-target; "
        "anchor-clear-stop; phase-reset; layer-fade; ack-home; trace=off)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
