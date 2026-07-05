package com.makeyourpet.chicaserver.control;
import android.util.Log;
import com.makeyourpet.chicaserver.BuildConfig;
import com.makeyourpet.chicaserver.gait.ChicaGaitEngine;
import com.makeyourpet.chicaserver.hardware.ChicaServoCalibration;
import com.makeyourpet.chicaserver.hardware.ServoBackend;
import com.makeyourpet.chicaserver.hardware.VirtualServoBackend;
import java.util.Arrays;
import java.util.Locale;
import java.util.Scanner;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class ChicaController {
    // Rebuilt-only developer commands not present in the original (trace,
    // virtualtouch, virtualadc, virtualorientation). Enabled for the emulator
    // test fixtures and verification tooling; set false for a source-accurate
    // public release so the command set matches the original interpreter.
    public static final boolean DEVELOPER_FIXTURES = false;
    private static final boolean MOTION_TRACE_ENABLED =
            DEVELOPER_FIXTURES || BuildConfig.MOTION_TRACE_ENABLED;

    private static final double ORIGINAL_STAND_BODY_Z = 40.0d;
    private static final double ORIGINAL_POSE_STEP_MS = 10.0d;
    private static final double ORIGINAL_STOP_VECTOR_THRESHOLD = 0.2d;
    private final double ORIGINAL_LEG_SITTING_Z;  // LEG_SITTING_Z from config (default -40)
    private final double ORIGINAL_QUAD_DISABLED_Z;  // z0.o:322 = femurScale*120 + LEG_CONNECTION_Z
    private static final double ORIGINAL_CALIBRATION_LOWER_STEP = -0.20000000298023224d;
    private static final int ORIGINAL_CALIBRATION_MAX_POLLS = 3000;
    private static final double[] ORIGINAL_QUAD_SET25_STEP_MS = {215.2d, 423.2d, 419.2d, 414.4d, 412.8d};
    private static final double[] ORIGINAL_QUAD_SET25_BLEND = {
            0.0975d,
            0.142625d,
            0.18549375d,
            0.2262190625d,
            0.264908109375d
    };
    private static final int[] ALL_LEGS = {0, 3, 1, 4, 2, 5};
    private static final int[] ORIGINAL_ACTIVE_ORDER = {5, 2, 1, 0, 3, 4};
    private static final int[] STAND_LEFT_TRIPOD = {0, 4, 2};
    private static final int[] STAND_RIGHT_TRIPOD = {3, 1, 5};
    private static final int[][] SERVO_PINS_BY_LEG = {
            {15, 16, 17},
            {9, 10, 11},
            {3, 4, 5},
            {12, 13, 14},
            {6, 7, 8},
            {0, 1, 2}
    };
    private final ModeParams[] ORIGINAL_MODES;  // MODE_* parameter sets from config
    private final ChicaServoCalibration servoCalibration;

    private final ChicaGaitEngine gaitEngine = new ChicaGaitEngine();
    private final ServoBackend servoBackend;
    private final ExecutorService originalCommandExecutor = Executors.newSingleThreadExecutor();
    private volatile long lastStepMillis = System.currentTimeMillis();
    private volatile long frameCount = 0;
    private volatile long lastCommandMillis = System.currentTimeMillis();
    private volatile int gait = 1;
    private volatile int animation = 0;
    private volatile boolean relayStatus = false;
    private volatile boolean standing = false;
    private volatile boolean autoSit = true;
    private volatile boolean levelMode = false;
    private volatile boolean calibPosition = false;
    private volatile boolean crabMode = false;
    private volatile int modeIndex = 0;
    private volatile int walkModeIndex = -1;
    private volatile boolean cameraEnabled = false;
    private volatile String pendingCommand = "";
    // Original z0.f.f7069e: pending beep count consumed by the beeper worker.
    private volatile int beepCount = 0;
    private android.media.ToneGenerator toneGenerator;
    // Latch so the warning-cutoff "torque" command is submitted once per
    // over-limit episode (original gates via f7078o command-in-flight).
    private volatile boolean cutoffTorquePending = false;
    private volatile String hardwareStatus = "virtual";
    private volatile String ipAddress = "0.0.0.0";
    private volatile boolean busy = false;
    private volatile boolean blockMode = false;
    private volatile boolean keepMode = false;
    private volatile boolean activeWalk = false;
    private volatile boolean pendingWalkClear = false;
    private volatile boolean pendingStopStep = false;
    private volatile boolean walkWorkerRunning = false;
    private volatile boolean setWorkerRunning = false;
    private volatile boolean levelWorkerRunning = false;
    private volatile long setPoseDecayUntilMillis = 0L;
    // Continuous angular-sweep set-poses (the original's worker case 2, p3.a.G):
    // 0 = static set-pose integrator, 1 = dive (z5=true), 2 = setrotate/flex (z5=false).
    private volatile int setSweepMode = 0;
    private volatile int walkStepCount = 0;
    private volatile int[] quadrupedActiveLegs = {0, 3, 2, 5};
    private volatile int[] activeOutputLegs = ALL_LEGS;
    private volatile int[] lastPulses = neutralPulses();
    private volatile double[] lastLegTouches = SurfaceStatus.UNKNOWN_LEG_TOUCHES;
    private volatile double lastVoltage = Double.NaN;
    private volatile double lastCurrent = Double.NaN;
    private volatile double voltageWarningDuration = 2.0d;
    private volatile double voltageWarningLevel = 6.4d;
    private volatile double voltageCutoffLevel = 6.0d;
    private volatile int voltageBeepCount = 3;
    private volatile double currentWarningDuration = 2.0d;
    private volatile double currentWarningLevel = 8.0d;
    private volatile double currentCutoffLevel = 10.0d;
    private volatile int currentBeepCount = 3;
    private volatile long voltageWarningSinceMillis = System.currentTimeMillis();
    private volatile long voltageCutoffSinceMillis = System.currentTimeMillis();
    private volatile long currentWarningSinceMillis = System.currentTimeMillis();
    private volatile long currentCutoffSinceMillis = System.currentTimeMillis();
    private volatile boolean voltageWarning = false;
    private volatile boolean currentWarning = false;
    private volatile double lastPrimaryX = 0.0d;
    private volatile double lastPrimaryY = 0.0d;
    private volatile double lastSecondaryX = 0.0d;
    private volatile double lastSecondaryY = 0.0d;
    // Tertiary holds the setzu (z,u) axes. The original's set-pose is a full
    // 6-DOF p3.a(x,y,z,u,v,w): primary=(x,y), tertiary=(z,u), secondary=(v,w).
    private volatile double lastTertiaryX = 0.0d;
    private volatile double lastTertiaryY = 0.0d;
    private volatile double orientationX = 0.0d;
    private volatile double orientationY = 0.0d;
    private volatile double orientationZ = 0.0d;
    private volatile WalkVector lastWalk = new WalkVector(0.0d, 0.0d, 0.0d);
    private volatile WalkVector filteredWalk = new WalkVector(0.0d, 0.0d, 0.0d);
    // Status BPS counts hardware-heartbeat iterations (the original z0.i.b). The
    // joystick panel's panelBps is the separate orientation-sensor fusion rate.
    private volatile double bps = 0.0d;
    private volatile long panelBpsWindowStartMillis = System.currentTimeMillis();
    private volatile long panelBpsWindowFrames = 0;
    private volatile double panelBps = 0.0d;

    public ChicaController() {
        this(new VirtualServoBackend());
    }

    public ChicaController(ServoBackend servoBackend) {
        this(servoBackend, null);
    }

    public ChicaController(ServoBackend servoBackend, String config) {
        this.servoBackend = servoBackend;
        this.hardwareStatus = servoBackend.name();
        ChicaRobotConfig robot = ChicaRobotConfig.parse(config);
        this.servoCalibration = ChicaServoCalibration.parse(config);
        this.ORIGINAL_LEG_SITTING_Z = robot.legSittingZ;
        double femurScale = ((robot.femurLen + 80.0d) / 2.0d) / 80.0d;
        this.ORIGINAL_QUAD_DISABLED_Z = (femurScale * 120.0d) + robot.legConnectionZ;
        this.ORIGINAL_MODES = buildModes(robot);
        servoBackend.setRelay(relayStatus);
        applyServoCalibration(servoCalibration);
        gaitEngine.configureGeometry(robot.coxaLen, robot.femurLen, robot.tibiaLen,
                robot.l1ToR1, robot.l1ToL3, robot.l2ToR2,
                robot.legConnectionZ, robot.legSittingZ);
        enterOriginalStartupPose();
        startOriginalHardwareMonitor();
        startOriginalBeeperWorker();
        // z0.f.b(): board install beeps once on a successful open, six times on
        // failure.
        beepCount = servoBackend.isConnected() ? 1 : 6;
    }

    // Original z0.c TimerTask (100 ms): play beepCount tones at 200 ms spacing,
    // zero the counter, then hold 1 s so repeated warning re-arms read as a
    // beep PATTERN (N beeps / second) rather than a continuous tone.
    private void startOriginalBeeperWorker() {
        Thread thread = new Thread(() -> {
            while (true) {
                try {
                    int count = beepCount;
                    while (count > 0) {
                        playOriginalBeepTone();
                        Thread.sleep(200L);
                        count--;
                        if (count == 0) {
                            beepCount = 0;
                            Thread.sleep(1000L);
                        }
                    }
                    Thread.sleep(100L);
                } catch (InterruptedException interrupted) {
                    Thread.currentThread().interrupt();
                    return;
                } catch (Exception ignored) {
                }
            }
        }, "chica-original-beeper");
        thread.setDaemon(true);
        thread.start();
    }

    // Original e4.f.a(): ToneGenerator(STREAM_MUSIC, 100).startTone(44, 200) on
    // the phone. Falls back to the backend hook (test fixtures) if audio is
    // unavailable (e.g. host-side unit tests).
    private void playOriginalBeepTone() {
        try {
            if (toneGenerator == null) {
                toneGenerator = new android.media.ToneGenerator(3, 100);
            }
            toneGenerator.startTone(44, 200);
        } catch (Throwable ignored) {
        }
        servoBackend.beep();
    }

    // Original z0.e heartbeat: wake at 1 ms resolution, but perform hardware work
    // only after more than 7 ms. Every heartbeat flushes the latest staged servo
    // packet; every second heartbeat also polls analog telemetry. Gait/pose workers
    // stage packets instead of performing a second, redundant synchronous write.
    private void startOriginalHardwareMonitor() {
        Thread thread = new Thread(() -> {
            long bpsWindowStart = System.currentTimeMillis();
            long lastHeartbeat = System.currentTimeMillis();
            int heartbeatCount = 0;
            int communicatedHeartbeats = 0;
            while (true) {
                try {
                    Thread.sleep(1L);
                    long now = System.currentTimeMillis();
                    if (now - lastHeartbeat > 7L) {
                        if (servoBackend.hasFault()) {
                            System.out.println("Attempting to restart the port due to error...");
                            servoBackend.restartPort();
                            Thread.sleep(250L);
                        } else {
                            // z0.e synchronizes on z0.i across both n() and i().
                            synchronized (servoBackend) {
                                servoBackend.flushServoPulses();
                                if ((heartbeatCount % 2) == 0) {
                                    servoBackend.refreshTelemetry();
                                }
                                if (servoBackend.isConnected() && !servoBackend.hasFault()) {
                                    communicatedHeartbeats++;
                                }
                            }
                            heartbeatCount++;
                            lastHeartbeat = now;
                        }
                    }
                    if (now - bpsWindowStart > 1000L) {
                        bps = communicatedHeartbeats;
                        heartbeatCount = 0;
                        communicatedHeartbeats = 0;
                        bpsWindowStart = now;
                    }
                } catch (InterruptedException interrupted) {
                    return;
                } catch (Exception error) {
                    System.out.println(error);
                }
            }
        }, "chica-hardware-monitor");
        thread.setDaemon(true);
        thread.start();
    }

    private static ModeParams[] buildModes(ChicaRobotConfig robot) {
        ModeParams[] out = new ModeParams[robot.modes.length];
        for (int i = 0; i < out.length; i++) {
            double[] m = robot.modes[i];
            out[i] = new ModeParams(m[0], m[1], m[2], m[3], m[4], m[5], m[6]);
        }
        return out;
    }

    // Push the per-servo calibration / attach angles / pin map from chica.config
    // into the gait engine before any pose is produced, so output pulses honor
    // the configured servo ranges (e.g. 270-degree servos) like the original.
    private void applyServoCalibration(ChicaServoCalibration cal) {
        gaitEngine.setServoConfig(cal.flatCalibration(), cal.coxaAttach,
                cal.femurAttach, cal.tibiaAttach, cal.flatPins());
    }

    public synchronized boolean isBusy() {
        return busy;
    }

    public synchronized String originalStatusString() {
        StringBuilder text = new StringBuilder(surfaceStatus(ipAddress).toOriginalPayload());
        text.append("|FLAGS=").append(relayStatus ? "1" : "0");
        text.append(standing ? "1" : "0");
        text.append(keepMode ? "1" : "0");
        text.append(crabMode ? "1" : "0");
        text.append(modeIndex);
        text.append(levelMode ? "1" : "0");
        text.append(autoSit ? "1" : "0");
        text.append(blockMode ? "1" : "0");
        text.append(calibPosition ? "1" : "0");
        return text.toString();
    }

    public synchronized boolean shouldHandleOriginalAck() {
        return relayStatus && !blockMode && !calibPosition && !busy;
    }

    public synchronized void logControl(String event, String command) {
        if (!MOTION_TRACE_ENABLED) return;
        Log.i("CHICA_CONTROL", controlTraceJson(event, command, Double.NaN));
    }

    public synchronized void logControlValue(String event, double value) {
        if (!MOTION_TRACE_ENABLED) return;
        Log.i("CHICA_CONTROL", controlTraceJson(event, null, value));
    }

    public void submitOriginalCommand(String command) {
        if (command == null || command.isEmpty() || "ack".equals(command) || "bye".equals(command)) return;
        synchronized (this) {
            pendingCommand = command;
            lastCommandMillis = System.currentTimeMillis();
            busy = true;
        }
        originalCommandExecutor.execute(() -> {
            try {
                synchronized (ChicaController.this) {
                    logControl("cmd_begin", command);
                }
                applyCommand(command);
                synchronized (ChicaController.this) {
                    logControl("cmd_end", null);
                }
            } finally {
                synchronized (ChicaController.this) {
                    busy = false;
                }
            }
        });
    }

    public void handleOriginalAck(int ackCount) {
        synchronized (this) {
            busy = true;
        }
        originalCommandExecutor.execute(() -> {
            try {
                if (ackCount <= 30) {
                    double rampValue = 50.0d - ackCount;
                    synchronized (ChicaController.this) {
                        logControlValue("ack_ramp", rampValue);
                    }
                    handleAckRampLocked(ackCount);
                    synchronized (ChicaController.this) {
                        pendingCommand = "ack";
                        lastCommandMillis = System.currentTimeMillis();
                    }
                } else if (ackCount == 60 && autoSit) {
                    if (standing && !activeWalk) {
                        synchronized (ChicaController.this) {
                            logControl("ack_autosit_begin", null);
                        }
                        enterOriginalSitPose();
                        synchronized (ChicaController.this) {
                            standing = false;
                            // The idle auto-sit also releases torque (servos
                            // are powered down once seated), so the status
                            // FLAGS return to the fully-idle 000000100.
                            relayStatus = false;
                            servoBackend.setRelay(false);
                            logControl("ack_autosit_end", null);
                        }
                    }
                    synchronized (ChicaController.this) {
                        pendingCommand = "ack";
                        lastCommandMillis = System.currentTimeMillis();
                    }
                }
            } finally {
                synchronized (ChicaController.this) {
                    busy = false;
                }
            }
        });
    }

    public synchronized void requestBlock() {
        submitOriginalCommand("block");
    }

    public synchronized void requestTorque() {
        submitOriginalCommand("torque");
    }

    public synchronized void requestOriginalClientStop() {
        requestOriginalWalkStop(false);
    }

    public synchronized void setCameraEnabled(boolean enabled) {
        cameraEnabled = enabled;
    }

    public synchronized void setOrientationVector(double x, double y, double z) {
        orientationX = x;
        orientationY = y;
        orientationZ = z;
        // Drives the on-screen joystick-panel number only (the original's f.t/k.g):
        // one tick per paired gravity+magnetic fusion update (e4.d). This is NOT the
        // status BPS, which the hardware-monitor worker derives from board polls.
        panelBpsWindowFrames++;
        updatePanelBps();
    }

    public synchronized boolean isCameraEnabled() {
        return cameraEnabled;
    }

    public synchronized boolean isRelayEnabled() {
        return relayStatus;
    }

    public synchronized boolean isBlockMode() {
        return blockMode;
    }

    public synchronized String configSummary(String config) {
        parseOriginalWarningConfig(config);
        int servoCount = 0;
        int modeCount = 0;
        for (String line : config.split("\\r?\\n")) {
            String trimmed = line.trim();
            if (trimmed.isEmpty() || trimmed.startsWith("#")) continue;
            if (trimmed.matches("[LR][123][123]\\s+.*")) servoCount++;
            if (trimmed.startsWith("MODE_")) modeCount++;
        }
        return "config: " + servoCount + " servos, " + modeCount + " modes";
    }

    public synchronized SurfaceStatus surfaceStatus(String ipAddress) {
        this.ipAddress = ipAddress == null ? "0.0.0.0" : ipAddress;
        // Telemetry is polled by the hardware-monitor worker; here we just read the
        // cached values and apply the EWMA blend (as the original's draw loop does).
        lastVoltage = originalTelemetryBlend(lastVoltage, servoBackend.voltage());
        lastCurrent = originalTelemetryBlend(lastCurrent, servoBackend.current());
        lastLegTouches = servoBackend.legTouches();
        updateOriginalWarnings();
        return new SurfaceStatus(lastVoltage, lastCurrent, bps, this.ipAddress,
                lastLegTouches,
                lastPrimaryX, lastPrimaryY, lastSecondaryX, lastSecondaryY,
                lastWalk.forward, lastWalk.strafe, lastWalk.turn,
                voltageWarning, currentWarning,
                voltageColorZone(), currentColorZone(), panelBps);
    }

    public synchronized String lastMathTrace() {
        return gaitEngine.lastTraceJson();
    }

    private String controlTraceJson(String event, String command, double value) {
        StringBuilder text = new StringBuilder();
        text.append("{\"event\":\"").append(jsonEscape(event)).append("\"");
        if (command != null) {
            text.append(",\"command\":\"").append(jsonEscape(command)).append("\"");
        }
        if (!Double.isNaN(value)) {
            text.append(",\"value\":").append(formatDouble(value));
        }
        text.append(",\"state\":{");
        text.append("\"busy\":").append(busy ? "true" : "false");
        text.append(",\"pending\":\"").append(jsonEscape(pendingCommand == null ? "null" : pendingCommand)).append("\"");
        text.append(",\"target\":\"").append(jsonEscape(lastWalk.active()
                ? String.format("[%1.3e, %1.3e, %1.3e]", lastWalk.forward, lastWalk.strafe, lastWalk.turn)
                : "null")).append("\"");
        text.append(",\"status\":\"").append(jsonEscape(originalStatusString())).append("\"");
        text.append(",\"torque\":").append(relayStatus ? "true" : "false");
        text.append(",\"stand\":").append(standing ? "true" : "false");
        text.append(",\"keep\":").append(keepMode ? "true" : "false");
        text.append(",\"walk\":").append(activeWalk ? "true" : "false");
        text.append(",\"set\":").append((lastPrimaryX != 0.0d || lastPrimaryY != 0.0d || lastSecondaryX != 0.0d || lastSecondaryY != 0.0d || lastTertiaryX != 0.0d || lastTertiaryY != 0.0d) ? "true" : "false");
        text.append(",\"crab\":").append(crabMode ? "true" : "false");
        text.append(",\"mode\":").append(modeIndex);
        text.append(",\"gaitMode\":").append(walkModeIndex < 0 ? 5 : walkModeIndex);
        text.append(",\"style\":").append(animation);
        text.append("}}");
        return text.toString();
    }

    public synchronized String statusText(String configSummary) {
        StringBuilder text = new StringBuilder();
        text.append("CHICA SERVER\n");
        text.append("backend: ").append(hardwareStatus).append('\n');
        text.append("tcp: 18711\n");
        text.append("relay: ").append(relayStatus ? "on" : "off");
        text.append("   position: ").append(standing ? "stand" : "sit").append('\n');
        text.append("gait: ").append(gait);
        text.append("   mode: ").append(modeIndex);
        text.append("   animation: ").append(animation);
        text.append("   frames: ").append(frameCount).append('\n');
        text.append("pending: ").append(pendingCommand.isEmpty() ? "none" : pendingCommand).append('\n');
        text.append("camera: ").append(cameraEnabled ? "enabled" : "disabled").append('\n');
        text.append(configSummary).append("\n\n");
        text.append("pulses:\n");
        for (int i = 0; i < lastPulses.length; i++) {
            if (i > 0 && i % 3 == 0) text.append('\n');
            text.append(String.format("%2d:%4d  ", i, lastPulses[i]));
        }
        return text.toString();
    }

    private void applyCommand(String command) {
        if (command == null || command.isEmpty()) return;
        if ("ack".equals(command) || "bye".equals(command)) return;
        pendingCommand = command;
        lastCommandMillis = System.currentTimeMillis();
        if (command.startsWith("torque")) {
            setOriginalRelay(!relayStatus);
        } else if (DEVELOPER_FIXTURES && command.startsWith("virtualtouch")) {
            servoBackend.setVirtualTouchFixture(!command.contains("off"));
        } else if (DEVELOPER_FIXTURES && command.startsWith("virtualadc:")) {
            double[] parts = parsePair(command);
            if (parts != null) {
                servoBackend.setVirtualTelemetry(parts[0], parts[1]);
            }
        } else if (DEVELOPER_FIXTURES && command.startsWith("virtualorientation:")) {
            double[] parts = parseTripleDouble(command);
            if (parts != null) {
                setOrientationVector(parts[0], parts[1], parts[2]);
            }
        } else if (command.startsWith("sit")) {
            setOriginalStanding(!standing, false);
            activeWalk = standing && lastWalk.active();
        } else if (command.startsWith("calibpos")) {
            setOriginalCalibPosition(!calibPosition);
        } else if (command.startsWith("block")) {
            setOriginalBlockMode(!blockMode, false);
        } else if (command.startsWith("autosit")) {
            autoSit = !autoSit;
        } else if (command.startsWith("level")) {
            levelMode = !levelMode;
            if (levelMode) {
                startOriginalLevelWorkerLocked();
            }
        } else if (command.startsWith("crab")) {
            crabMode = !crabMode;
        } else if (command.startsWith("keep")) {
            // d.n0 keep branch: toggle the keep flag; when turned off, stand up
            // if not busy (z0.o.i toggle, then z0.o.d(true, false)).
            keepMode = !keepMode;
            if (!keepMode && !originalMotionBusy()) {
                setOriginalStanding(true, false);
            }
        } else if (command.startsWith("home")) {
            enterOriginalHomePose();
        } else if (command.startsWith("bounce")) {
            runOriginalBounce();
        } else if (command.startsWith("jump")) {
            runOriginalJump();
        } else if (command.startsWith("calibrate")) {
            runOriginalCalibrate();
        } else if (command.startsWith("reboot")) {
            runOriginalSystemCommand("reboot");
        } else if (command.startsWith("restart")) {
            runOriginalSystemCommand("/home/pi/code/chica.sh restart");
        } else if (command.startsWith("standard")) {
            applyOriginalMode(0);
        } else if (command.startsWith("race")) {
            applyOriginalMode(1);
        } else if (command.startsWith("offroad")) {
            applyOriginalMode(2);
        } else if (command.startsWith("custom")) {
            applyOriginalMode(3);
        } else if (command.startsWith("quad")) {
            applyOriginalMode(4, parseQuadDisabledLegs(command));
        } else if (command.startsWith("walkclear")) {
            requestOriginalWalkStop(true);
        } else if (command.startsWith("walk")) {
            boolean startingWalkWorker = !walkWorkerRunning;
            int previousWalkMode = walkModeIndex;
            int previousAnimation = animation;
            applyWalkCommand(command);
            if (!startingWalkWorker) {
                // d.n0 updates only the shared target while z0.e is running.
                walkModeIndex = previousWalkMode;
                animation = previousAnimation;
            }
            relayStatus = true;
            servoBackend.setRelay(true);
            if (!standing) {
                enterOriginalStandPose();
                standing = true;
            }
            activeWalk = true;
            pendingWalkClear = false;
            pendingStopStep = false;
            blockMode = false;
            if (startingWalkWorker) {
                gaitEngine.beginWalkSession();
                filteredWalk = lerpWalk(new WalkVector(0.0d, 0.0d, 0.0d), lastWalk, 0.05d);
                lastStepMillis = System.currentTimeMillis();
                walkStepCount = 0;
                startOriginalWalkWorkerLocked();
            }
        } else if (command.startsWith("clear")) {
            activeWalk = false;
            pendingWalkClear = false;
            pendingStopStep = false;
            lastWalk = new WalkVector(0.0d, 0.0d, 0.0d);
            filteredWalk = new WalkVector(0.0d, 0.0d, 0.0d);
            walkStepCount = 0;
            walkModeIndex = -1;
            resetSetControls();
            if (relayStatus && !originalMotionBusy()) {
                setOriginalStanding(false, true);
            }
        } else if (command.startsWith("setclear")) {
            resetSetControls();
        } else if (command.startsWith("set")) {
            applySetCommand(command);
        } else if (command.startsWith("beep")) {
            playOriginalBeepTone();  // original dispatch: i.a() direct tone
        }
    }

    private void runOriginalSystemCommand(String command) {
        setOriginalRelay(false);
        try {
            Runtime.getRuntime().exec(command);
        } catch (Exception error) {
            System.out.println(error);
        }
    }

    private void applyWalkCommand(String command) {
        if (command.startsWith("walkclear")) {
            requestOriginalWalkStop(true);
            return;
        }
        double[] parts = parseTriple(command);
        if (parts == null) return;
        double forward = parts[1];
        double strafe = crabMode ? parts[0] : 0.0d;
        double turn = crabMode ? 0.0d : parts[0];
        lastWalk = new WalkVector(clampUnit(forward), clampUnit(strafe), clampUnit(turn));
        animation = (int) parts[2];
        if (command.startsWith("walk3:")) {
            walkModeIndex = 5;
        } else if (command.startsWith("walk2:")) {
            walkModeIndex = 6;
        } else if (command.startsWith("walk1:")) {
            walkModeIndex = 7;
        } else if (command.startsWith("walk15:")) {
            walkModeIndex = 8;
        } else if (command.startsWith("walk25:")) {
            walkModeIndex = 9;
        } else if (command.startsWith("walkwave:")) {
            walkModeIndex = 10;
        }
    }

    private void applySetCommand(String command) {
        if (command.startsWith("setclear")) {
            resetSetControls();
            return;
        }
        setPoseDecayUntilMillis = 0L;
        double[] parts = parsePair(command);
        if (parts == null) return;
        // The original builds a p3.a(DDDDDD) holding two z0.m vectors (e, f) and
        // clamps EACH vector to unit *magnitude* (z0.m.e()/h()), not each axis
        // independently. For the two-axis commands both axes occupy one vector,
        // so the pair is scaled down together when its length exceeds 1. setzu is
        // the exception: its axes live in separate single-component vectors
        // (e=(0,0,zu) and f=(zu2,0,0)), so there each axis clamps on its own.
        if (command.startsWith("setxy:")) {
            double[] e = clampStickPair(-parts[0], parts[1]);
            lastPrimaryX = e[0];
            lastPrimaryY = e[1];
        } else if (command.startsWith("setzu:")) {
            // z = second value, u = first value (original: z=v8, u=v3). z/u live
            // in separate xyz/uvw vectors, so clamp each axis independently and
            // store in the tertiary slot (NOT secondary, which is v,w).
            lastTertiaryX = clampUnit(parts[1]);
            lastTertiaryY = clampUnit(parts[0]);
        } else if (command.startsWith("setvw:")) {
            double[] f = clampStickPair(-parts[0], -parts[1]);
            lastSecondaryX = f[0];
            lastSecondaryY = f[1];
        } else if (command.startsWith("setxyvw:") || command.startsWith("setdive:")) {
            double[] e = clampStickPair(-parts[0], parts[1]);
            lastPrimaryX = e[0];
            lastPrimaryY = e[1];
            double[] f = clampStickPair(parts[0], parts[1]);
            lastSecondaryX = f[0];
            lastSecondaryY = f[1];
        } else if (command.startsWith("setrotate:")) {
            double[] e = clampStickPair(-parts[0], parts[1]);
            lastPrimaryX = e[0];
            lastPrimaryY = e[1];
        }
        // dive and setrotate route to the continuous angular sweep (G), not the
        // static set-pose integrator. They differ only by pose form (z5).
        if (command.startsWith("setdive:")) {
            setSweepMode = 1;
        } else if (command.startsWith("setrotate:")) {
            setSweepMode = 2;
        } else {
            setSweepMode = 0;
        }
        if (isQuadOutputMode()) {
            publishOriginalFrame();
            if (isQuad25SetMode()) {
                startOriginalQuadSetWorkerLocked();
            }
        } else {
            startOriginalSetWorkerLocked();
        }
    }

    private void resetSetControls() {
        if (lastPrimaryX != 0.0d || lastPrimaryY != 0.0d
                || lastSecondaryX != 0.0d || lastSecondaryY != 0.0d || lastTertiaryX != 0.0d || lastTertiaryY != 0.0d || setWorkerRunning) {
            setPoseDecayUntilMillis = System.currentTimeMillis() + 1500L;
        }
        lastPrimaryX = 0.0d;
        lastPrimaryY = 0.0d;
        lastSecondaryX = 0.0d;
        lastSecondaryY = 0.0d;
        lastTertiaryX = 0.0d;
        lastTertiaryY = 0.0d;
        // Let the post-release decay ramp the orbit pose back down via the static
        // integrator, then clearSetPose resets the sweep angle.
        setSweepMode = 0;
    }

    private void setOriginalRelay(boolean enabled) {
        if (enabled == relayStatus) return;
        if (enabled) {
            relayStatus = true;
            servoBackend.setRelay(true);
            return;
        }
        if (originalMotionBusy()) return;
        if (standing) {
            setOriginalStanding(false, false);
        }
        relayStatus = false;
        servoBackend.setRelay(false);
    }

    private boolean setOriginalStanding(boolean enabled, boolean powerOffAfterSit) {
        if (blockMode) {
            setOriginalBlockMode(false, false);
        }
        if (calibPosition) {
            setOriginalCalibPosition(false);
        }
        if (enabled == standing) return true;
        if (enabled) {
            setOriginalRelay(true);
            enterOriginalStandPose();
            standing = true;
        } else {
            if (originalMotionBusy()) return false;
            enterOriginalSitPose();
            standing = false;
            if (powerOffAfterSit) {
                relayStatus = false;
                servoBackend.setRelay(false);
            }
        }
        return true;
    }

    private void setOriginalBlockMode(boolean enabled, boolean force) {
        if (enabled == blockMode && !force) return;
        boolean wasRelayOn = relayStatus;
        if (enabled) {
            if (originalMotionBusy() || !setOriginalStanding(false, false)) return;
            setOriginalRelay(true);
            publishOriginalBlockShapeRamp();
            blockMode = true;
            return;
        }
        setOriginalRelay(true);
        publishOriginalBlockRaisedRamp();
        publishOriginalSittingShapeRampForMode(currentMode());
        if (!wasRelayOn) {
            setOriginalRelay(false);
        }
        blockMode = false;
    }

    private void setOriginalCalibPosition(boolean enabled) {
        if (enabled == calibPosition) return;
        boolean wasRelayOn = relayStatus;
        if (enabled) {
            if (originalMotionBusy() || !setOriginalStanding(false, false)) return;
            setOriginalRelay(true);
            publishOriginalBlockRaisedRamp();
            publishOriginalCalibPoseRamp();
            calibPosition = true;
            return;
        }
        setOriginalRelay(true);
        publishOriginalBlockRaisedRamp();
        publishOriginalSittingShapeRampForMode(currentMode());
        if (!wasRelayOn) {
            setOriginalRelay(false);
        }
        calibPosition = false;
    }

    private boolean originalMotionBusy() {
        return activeWalk || walkWorkerRunning || setWorkerRunning || pendingStopStep || pendingWalkClear;
    }

    private boolean isQuadOutputMode() {
        return activeOutputLegs != ALL_LEGS && activeOutputLegs.length < 6;
    }

    private boolean isQuad25SetMode() {
        if (quadrupedActiveLegs.length != 4) return false;
        return quadrupedActiveLegs[0] == 1
                && quadrupedActiveLegs[1] == 0
                && quadrupedActiveLegs[2] == 3
                && quadrupedActiveLegs[3] == 4;
    }

    private boolean stepOriginalSetPose() {
        if (!relayStatus || !standing) return false;
        long now = System.currentTimeMillis();
        boolean hasTarget = lastPrimaryX != 0.0d || lastPrimaryY != 0.0d
                || lastSecondaryX != 0.0d || lastSecondaryY != 0.0d || lastTertiaryX != 0.0d || lastTertiaryY != 0.0d;
        if (!hasTarget && now > setPoseDecayUntilMillis) {
            setPoseDecayUntilMillis = 0L;
            lastPulses = mergeActiveLegPulses(gaitEngine.clearSetPose());
            publishOriginalFrame();
            return false;
        }
        double dtMs = Math.max(0.0d, now - lastStepMillis) * currentMode().speed;
        lastStepMillis = now;
        if (setSweepMode != 0) {
            // Continuous orbit: the held stick (lastPrimaryX/Y = the original's
            // aVar.R()/S()) sets the sweep speed; the body keeps animating.
            lastPulses = mergeActiveLegPulses(gaitEngine.stepSetSweep(
                    lastPrimaryX,
                    lastPrimaryY,
                    setSweepMode == 1,
                    dtMs));
        } else {
            lastPulses = mergeActiveLegPulses(gaitEngine.stepSetPose(
                    lastPrimaryX,
                    lastPrimaryY,
                    lastTertiaryX,
                    lastTertiaryY,
                    lastSecondaryX,
                    lastSecondaryY,
                    dtMs));
        }
        publishOriginalFrame();
        return true;
    }

    private void startOriginalSetWorkerLocked() {
        if (setWorkerRunning) return;
        setWorkerRunning = true;
        lastStepMillis = System.currentTimeMillis();
        Thread thread = new Thread(() -> {
            while (true) {
                synchronized (ChicaController.this) {
                    if (!stepOriginalSetPose()) {
                        setWorkerRunning = false;
                        return;
                    }
                }
                try {
                    Thread.sleep(10L);
                } catch (InterruptedException ignored) {
                    Thread.currentThread().interrupt();
                    synchronized (ChicaController.this) {
                        setWorkerRunning = false;
                    }
                    return;
                }
            }
        }, "chica-original-set");
        thread.setDaemon(true);
        thread.start();
    }

    private void startOriginalQuadSetWorkerLocked() {
        if (setWorkerRunning) return;
        setWorkerRunning = true;
        Thread thread = new Thread(() -> {
            try {
                for (int i = 0; i < ORIGINAL_QUAD_SET25_STEP_MS.length; i++) {
                    Thread.sleep(510L);
                    synchronized (ChicaController.this) {
                        if (!relayStatus || !standing || !isQuad25SetMode()) {
                            setWorkerRunning = false;
                            return;
                        }
                        double blend = ORIGINAL_QUAD_SET25_BLEND[i];
                        lastPulses = mergeActiveLegPulses(gaitEngine.stepSetPose(
                                lastPrimaryX * blend,
                                lastPrimaryY * blend,
                                lastTertiaryX * blend,
                                lastTertiaryY * blend,
                                lastSecondaryX * blend,
                                lastSecondaryY * blend,
                                ORIGINAL_QUAD_SET25_STEP_MS[i]));
                        publishOriginalFrame();
                    }
                }
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
            synchronized (ChicaController.this) {
                setWorkerRunning = false;
            }
        }, "chica-original-quad-set");
        thread.setDaemon(true);
        thread.start();
    }

    private boolean stepOriginalLevelPose() {
        if (levelMode) {
            lastPulses = mergeActiveLegPulses(gaitEngine.applyLevelPose(orientationX, orientationY));
            publishOriginalFrame();
            return true;
        }
        if (gaitEngine.levelPoseMagnitude() > 0.1d) {
            lastPulses = mergeActiveLegPulses(gaitEngine.decayLevelPose(0.98d));
            publishOriginalFrame();
            return true;
        }
        lastPulses = mergeActiveLegPulses(gaitEngine.decayLevelPose(0.0d));
        publishOriginalFrame();
        return false;
    }

    private void startOriginalLevelWorkerLocked() {
        if (levelWorkerRunning) return;
        levelWorkerRunning = true;
        Thread thread = new Thread(() -> {
            try {
                while (true) {
                    synchronized (ChicaController.this) {
                        if (!stepOriginalLevelPose()) {
                            levelWorkerRunning = false;
                            levelMode = false;
                            return;
                        }
                    }
                    sleepOriginalPoseStep(ORIGINAL_POSE_STEP_MS);
                }
            } catch (Exception ignored) {
                synchronized (ChicaController.this) {
                    levelWorkerRunning = false;
                    levelMode = false;
                }
            }
        }, "chica-original-level");
        thread.setDaemon(true);
        thread.start();
    }

    private void enterOriginalStandPose() {
        ModeParams mode = currentMode();
        publishOriginalHomePose(10.0d);
        publishTimedBodyZRamp(mode.bodyLift, 400.0d / mode.speed);
    }

    private void enterOriginalSitPose() {
        ModeParams mode = currentMode();
        publishOriginalHomePose(20.0d);
        publishTimedBodyZRamp(0.0d, 550.0d / mode.speed);
    }

    private void enterOriginalStartupPose() {
        // Original boot = o() ctor -> i([], 0): old mode is the sentinel 9, so it
        // runs the block-mode EXIT with force (a(false, true)) — raise to the
        // block row + (80, 100) then settle to MODE_STANDARD at sitting z. Derive
        // from the parsed mode table so custom configs boot exactly.
        lastPulses = gaitEngine.enterConstructorPose();
        publishOriginalFrame();
        ModeParams block = ORIGINAL_MODES[5];
        ModeParams standard = ORIGINAL_MODES[0];
        publishTimedShapeRamp(block.radius + 80.0d, block.bodyLift + 100.0d,
                block.cornerAngle, block.elongation, 800.0d / standard.speed);
        publishTimedShapeRamp(standard.radius, ORIGINAL_LEG_SITTING_Z,
                standard.cornerAngle, standard.elongation, 1200.0d / standard.speed);
    }

    private void handleAckRampLocked(int ackCount) {
        // z0.b -> z0.o.g(50 - ackCount).
        publishOriginalHomePose(50.0d - ackCount);
    }

    private void enterOriginalHomePose() {
        long started = System.currentTimeMillis();
        publishOriginalHomePose(-1.0d);
        sleepRemaining(started, originalHomeBusyMillis(modeIndex));
    }

    private void publishOriginalHomePose(double threshold) {
        if (activeWalk || lastPrimaryX != 0.0d || lastPrimaryY != 0.0d || lastSecondaryX != 0.0d || lastSecondaryY != 0.0d || lastTertiaryX != 0.0d || lastTertiaryY != 0.0d) {
            return;
        }
        ModeParams mode = currentMode();
        double lift = standing ? mode.stepLift : 15.0d;
        double layerBlend = standing ? mode.animationFactor : 0.0d;
        double duration = 650.0d / mode.speed;
        if (modeIndex == 4) {
            if (standing) {
                for (int leg : quadrupedActiveLegs) {
                    publishTimedPoseRampToNeutral(new int[] {leg}, threshold, lift, layerBlend, duration);
                }
            } else {
                publishTimedPoseRampToNeutral(quadrupedActiveLegs, threshold, lift, layerBlend, duration);
            }
            return;
        }
        if (standing) {
            publishTimedPoseRampToNeutral(STAND_LEFT_TRIPOD, threshold, lift, layerBlend, duration);
            publishTimedPoseRampToNeutral(STAND_RIGHT_TRIPOD, threshold, lift, layerBlend, duration);
        } else {
            publishTimedPoseRampToNeutral(ALL_LEGS, threshold, lift, layerBlend, duration);
        }
    }

    private void applyOriginalMode(int requestedMode) {
        applyOriginalMode(requestedMode, null);
    }

    private void applyOriginalMode(int requestedMode, int[] disabledQuadLegs) {
        long started = System.currentTimeMillis();
        int nextMode = Math.max(0, Math.min(4, requestedMode));
        if (nextMode == modeIndex || activeWalk || lastPrimaryX != 0.0d || lastPrimaryY != 0.0d
                || lastSecondaryX != 0.0d || lastSecondaryY != 0.0d || lastTertiaryX != 0.0d || lastTertiaryY != 0.0d) {
            return;
        }
        if (nextMode == 4) {
            applyOriginalQuadMode(started, disabledQuadLegs == null ? new int[] {1, 4} : disabledQuadLegs);
            return;
        }
        ModeParams mode = ORIGINAL_MODES[nextMode];
        int oldModeIndex = modeIndex;
        boolean wasStanding = standing;

        // QUAD EXIT (original z0.o.i() i8==4 path). The original SITS, then
        // shape-ramps the previously-disabled legs OUT to the new mode geometry
        // at sitting z (h.f7099h = -40) over 700/quad-speed, re-enables all legs,
        // home-poses, and re-stands. The shape-ramp is a LINEAR lerp (no swing
        // arc), so the tucked legs (z = femurScale*120 + connZ = 110) descend
        // smoothly to neutral. Without it, the home pose's lift arc sweeps those
        // legs UP past z=150 then down -> the "not smooth" quad->hex snap. Only
        // quad->* needs this; hex->hex re-seats via the plain home pose below.
        if (oldModeIndex == 4) {
            if (wasStanding) {
                enterOriginalSitPose();
                standing = false;
            }
            relayStatus = true;
            servoBackend.setRelay(true);
            // Re-enable ALL legs BEFORE the shape-ramp (original z0.o.i does
            // a(null) before C()). TWO masks must open, or the disabled legs
            // misbehave through the ramp: (1) engine IK mask via configureMode —
            // otherwise inverseKinematics skips legs {1,4} (leaving stale angles)
            // and they collapse fully tucked under the body once visible; and
            // (2) the output mask activeOutputLegs — otherwise mergeActiveLegPulses
            // freezes {1,4} at their tuck pulses through the ramp (invisible
            // "wait") and the later ALL_LEGS flip snaps them to hex. With both
            // open, the ramp's descent is IK-solved and actually sent → smooth.
            gaitEngine.configureMode(mode.radius, mode.cornerAngle, mode.elongation,
                    ORIGINAL_LEG_SITTING_Z, mode.stepLift, mode.animationFactor);
            activeOutputLegs = ALL_LEGS;
            // Re-seat every foot from the current servo angles (forward
            // kinematics) before ramping. The parked disabled legs stayed frozen
            // at their tuck angles while the body drifted forward during the
            // walk, so their stored world position is stale (~travelled distance
            // behind the body). FK snaps them back under the CURRENT body;
            // without it the shape-ramp starts from that stale spot and sweeps
            // the whole distance -> malformed away from the origin. Mirrors the
            // original z0.a.a(null) -> z0.j.a() on re-enable.
            gaitEngine.reseatFeetFromForwardKinematics();
            // The disabled legs are the two NOT in the quad active set. (Do NOT
            // reuse activeLegComplement here: that maps disabled->active, so
            // feeding it the active set returns a bogus 4-leg mix that ramps two
            // active legs and misses two, splitting the active reconfiguration
            // across both phases — the observed disabled-leg snap + off sequence.)
            boolean[] isActive = new boolean[6];
            for (int leg : quadrupedActiveLegs) {
                if (leg >= 0 && leg < 6) isActive[leg] = true;
            }
            int[] disabledLegs = new int[6 - quadrupedActiveLegs.length];
            for (int leg = 0, di = 0; leg < 6 && di < disabledLegs.length; leg++) {
                if (!isActive[leg]) disabledLegs[di++] = leg;
            }
            publishTimedShapeRampForLegs(disabledLegs, mode.radius, ORIGINAL_LEG_SITTING_Z,
                    mode.cornerAngle, mode.elongation, 700.0d / ORIGINAL_MODES[4].speed);
        }

        modeIndex = nextMode;
        activeOutputLegs = ALL_LEGS;
        if (modeIndex <= 3) gait = modeIndex + 1;
        gaitEngine.configureMode(mode.radius, mode.cornerAngle, mode.elongation, ORIGINAL_LEG_SITTING_Z,
                mode.stepLift, mode.animationFactor);
        if (standing) {
            publishTimedBodyZRamp(mode.bodyLift, 400.0d / mode.speed);
        }
        enterOriginalHomePose();
        if (oldModeIndex == 4 && wasStanding) {
            enterOriginalStandPose();
            standing = true;
        }
        sleepRemaining(started, originalModeBusyMillis(nextMode));
    }

    private void applyOriginalQuadMode(long startedMillis, int[] disabledLegs) {
        int oldModeIndex = modeIndex;
        boolean oldRelayStatus = relayStatus;
        boolean oldStanding = standing;
        ModeParams oldMode = currentMode();

        if (oldStanding) {
            enterOriginalSitPose();
            standing = false;
        }

        relayStatus = true;
        servoBackend.setRelay(true);

        int[] normalizedDisabled = normalizeDisabledQuadLegs(disabledLegs);
        // Original tucks the disabled legs at STANDARD radius+40 (h.f7104n[0]),
        // not the outgoing mode's radius; duration uses the outgoing speed.
        publishTimedShapeRampForLegs(
                normalizedDisabled,
                ORIGINAL_MODES[0].radius + 40.0d,
                ORIGINAL_QUAD_DISABLED_Z,
                80.0d,
                1.0d,
                700.0d / oldMode.speed);

        quadrupedActiveLegs = activeLegComplement(normalizedDisabled);
        activeOutputLegs = quadrupedActiveLegs;
        ModeParams quadMode = ORIGINAL_MODES[4];
        modeIndex = 4;
        gaitEngine.configureModeForLegs(
                quadMode.radius,
                quadMode.cornerAngle,
                quadMode.elongation,
                ORIGINAL_LEG_SITTING_Z,
                quadMode.stepLift,
                quadMode.animationFactor,
                quadrupedActiveLegs);
        if (oldModeIndex <= 3) gait = oldModeIndex + 1;

        enterOriginalHomePose();

        if (oldStanding) {
            enterOriginalStandPose();
            standing = true;
        } else if (!oldRelayStatus) {
            relayStatus = false;
            servoBackend.setRelay(false);
        }

        sleepRemaining(startedMillis, originalModeBusyMillis(4));
    }

    private static long originalModeBusyMillis(int mode) {
        if (mode == 2) return 3000L;
        if (mode == 4) return 3300L;
        return 1400L;
    }

    private static long originalHomeBusyMillis(int mode) {
        if (mode == 2) return 3000L;
        return 1400L;
    }

    private static void sleepRemaining(long startedMillis, long targetDurationMillis) {
        long remaining = targetDurationMillis - (System.currentTimeMillis() - startedMillis);
        if (remaining <= 0L) return;
        try {
            Thread.sleep(remaining);
        } catch (InterruptedException ignored) {
            Thread.currentThread().interrupt();
        }
    }

    private void publishTimedPoseRampToNeutral(int[] legs,
                                               double threshold,
                                               double lift,
                                               double layerBlend,
                                               double durationMs) {
        if (!gaitEngine.beginPoseRampToNeutral(legs, threshold, lift, layerBlend, durationMs)) return;
        publishTimedAnimation("M", durationMs);
    }

    private void publishTimedBodyZRamp(double bodyZ, double durationMs) {
        if (!gaitEngine.beginBodyZRamp(bodyZ, durationMs)) return;
        publishTimedAnimation("l", durationMs);
    }

    private void publishTimedBodyZDeltaRamp(double bodyZDelta, double durationMs) {
        if (!gaitEngine.beginBodyZDeltaRamp(bodyZDelta, durationMs)) return;
        publishTimedAnimation("l", durationMs);
    }

    private void publishTimedShapeRamp(double radius,
                                       double z,
                                       double cornerAngleDeg,
                                       double elongation,
                                       double durationMs) {
        if (!gaitEngine.beginShapeRamp(radius, z, cornerAngleDeg, elongation, durationMs)) return;
        publishTimedAnimation("l", durationMs);
    }

    private void publishOriginalBlockRaisedRamp() {
        ModeParams block = ORIGINAL_MODES[5];
        // Durations divide by j.f7130i = the CURRENT mode's speed (block is not
        // a mode; f7161n/f7130i stay on the active mode), not the block row's.
        publishTimedShapeRamp(block.radius + 80.0d,
                block.bodyLift + 100.0d,
                block.cornerAngle,
                block.elongation,
                800.0d / currentMode().speed);
    }

    private void publishOriginalBlockShapeRamp() {
        publishOriginalBlockRaisedRamp();
        ModeParams block = ORIGINAL_MODES[5];
        publishTimedShapeRamp(block.radius,
                block.bodyLift,
                block.cornerAngle,
                block.elongation,
                1200.0d / currentMode().speed);
    }

    private void publishOriginalSittingShapeRampForMode(ModeParams mode) {
        publishTimedShapeRamp(mode.radius,
                ORIGINAL_LEG_SITTING_Z,
                mode.cornerAngle,
                mode.elongation,
                1200.0d / mode.speed);
    }

    private void publishOriginalCalibPoseRamp() {
        int[] target = new int[18];
        for (int leg = 0; leg < servoCalibration.pin.length; leg++) {
            target[servoCalibration.pin[leg][0]] = originalAngleToPulse(0.0d, leg, 0);
            target[servoCalibration.pin[leg][1]] = originalAngleToPulse(90.0d, leg, 1);
            target[servoCalibration.pin[leg][2]] = originalAngleToPulse(90.0d, leg, 2);
        }
        publishTimedPulseRamp(target, 1200.0d / currentMode().speed);
    }

    private void publishTimedPulseRamp(int[] target, double durationMs) {
        int[] start = lastPulses.clone();
        long started = System.currentTimeMillis();
        while (true) {
            double elapsed = Math.max(0.0d, (double) (System.currentTimeMillis() - started));
            double amount = durationMs <= 0.0d ? 1.0d : Math.min(1.0d, elapsed / durationMs);
            int[] frame = new int[18];
            for (int i = 0; i < frame.length; i++) {
                frame[i] = (int) (((double) start[i] * (1.0d - amount)) + ((double) target[i] * amount));
            }
            lastPulses = mergeActiveLegPulses(frame);
            if (MOTION_TRACE_ENABLED) {
                Log.i("CHICA_ANIM", animationTraceJson("l", durationMs, elapsed));
            }
            publishOriginalFrame();
            if (elapsed >= durationMs) return;
            sleepOriginalPoseStep(ORIGINAL_POSE_STEP_MS);
        }
    }

    private int originalAngleToPulse(double angleDeg, int leg, int joint) {
        double corrected = angleDeg;
        if (joint == 0) {
            corrected = (corrected - servoCalibration.coxaAttach[Math.max(0, Math.min(5, leg))]) * -1.0d;
        } else if (joint == 1) {
            corrected -= servoCalibration.femurAttach;
        } else {
            corrected = (corrected * -1.0d) + servoCalibration.tibiaAttach;
        }
        int[] cal = servoCalibration.calibration[Math.max(0, Math.min(5, leg))][Math.max(0, Math.min(2, joint))];
        int center = (cal[1] + cal[0]) / 2;
        int scaled = (int) (corrected * (((double) (cal[1] - cal[0])) / 90.0d));
        int side = leg > 2 ? -1 : 1;
        return (scaled * side) + center;
    }

    private void publishTimedShapeRampForLegs(int[] legs,
                                              double radius,
                                              double z,
                                              double cornerAngleDeg,
                                              double elongation,
                                              double durationMs) {
        if (!gaitEngine.beginShapeRampForLegs(legs, radius, z, cornerAngleDeg, elongation, durationMs)) return;
        publishTimedAnimation("l", durationMs);
    }

    private void publishTimedAnimation(String method, double durationMs) {
        long started = System.currentTimeMillis();
        while (true) {
            double elapsed = Math.max(0.0d, (double) (System.currentTimeMillis() - started));
            lastPulses = mergeActiveLegPulses(gaitEngine.sampleTimedAnimation(elapsed));
            if (MOTION_TRACE_ENABLED) {
                Log.i("CHICA_ANIM", animationTraceJson(method, durationMs, elapsed));
            }
            publishOriginalFrame();
            if (elapsed >= durationMs) return;
            sleepOriginalPoseStep(ORIGINAL_POSE_STEP_MS);
        }
    }

    private boolean ensureOriginalStandingForImpulse() {
        if (activeWalk || lastWalk.active() || lastPrimaryX != 0.0d || lastPrimaryY != 0.0d
                || lastSecondaryX != 0.0d || lastSecondaryY != 0.0d || lastTertiaryX != 0.0d || lastTertiaryY != 0.0d) {
            return false;
        }
        blockMode = false;
        calibPosition = false;
        if (!relayStatus) {
            relayStatus = true;
            servoBackend.setRelay(true);
        }
        if (!standing) {
            enterOriginalStandPose();
            standing = true;
        }
        return true;
    }

    private void runOriginalBounce() {
        if (!ensureOriginalStandingForImpulse()) return;
        double duration = 168.0d / currentMode().speed;
        for (int i = 0; i < 2; i++) {
            publishTimedBodyZDeltaRamp(10.0d, duration);
            publishTimedBodyZDeltaRamp(-10.0d, duration / 2.0d);
        }
    }

    private void runOriginalJump() {
        if (!ensureOriginalStandingForImpulse()) return;
        double duration = 200.0d / currentMode().speed;
        publishTimedBodyZDeltaRamp(18.0d, duration);
        publishTimedBodyZDeltaRamp(126.0d, duration / 3.0d);
        publishTimedBodyZDeltaRamp(-216.0d, duration / 3.0d);
        publishTimedBodyZDeltaRamp(72.0d, duration);
    }

    private void runOriginalCalibrate() {
        if (activeWalk || walkWorkerRunning || setWorkerRunning || levelWorkerRunning) return;
        if (standing) {
            enterOriginalSitPose();
            standing = false;
            relayStatus = false;
            servoBackend.setRelay(false);
        }
        servoBackend.refreshTelemetry();
        if (!hasNumericLegTelemetry(servoBackend.legTouches())) {
            Log.w("CHICA_CALIBRATE", "calibrate aborted: leg touch telemetry unavailable");
            return;
        }

        int[] previousActiveOutputLegs = activeOutputLegs;
        activeOutputLegs = ALL_LEGS;
        try {
            for (int pass = 0; pass < 3; pass++) {
                lastPulses = mergeActiveLegPulses(gaitEngine.calibrationRaiseAll(10.0d));
                publishOriginalFrame();
                sleepOriginalPoseStep(200.0d);

                boolean[] contacted = new boolean[6];
                for (int poll = 0; poll < ORIGINAL_CALIBRATION_MAX_POLLS; poll++) {
                    lastPulses = mergeActiveLegPulses(gaitEngine.calibrationCurrentPulses());
                    publishOriginalFrame();
                    sleepOriginalPoseStep(pass == 0 ? 5.0d : 100.0d);
                    servoBackend.refreshTelemetry();
                    double[] touches = servoBackend.legTouches();
                    if (!hasNumericLegTelemetry(touches)) {
                        Log.w("CHICA_CALIBRATE", "calibrate aborted: leg touch telemetry became unavailable");
                        return;
                    }
                    boolean allContacted = true;
                    for (int leg : ALL_LEGS) {
                        if (touches[leg] > 0.5d) {
                            contacted[leg] = true;
                        }
                        if (!contacted[leg]) {
                            allContacted = false;
                        }
                    }
                    if (allContacted) break;
                    gaitEngine.calibrationLowerUntouched(contacted, ORIGINAL_CALIBRATION_LOWER_STEP);
                    if (poll == ORIGINAL_CALIBRATION_MAX_POLLS - 1) {
                        Log.w("CHICA_CALIBRATE", "calibrate aborted: contact search exceeded guard limit");
                        return;
                    }
                }
            }
        } finally {
            activeOutputLegs = previousActiveOutputLegs;
        }
    }

    private static boolean hasNumericLegTelemetry(double[] touches) {
        if (touches == null || touches.length < 6) return false;
        for (int leg : ALL_LEGS) {
            if (Double.isNaN(touches[leg]) || Double.isInfinite(touches[leg])) return false;
        }
        return true;
    }

    private static String animationTraceJson(String method, double durationMs, double elapsedMs) {
        return "{\"method\":\"" + jsonEscape(method) + "\",\"duration\":" + formatDouble(durationMs)
                + ",\"elapsed\":" + formatDouble(elapsedMs) + "}";
    }

    private static String formatDouble(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) return "null";
        return Double.toString(value);
    }

    private static void sleepOriginalPoseStep(double millis) {
        if (millis < 1.0d) return;
        try {
            long started = System.currentTimeMillis();
            if (millis > 10.0d) {
                Thread.sleep((long) (((int) millis) - 10));
            }
            while ((double) (System.currentTimeMillis() - started) < millis) {
                Thread.sleep(1L);
            }
        } catch (InterruptedException ignored) {
            Thread.currentThread().interrupt();
        }
    }

    private void publishFrameSequence(int[][] frames) {
        for (int[] frame : frames) {
            if (frame == null || frame.length != 18) continue;
            lastPulses = mergeActiveLegPulses(frame);
            publishOriginalFrame();
            try {
                Thread.sleep(10L);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    private int[] mergeActiveLegPulses(int[] frame) {
        if (frame == null || frame.length != 18) return lastPulses;
        if (activeOutputLegs == ALL_LEGS || activeOutputLegs.length >= 6) return frame.clone();
        int[] merged = lastPulses.clone();
        for (int leg : activeOutputLegs) {
            if (leg < 0 || leg >= SERVO_PINS_BY_LEG.length) continue;
            for (int pin : SERVO_PINS_BY_LEG[leg]) {
                merged[pin] = frame[pin];
            }
        }
        return merged;
    }

    private static int[] parseQuadDisabledLegs(String command) {
        int colon = command.indexOf(':');
        if (colon < 0 || colon + 1 >= command.length()) return new int[] {1, 4};
        String[] parts = command.substring(colon + 1).split(",");
        if (parts.length < 2) return new int[] {1, 4};
        try {
            return normalizeDisabledQuadLegs(new int[] {
                    Integer.parseInt(parts[0].trim()),
                    Integer.parseInt(parts[1].trim())
            });
        } catch (NumberFormatException error) {
            return new int[] {1, 4};
        }
    }

    private static int[] normalizeDisabledQuadLegs(int[] disabledLegs) {
        int first = validLeg(disabledLegs != null && disabledLegs.length > 0 ? disabledLegs[0] : 1, 1);
        int second = validLeg(disabledLegs != null && disabledLegs.length > 1 ? disabledLegs[1] : 4, 4);
        if (first == second) second = first == 4 ? 1 : 4;
        return new int[] {first, second};
    }

    private static int[] activeLegComplement(int[] disabledLegs) {
        int[] disabled = normalizeDisabledQuadLegs(disabledLegs);
        int[] active = new int[4];
        int index = 0;
        for (int leg : ORIGINAL_ACTIVE_ORDER) {
            if (leg == disabled[0] || leg == disabled[1]) continue;
            active[index++] = leg;
        }
        return active;
    }

    private static int validLeg(int value, int fallback) {
        return value >= 0 && value < 6 ? value : fallback;
    }

    private static double[] parsePair(String command) {
        int colon = command.indexOf(':');
        if (colon < 0 || colon + 1 >= command.length()) return null;
        String[] parts = command.substring(colon + 1).split(",");
        if (parts.length < 2) return null;
        try {
            return new double[] {Double.parseDouble(parts[0]), Double.parseDouble(parts[1])};
        } catch (NumberFormatException error) {
            return null;
        }
    }

    private static double[] parseTriple(String command) {
        int colon = command.indexOf(':');
        if (colon < 0 || colon + 1 >= command.length()) return null;
        String[] parts = command.substring(colon + 1).split(",");
        if (parts.length < 3) return null;
        try {
            return new double[] {
                    Double.parseDouble(parts[0]),
                    Double.parseDouble(parts[1]),
                    Integer.parseInt(parts[2])
            };
        } catch (NumberFormatException error) {
            return null;
        }
    }

    private static double[] parseTripleDouble(String command) {
        int colon = command.indexOf(':');
        if (colon < 0 || colon + 1 >= command.length()) return null;
        String[] parts = command.substring(colon + 1).split(",");
        if (parts.length < 3) return null;
        try {
            return new double[] {
                    Double.parseDouble(parts[0]),
                    Double.parseDouble(parts[1]),
                    Double.parseDouble(parts[2])
            };
        } catch (NumberFormatException error) {
            return null;
        }
    }

    private boolean stepOriginalGait() {
        long now = System.currentTimeMillis();
        double dtMs = Math.max(0.0d, now - lastStepMillis) * currentMode().speed;
        boolean transitioningToStop = pendingWalkClear;
        boolean walking = activeWalk || transitioningToStop;
        boolean stopping = pendingStopStep;
        if (!relayStatus || !standing || (!walking && !stopping)) return false;
        dtMs = originalFrameDt(walkStepCount, dtMs);
        lastStepMillis = now;
        boolean allowGait = walking || walkMagnitude(filteredWalk) > ORIGINAL_STOP_VECTOR_THRESHOLD;
        lastPulses = mergeActiveLegPulses(gaitEngine.step(gaitForMode(), animation,
                filteredWalk.forward, filteredWalk.strafe, filteredWalk.turn, dtMs, allowGait));
        walkStepCount++;
        publishOriginalFrame();
        if (MOTION_TRACE_ENABLED) Log.i("CHICA_GAIT", gaitEngine.lastCompactTraceJson());

        if (transitioningToStop) {
            // z0.e finishes one frame with its worker-local target before it
            // observes shared f.g == null and begins 0.9 decay next iteration.
            pendingWalkClear = false;
            pendingStopStep = true;
        } else if (walking) {
            // z0.e blends the next frame's worker-local command only after the
            // current gait step, so target updates never reset phase or dt.
            filteredWalk = lerpWalk(filteredWalk, lastWalk, 0.05d);
        } else if (allowGait) {
            filteredWalk = scaleWalk(filteredWalk, 0.9d);
        } else if (!gaitEngine.hasActiveWalkAnchors()) {
            pendingStopStep = false;
            pendingWalkClear = false;
            filteredWalk = new WalkVector(0.0d, 0.0d, 0.0d);
            walkStepCount = 0;
            return false;
        }
        return true;
    }

    private void startOriginalWalkWorkerLocked() {
        if (walkWorkerRunning) return;
        walkWorkerRunning = true;
        final boolean preserveKeptQuadrupedPose = keepMode && modeIndex == 4;
        Thread thread = new Thread(() -> {
            while (true) {
                synchronized (ChicaController.this) {
                    if (!stepOriginalGait()) {
                        break;
                    }
                }
                sleepOriginalPoseStep(ORIGINAL_POSE_STEP_MS);
                if (Thread.currentThread().isInterrupted()) {
                    synchronized (ChicaController.this) {
                        walkWorkerRunning = false;
                    }
                    return;
                }
            }
            // z0.e skips p3.a.p() only for a kept quadruped pose (gait id 20).
            if (!preserveKeptQuadrupedPose) {
                publishOriginalWalkLayerFade();
            }
            synchronized (ChicaController.this) {
                walkWorkerRunning = false;
            }
        }, "chica-original-walk");
        thread.setDaemon(true);
        thread.start();
    }

    private void publishOriginalWalkLayerFade() {
        double magnitude = gaitEngine.beginWalkLayerFade();
        long previous = System.currentTimeMillis();
        while (Double.isFinite(magnitude) && magnitude > 0.05000000074505806d) {
            long now = System.currentTimeMillis();
            double amount = currentMode().speed * 0.1d * (double) (now - previous);
            if (amount >= magnitude) break;
            synchronized (this) {
                lastPulses = mergeActiveLegPulses(gaitEngine.stepWalkLayerFade(amount));
                publishOriginalFrame();
            }
            sleepOriginalPoseStep(ORIGINAL_POSE_STEP_MS);
            magnitude -= amount;
            previous = now;
        }
        synchronized (this) {
            lastPulses = mergeActiveLegPulses(gaitEngine.finishWalkLayerFade());
            publishOriginalFrame();
        }
    }

    private static double originalFrameDt(int stepCount, double measuredDtMs) {
        if (stepCount == 0) return 0.0d;
        return measuredDtMs;
    }

    private void publishOriginalFrame() {
        servoBackend.stageServoPulses(lastPulses);
        if (MOTION_TRACE_ENABLED) Log.i("CHICA_SERVO", Arrays.toString(lastPulses));
        frameCount++;
        updatePanelBps();
    }

    private void requestOriginalWalkStop(boolean logTargetNull) {
        if (logTargetNull) {
            logControl("walkclear_target_null", null);
        }
        if (standing && relayStatus && walkWorkerRunning) {
            pendingStopStep = false;
            pendingWalkClear = true;
            activeWalk = false;
            lastWalk = new WalkVector(0.0d, 0.0d, 0.0d);
            return;
        }
        pendingStopStep = false;
        pendingWalkClear = false;
        activeWalk = false;
        lastWalk = new WalkVector(0.0d, 0.0d, 0.0d);
        filteredWalk = new WalkVector(0.0d, 0.0d, 0.0d);
        walkStepCount = 0;
    }

    private int gaitForMode() {
        // Quad uses the dedicated Quad gait (apk id 20): legs 0,2,3,5 swing in a
        // quad amble, legs 1,4 never swing. Without this, quad falls back to the
        // walk-style gait (Tripod) which flails 4 legs in a 6-leg pattern.
        if (modeIndex == 4) {
            return 20;
        }
        // Walk styles map directly to apk gait ids (walkModeIndex already IS the
        // apk gait id): walk3=5 Tripod, walk2=6, walk1=7, walk15=8, walk25=9,
        // walkwave=10. The old switch only handled 5/6/7/9 and dropped walk15(8)
        // and walkwave(10) to the mode default, so those two styles silently
        // walked the wrong gait. apkGaitFromJava passes 5-10 through.
        if (walkModeIndex >= 5 && walkModeIndex <= 10) {
            return walkModeIndex;
        }
        return clampGait(gait);
    }

    private static int clampGait(int gait) {
        if (gait < 1) return 1;
        if (gait > 4) return 4;
        return gait;
    }

    private void updatePanelBps() {
        long now = System.currentTimeMillis();
        long elapsed = now - panelBpsWindowStartMillis;
        if (elapsed >= 1000L) {
            panelBps = (panelBpsWindowFrames * 1000.0d) / elapsed;
            panelBpsWindowFrames = 0;
            panelBpsWindowStartMillis = now;
        }
    }

    // Immediate V/I text-colour zone, matching the original z0.d threshold
    // colouring (compared against the live value, not the 2 s duration-gated
    // warning that drives the warning boxes). NaN telemetry -> OK/green, since
    // the comparisons are false.
    private int voltageColorZone() {
        if (lastVoltage < voltageCutoffLevel) return SurfaceStatus.ZONE_CRITICAL;
        if (lastVoltage < voltageWarningLevel) return SurfaceStatus.ZONE_WARN;
        return SurfaceStatus.ZONE_OK;
    }

    private int currentColorZone() {
        if (lastCurrent > currentCutoffLevel) return SurfaceStatus.ZONE_CRITICAL;
        if (lastCurrent > currentWarningLevel) return SurfaceStatus.ZONE_WARN;
        return SurfaceStatus.ZONE_OK;
    }

    private static double clampStick(double value) {
        return clampUnit(value);
    }

    // Mirrors z0.m unit-magnitude clamping (e()/h()): scale the (x, y) pair down
    // together when its Euclidean length exceeds 1, preserving direction. For
    // in-disk input (length <= 1) the values pass through unchanged, matching the
    // per-axis clamp for the verified nominal range.
    private static double[] clampStickPair(double x, double y) {
        double magnitude = Math.sqrt(x * x + y * y);
        if (magnitude > 1.0d) {
            x /= magnitude;
            y /= magnitude;
        }
        return new double[] {x, y};
    }

    private static double clampUnit(double value) {
        if (Double.isNaN(value)) return 0.0d;
        if (value > 1.0d) return 1.0d;
        if (value < -1.0d) return -1.0d;
        return value;
    }

    private static int[] neutralPulses() {
        int[] pulses = new int[18];
        for (int i = 0; i < pulses.length; i++) pulses[i] = 1500;
        return pulses;
    }

    private static String jsonEscape(String value) {
        if (value == null) return "";
        StringBuilder out = new StringBuilder(value.length() + 8);
        for (int i = 0; i < value.length(); i++) {
            char c = value.charAt(i);
            if (c == '"' || c == '\\') {
                out.append('\\').append(c);
            } else if (c == '\n') {
                out.append("\\n");
            } else if (c == '\r') {
                out.append("\\r");
            } else if (c == '\t') {
                out.append("\\t");
            } else {
                out.append(c);
            }
        }
        return out.toString();
    }

    private static double originalTelemetryBlend(double previous, double next) {
        if (Double.isNaN(previous) || Double.isNaN(next)) return next;
        return (previous * 0.8d) + (next * 0.2d);
    }

    private void parseOriginalWarningConfig(String config) {
        Scanner scanner = new Scanner(config == null ? "" : config);
        scanner.useLocale(Locale.ENGLISH);
        while (scanner.hasNextLine()) {
            String line = scanner.nextLine().trim();
            if (line.isEmpty() || line.startsWith("#")) continue;
            Scanner lineScanner = new Scanner(line);
            lineScanner.useLocale(Locale.ENGLISH);
            if (!lineScanner.hasNext()) {
                lineScanner.close();
                continue;
            }
            String key = lineScanner.next();
            if ("WARN_VOL".equals(key)) {
                if (lineScanner.hasNextDouble()) voltageWarningDuration = lineScanner.nextDouble();
                if (lineScanner.hasNextDouble()) voltageWarningLevel = lineScanner.nextDouble();
                if (lineScanner.hasNextDouble()) voltageCutoffLevel = lineScanner.nextDouble();
                if (lineScanner.hasNextInt()) voltageBeepCount = lineScanner.nextInt();
            } else if ("WARN_CUR".equals(key)) {
                if (lineScanner.hasNextDouble()) currentWarningDuration = lineScanner.nextDouble();
                if (lineScanner.hasNextDouble()) currentWarningLevel = lineScanner.nextDouble();
                if (lineScanner.hasNextDouble()) currentCutoffLevel = lineScanner.nextDouble();
                if (lineScanner.hasNextInt()) currentBeepCount = lineScanner.nextInt();
            }
            lineScanner.close();
        }
        scanner.close();
        resetOriginalWarningTimers(System.currentTimeMillis());
    }

    private void updateOriginalWarnings() {
        long now = System.currentTimeMillis();
        if (Double.isNaN(lastCurrent) || !relayStatus) {
            resetOriginalWarningTimers(now);
            voltageWarning = false;
            currentWarning = false;
            cutoffTorquePending = false;
            return;
        }
        if (lastCurrent < currentWarningLevel) currentWarningSinceMillis = now;
        if (lastCurrent < currentCutoffLevel) currentCutoffSinceMillis = now;
        if (lastVoltage > voltageWarningLevel) voltageWarningSinceMillis = now;
        if (lastVoltage > voltageCutoffLevel) voltageCutoffSinceMillis = now;

        currentWarning = elapsedPast(now, currentWarningSinceMillis, currentWarningDuration);
        voltageWarning = elapsedPast(now, voltageWarningSinceMillis, voltageWarningDuration);
        boolean currentCutoff = elapsedPast(now, currentCutoffSinceMillis, currentWarningDuration);
        boolean voltageCutoff = elapsedPast(now, voltageCutoffSinceMillis, voltageWarningDuration);
        if (currentCutoff || voltageCutoff) {
            // Original z0.d: f7069e = 6 (six beeps) and ENQUEUE the "torque"
            // command (f7079p -> executor), which runs the full toggle path —
            // sit first if standing, THEN relay off — a graceful shutdown, not a
            // hard relay cut. One command in flight at a time (f7078o gate);
            // retried every tick until the relay actually drops (e.g. if the
            // toggle bailed on originalMotionBusy), matching the original's
            // continuous re-enqueue. Relay-off resets the latch above.
            if (!cutoffTorquePending || !busy) {
                cutoffTorquePending = true;
                beepCount = 6;
                submitOriginalCommand("torque");
            }
        } else if (voltageBeepCount > 0 && voltageWarning) {
            beepCount = voltageBeepCount;  // original: f7069e = h.D
        } else if (currentBeepCount > 0 && currentWarning) {
            beepCount = currentBeepCount;  // original: f7069e = h.f7113w
        }
    }

    private void resetOriginalWarningTimers(long now) {
        currentWarningSinceMillis = now;
        currentCutoffSinceMillis = now;
        voltageWarningSinceMillis = now;
        voltageCutoffSinceMillis = now;
    }

    private static boolean elapsedPast(long now, long since, double seconds) {
        return ((double) (now - since)) > seconds * 1000.0d;
    }

    private ModeParams currentMode() {
        int index = Math.max(0, Math.min(modeIndex, ORIGINAL_MODES.length - 1));
        return ORIGINAL_MODES[index];
    }

    private static WalkVector lerpWalk(WalkVector from, WalkVector to, double amount) {
        return new WalkVector(
                lerp(from.forward, to.forward, amount),
                lerp(from.strafe, to.strafe, amount),
                lerp(from.turn, to.turn, amount));
    }

    private static WalkVector scaleWalk(WalkVector from, double scale) {
        return new WalkVector(
                from.forward * scale,
                from.strafe * scale,
                from.turn * scale);
    }

    private static double walkMagnitude(WalkVector vector) {
        return Math.sqrt((vector.forward * vector.forward)
                + (vector.strafe * vector.strafe)
                + (vector.turn * vector.turn));
    }

    private static double lerp(double from, double to, double amount) {
        return ((1.0d - amount) * from) + (to * amount);
    }

    private static final class ModeParams {
        final double radius;
        final double cornerAngle;
        final double elongation;
        final double bodyLift;
        final double stepLift;
        final double speed;
        final double animationFactor;

        ModeParams(double radius,
                   double cornerAngle,
                   double elongation,
                   double bodyLift,
                   double stepLift,
                   double speed,
                   double animationFactor) {
            this.radius = radius;
            this.cornerAngle = cornerAngle;
            this.elongation = elongation;
            this.bodyLift = bodyLift;
            this.stepLift = stepLift;
            this.speed = speed;
            this.animationFactor = animationFactor;
        }
    }
}
