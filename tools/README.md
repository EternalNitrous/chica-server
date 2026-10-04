# tools

the harnesses I used to reconstruct and verify ChicaServer against the original
app. Captured reference traces stay outside the public source tree. The runner
reads comparator fixtures from `research-private/oracle/` (or
`CHICA_ORACLE_DIR`) and discovers saved original APK `CHICA_GT` captures in
`research-private/server-apk-analysis/` (or `CHICA_CAPTURE_DIR`).

| path | what's inside |
| :--- | :------------ |
| `run_exactness_regression.py` | the entry point — runs available checks and summarizes PASS, BOUNDED, SKIP, and FAIL |
| `capture/` | captures original APK `CHICA_GT` records and instrumented runtime logs (logcat / Frida hooks, coverage) |
| `oracle/` | compares gait, quad, walk, animation, calibration, APK pose/PWM captures, and runtime timing; includes C++ probes |
| `device/` | the board protocol model, the socket/TCP fakes, the emulator↔hardware serial bridge, and the protocol / no-hardware verifiers |

An absent capture is reported as `SKIP`; it is not treated as an app mismatch.
The runner exits `0` only when all configured checks pass their assertions, `1` for a
failed check, and `2` when evidence is incomplete or a comparison is bounded.
Use `--require-all-oracles` to fail when any fixture is missing.

`verify_set_pose_model.py` tests the actual native B/G helpers against separately
written formulae from the APK: 54,000 frames, varying frame times, inputs and
femur scales, with a `1e-10` pose tolerance. Its PWM check shares IK and conversion
code; it does not independently establish their parity with the APK.
`verify_set_worker_jni.py` runs the shipped Java engine and JNI library on a host
JVM. It checks independent worker velocities/angles, simultaneous fade contexts,
keep folding and calibration reset. These deterministic checks cover state
handling, not Android scheduling. Both need a JDK with JNI headers and a C++ compiler.
The runtime source-contract check only guards statements derived from the APK;
its passing result is not a measurement of runtime timing.

`compare_gt_kinematics.py` replays each complete original `CHICA_GT` pose
through the current native IK and pulse-conversion code. Original pose vectors
are logged at limited decimal precision; the comparator propagates the logged
rounding resolution through the native probe. Quantization-compatible pulse
deltas and dropped capture records are `BOUNDED`, never exact passes. It checks
frame pairing and all 18 pulse channels; it does not claim gait timing or
command-path parity.
New precise captures also record the actual enabled-leg mask and retained joint
angles. The native replay preserves those parked angles. Legacy quadruped
captures without that metadata are `BOUNDED`; treating their six legs as enabled
would produce a false IK/PWM failure. APK pose logging must run inside the
original frame lock to keep each pose and PWM record paired during concurrent
workers. Capture metadata includes the installed APK's SHA-256.

`compare_walk_runtime_replay.py` replays each saved instrumented app gait frame
from its logged gait/style/dt/allow/command inputs and compares all 18 emitted
PWM channels. Its command vector is rounded in the diagnostic log, so a
one-microsecond delta is also `BOUNDED`. This checks app-to-engine consistency,
not original APK parity.
When sit/home/mode animations overlap gait frames, the walk-only replay checks
the prefix before that external animation and marks remaining coverage
`BOUNDED`. Replaying the whole interval without those pose-state changes would
produce a false gait mismatch. This does not certify the concurrent path.

`compare_apk_runtime_capture.py` discovers matching `gt_jsonl_<name>.jsonl` and
`rebuild_jsonl_<name>.jsonl` runtime captures. It requires exact command and
FLAGS transition sequences, then compares body, animation layer, and all PWM
channels in separate intervals for each command. New control captures also
number every TCP request (commands and ACKs) and pair replies in stream order.
The comparator checks each command's READY/BUSY disposition and reports returned
FLAGS snapshots separately because the server dispatches commands asynchronously.
For isolated checks, make one command per capture, use `--no-heartbeat`, and
allow at least 1.5 seconds after the command; `--status-probe-after-ms` sends
ACK probes at the supplied offsets during the action and stops probing after the
first READY response. BUSY probes are rejected without advancing ACK-ramp
behavior; an accepted READY probe does advance it. Repeated ACKs are not a
neutral readiness check. The legacy `--wait-ready-before-command` path therefore
remains stress diagnostics, not strict evidence. Captures made before
request/reply IDs were added report this evidence as unavailable. Marker writes
run outside the TCP/ACK loop, and each capture
records the host time when its `adb log` call returns. The comparator uses that
paired timestamp to estimate the host/logcat clock offset without treating adb
launch latency as command latency. It reports raw nearest-frame deltas and a separate trajectory
comparison after fitting one constant timing offset per command. That second
comparison helps distinguish sampling/update jitter from a different pose path,
but it does not hide command-to-output delay. Each command's comparison window
is capped at its requested step duration, so the extra logcat tail after the
capture client disconnects is not attributed to the last command. READY-gated
captures with complete paired replies also fail when the time-aligned pose path
still has a large residual; ungated captures remain diagnostic because
overlapping asynchronous commands can race. Runtime results remain `BOUNDED`
when separate-run sampling, event timing, or capture precision prevents a
frame-for-frame proof. Terminal PWM and pose deltas are reported separately so
matching settled outputs remain visible when transient trajectories are not
sample-aligned.

The regression runner prints route coverage from `ChicaController` beside the
PASS/BOUNDED/SKIP/FAIL totals. A route listed as captured only means a paired
original/rebuild command log exercised it; the per-capture comparator result
determines what that evidence establishes. The native gait differential is a
separate check against an APK-derived independent transliteration, not a direct
original-APK runtime comparison. By default, the runner summarizes the
original-pose replay, saved-walk replay, and live-capture component checks
instead of printing every frame interval. Use `--verbose` to show the
per-capture and per-command details.
The summary reports command-sequence, FLAGS-transition, READY/BUSY-reply, status
probe, and capture-plan parity separately; none of those counts imply that the
pose/PWM trajectory is exact.

The original TCP socket times out after one second without a line. For longer
captures without ACKs, combine `--no-heartbeat --empty-line-keepalive`. The tool
sends empty lines every 250 ms; the original ignores them for motion, replies
remain numbered, and capture metadata records that traffic for plan comparison.

`device/run_servo2040_tcp_fake.py` accepts `--timing-out` to timestamp complete
18-channel packets and `--reply-delay-ms` to test the original heartbeat under
controlled telemetry latency.

```bash
# run the full exactness regression
python3 tools/run_exactness_regression.py

# require every optional capture set to be present
python3 tools/run_exactness_regression.py --require-all-oracles

# build an instrumented APK for live motion capture
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
  ./gradlew -PmotionTrace=true :app:assembleDebug

# bridge the Android emulator to a real board on the host's USB
python3 tools/device/run_servo2040_serial_bridge.py --device /dev/cu.usbmodemXXXX
```
