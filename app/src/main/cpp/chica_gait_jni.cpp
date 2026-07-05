#include "apk_model.h"
#include "pulse_conversion.h"

#include <jni.h>
#include <cerrno>
#include <algorithm>
#include <array>
#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

enum class TimedAnimationKind {
    None,
    PoseRamp,
    BodyRamp,
    ShapeRamp,
    CogLean,
};

struct TimedAnimation {
    TimedAnimationKind kind = TimedAnimationKind::None;
    apk_model::BodyState start;
    apk_model::BodyState target;
    std::vector<int> movingLegs;
    apk_model::Pose cogStart;
    apk_model::Pose cogTarget;
    double lift = 0.0;
    double layerBlend = 0.0;
    double durationMs = 1.0;
};

struct Engine {
    apk_model::RobotConfig config = apk_model::makeDefaultConfig();
    apk_model::WalkState state;
    std::array<apk_model::Pose, 4> layers = {};
    TimedAnimation timed;
    apk_model::WalkStepResult lastStep;
    apk_model::Pose setVelocity;
    double sweepAngle = 0.0;
    double sweepDR = 0.0;
    double sweepDS = 0.0;
    std::array<int, 18> lastPulses = {};
    // Persistent last-committed joint angles per leg. Active legs are refreshed
    // every IK flush / walk step; PARKED (disabled) legs keep their frozen tuck
    // angles because inverseKinematics skips inactive legs. Forward-kinematics
    // off this array re-seats the feet on leg re-enable (original
    // z0.a.a(null) -> z0.j.a()), fixing the parked legs' stale stored world
    // position after the body has drifted during a walk.
    std::array<std::array<double, 3>, 6> lastAngles = {};
    int lastJavaGait = 1;
    int lastApkGait = 5;
    // Which legs are active (used). Hexapod = all 6; quad = the 4 enabled legs.
    // walkStep needs this so body IK only solves the active legs (parked legs
    // are tucked at disabled-Z and unreachable, which otherwise makes every
    // body-translation IK fail -> no travel) and so the gait-20 remap targets
    // the correct legs. Mirrors the original z0.a.c()/f7054d enabled mask.
    std::array<bool, 6> activeLegs = {true, true, true, true, true, true};
    int lastAnimation = 0;
    double lastDtMs = 0.0;
    bool lastAllowNewAnchors = true;
    apk_model::BodyState walkLayerFadeBody;
    bool walkLayerFadeActive = false;

    Engine()
    {
        apk_model::initializeWalkState(config, state);
        for (auto& a : lastAngles) a = {0.0, 90.0, 120.0};
    }
};

int apkGaitFromJava(int javaGait)
{
    if (javaGait == 20) return 20; // Quad gait (Quad phase table)
    // Walk styles already pass the apk gait id directly (5-10): Tripod, Triple,
    // Ripple, Ripple15, Triple25, Wave. Pass them through (matches iOS bridge).
    if (javaGait >= 5 && javaGait <= 10) return javaGait;
    switch (javaGait) {
        case 2: return 9;  // Ripple 2.5 / RippleExt
        case 3: return 6;  // Ripple
        case 4: return 7;  // Amble / walk1
        default: return 5; // Tripod
    }
}

std::array<int, 18> toPulses(const apk_model::WalkStepResult& step)
{
    std::array<int, 18> pulses = {};
    const ChicaServoConfig& cfg = chica_apk_servo_config();
    for (int leg = 0; leg < 6; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            int pin = cfg.pin[leg][joint];
            pulses[pin] = chica_apk_angle_to_pulse(step.angles_deg[leg][joint], leg, joint);
        }
    }
    return pulses;
}

std::array<int, 18> toPulses(const std::array<std::array<double, 3>, 6>& angles)
{
    std::array<int, 18> pulses = {};
    const ChicaServoConfig& cfg = chica_apk_servo_config();
    for (int leg = 0; leg < 6; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            int pin = cfg.pin[leg][joint];
            pulses[pin] = chica_apk_angle_to_pulse(angles[leg][joint], leg, joint);
        }
    }
    return pulses;
}

std::array<int, 18> toPulsesFromPose(Engine& engine)
{
    // Seed from the persisted angles so PARKED legs (skipped by IK below) keep
    // their frozen tuck angles; the IK refreshes only the active legs.
    std::array<std::array<double, 3>, 6> angles = engine.lastAngles;
    // Active-leg mask, not all-true: quad's parked legs are tucked/unreachable;
    // an all-true IK fails and freezes on stale pulses (original IKs only the
    // enabled legs, z0.a.c()/f7054d).
    std::array<bool, 6> active = engine.activeLegs;
    apk_model::Pose combined = {};
    for (const auto& layer : engine.layers) {
        combined = apk_model::addPose(combined, layer);
    }
    // Include the quad CoG layer (original layer[2]) like j.e sums all layers.
    combined = apk_model::addPose(combined, engine.state.cog_layer);
    engine.state.animation_layer = combined;
    if (!apk_model::inverseKinematics(engine.config,
                                      engine.state.body,
                                      combined,
                                      active,
                                      angles)) {
        return engine.lastPulses;
    }
    engine.lastAngles = angles;  // persist (active refreshed, parked preserved)
    engine.lastPulses = toPulses(angles);
    return engine.lastPulses;
}

jintArray toPulseArray(JNIEnv* env, const std::array<int, 18>& pulses)
{
    jintArray out = env->NewIntArray(static_cast<jsize>(pulses.size()));
    std::array<jint, 18> jintPulses = {};
    for (size_t i = 0; i < pulses.size(); ++i) {
        jintPulses[i] = pulses[i];
    }
    env->SetIntArrayRegion(out, 0, static_cast<jsize>(jintPulses.size()), jintPulses.data());
    return out;
}

void appendFrame(std::vector<jint>& out, const std::array<int, 18>& pulses)
{
    for (int pulse : pulses) {
        out.push_back(pulse);
    }
}

jintArray toFlatFrameArray(JNIEnv* env, const std::vector<jint>& frames)
{
    jintArray out = env->NewIntArray(static_cast<jsize>(frames.size()));
    if (!frames.empty()) {
        env->SetIntArrayRegion(out, 0, static_cast<jsize>(frames.size()), frames.data());
    }
    return out;
}

Engine* fromHandle(jlong handle)
{
    return reinterpret_cast<Engine*>(handle);
}

double correctedAngleForTrace(double angleDeg, int leg, int joint)
{
    static constexpr std::array<double, 6> coxaAttach = {-8.0, 0.0, 8.0, -8.0, 0.0, 8.0};
    if (joint == 0) return (angleDeg - coxaAttach[leg]) * -1.0;
    if (joint == 1) return angleDeg - 35.0;
    return (angleDeg * -1.0) + 68.0;
}

std::array<apk_model::Vec3, 6> neutralFeet(double radius,
                                           double z,
                                           double cornerAngleDeg,
                                           double elongation,
                                           const std::vector<int>& active = std::vector<int>(apk_model::ApkLegOrder.begin(), apk_model::ApkLegOrder.end()))
{
    return apk_model::makeNeutralFeet(radius, z, cornerAngleDeg, elongation, active);
}

void appendVec(std::ostringstream& out, const apk_model::Vec3& value)
{
    out << "{\"x\":" << value.x
        << ",\"y\":" << value.y
        << ",\"z\":" << value.z << "}";
}

void appendPose(std::ostringstream& out, const apk_model::Pose& pose)
{
    out << "{\"xyz\":";
    appendVec(out, pose.xyz);
    out << ",\"uvw\":";
    appendVec(out, pose.uvw);
    out << "}";
}

void appendAngles(std::ostringstream& out, const std::array<std::array<double, 3>, 6>& angles)
{
    out << "[";
    for (int leg = 0; leg < 6; ++leg) {
        if (leg > 0) out << ",";
        out << "[" << angles[leg][0] << "," << angles[leg][1] << "," << angles[leg][2] << "]";
    }
    out << "]";
}

void appendFeet(std::ostringstream& out, const std::array<apk_model::Vec3, 6>& feet)
{
    out << "[";
    for (int leg = 0; leg < 6; ++leg) {
        if (leg > 0) out << ",";
        appendVec(out, feet[leg]);
    }
    out << "]";
}

apk_model::Pose lerpPose(const apk_model::Pose& from, const apk_model::Pose& to, double t)
{
    return {
        {
            from.xyz.x + ((to.xyz.x - from.xyz.x) * t),
            from.xyz.y + ((to.xyz.y - from.xyz.y) * t),
            from.xyz.z + ((to.xyz.z - from.xyz.z) * t),
        },
        {
            from.uvw.x + ((to.uvw.x - from.uvw.x) * t),
            from.uvw.y + ((to.uvw.y - from.uvw.y) * t),
            from.uvw.z + ((to.uvw.z - from.uvw.z) * t),
        },
    };
}

double poseMagnitude(const apk_model::Vec3& value)
{
    return std::sqrt((value.x * value.x) + (value.y * value.y) + (value.z * value.z));
}

void normalizePoseVectors(apk_model::Pose& pose)
{
    double xyzMag = poseMagnitude(pose.xyz);
    if (xyzMag > 1.0) {
        pose.xyz.x /= xyzMag;
        pose.xyz.y /= xyzMag;
        pose.xyz.z /= xyzMag;
    }
    double uvwMag = poseMagnitude(pose.uvw);
    if (uvwMag > 1.0) {
        pose.uvw.x /= uvwMag;
        pose.uvw.y /= uvwMag;
        pose.uvw.z /= uvwMag;
    }
}

void scalePoseComponents(apk_model::Pose& pose, double scale = 1.0)
{
    pose.xyz.x *= 60.0 * scale;
    pose.xyz.y *= 100.0 * scale;
    pose.xyz.z *= 100.0 * scale;
    pose.uvw.x *= 28.0 * scale;
    pose.uvw.y *= 18.0 * scale;
    pose.uvw.z *= 18.0 * scale;
}

void clampPose(apk_model::Pose& pose, double scale = 1.0)
{
    double x = 60.0 * scale, y = 100.0 * scale, z = 100.0 * scale;
    double u = 28.0 * scale, v = 18.0 * scale, w = 18.0 * scale;
    pose.xyz.x = std::min(x, std::max(-x, pose.xyz.x));
    pose.xyz.y = std::min(y, std::max(-y, pose.xyz.y));
    pose.xyz.z = std::min(z, std::max(-z, pose.xyz.z));
    pose.uvw.x = std::min(u, std::max(-u, pose.uvw.x));
    pose.uvw.y = std::min(v, std::max(-v, pose.uvw.y));
    pose.uvw.z = std::min(w, std::max(-w, pose.uvw.z));
}

void scalePoseInPlace(apk_model::Pose& pose, double amount)
{
    pose.xyz.x *= amount;
    pose.xyz.y *= amount;
    pose.xyz.z *= amount;
    pose.uvw.x *= amount;
    pose.uvw.y *= amount;
    pose.uvw.z *= amount;
}

apk_model::Pose subtractPose(const apk_model::Pose& left, const apk_model::Pose& right)
{
    return {
        {left.xyz.x - right.xyz.x, left.xyz.y - right.xyz.y, left.xyz.z - right.xyz.z},
        {left.uvw.x - right.uvw.x, left.uvw.y - right.uvw.y, left.uvw.z - right.uvw.z},
    };
}

apk_model::Vec3 lerpVec(const apk_model::Vec3& from, const apk_model::Vec3& to, double t)
{
    return {
        from.x + ((to.x - from.x) * t),
        from.y + ((to.y - from.y) * t),
        from.z + ((to.z - from.z) * t),
    };
}

void scalePose(apk_model::Pose& pose, double amount)
{
    pose.xyz.x *= amount;
    pose.xyz.y *= amount;
    pose.xyz.z *= amount;
    pose.uvw.x *= amount;
    pose.uvw.y *= amount;
    pose.uvw.z *= amount;
}

apk_model::Pose originalAnimationPose(const Engine& engine, double phase, int animation)
{
    double angle = 3.14159265358979323846 * phase * 2.0;
    apk_model::Pose pose = {};
    switch (animation) {
        case 1:
            pose.xyz.z = (-std::cos(angle) * 60.0) + 30.0;
            pose.uvw.z = std::sin(angle) * 15.0;
            break;
        case 2:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.uvw.y = std::sin(angle) * 15.0;
            break;
        case 3:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.uvw.x = -std::sin(angle) * 15.0;
            break;
        case 4:
            pose.xyz.y = -std::cos(angle) * 60.0;
            pose.uvw.z = -std::sin(angle) * 15.0;
            break;
        case 5:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.uvw.y = std::cos(angle) * 15.0;
            pose.uvw.z = -7.0;
            break;
        case 6:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.xyz.y = std::sin(angle) * 50.0;
            break;
        case 7:
            pose.xyz.x = -std::cos(angle) * 40.0;
            pose.xyz.y = std::sin(angle) * 50.0;
            pose.uvw.y = std::cos(angle) * 12.0;
            pose.uvw.z = std::sin(angle) * 12.0;
            break;
        default: {
            // anim0 body bob = half the ENABLED-leg foot-z spread. Excluding
            // disabled legs is essential in quad: they tuck ~65mm above the body
            // and would otherwise inflate the spread ~4x (body rides very high).
            double minZ = 1.7976931348623157E308;
            double maxZ = -1.7976931348623157E308;
            for (int leg : apk_model::ApkLegOrder) {
                if (!engine.activeLegs[leg]) continue;
                minZ = std::min(minZ, engine.state.body.feet[leg].z);
                maxZ = std::max(maxZ, engine.state.body.feet[leg].z);
            }
            if (minZ <= maxZ) pose.xyz.z = (maxZ - minZ) / 2.0;
            break;
        }
    }
    scalePose(pose, 1.0);
    return pose;
}

bool beginPoseRamp(Engine& engine,
                   const std::vector<int>& requested,
                   double threshold,
                   double lift,
                   double layerBlend,
                   double durationMs)
{
    TimedAnimation timed;
    timed.kind = TimedAnimationKind::PoseRamp;
    timed.start = engine.state.body;
    timed.target = timed.start;
    timed.lift = lift;
    timed.layerBlend = layerBlend;
    timed.durationMs = std::max(1.0, durationMs);

    for (int leg : requested) {
        apk_model::Vec3 neutral = apk_model::neutralFootForBody(engine.config, timed.start, {}, leg);
        timed.target.feet[leg] = neutral;
        double dx = neutral.x - timed.start.feet[leg].x;
        double dy = neutral.y - timed.start.feet[leg].y;
        if (threshold < 0.0 || ((dx * dx) + (dy * dy)) > (threshold * threshold)) {
            timed.movingLegs.push_back(leg);
        }
    }

    if (timed.movingLegs.empty()) {
        engine.timed = {};
        return false;
    }
    engine.timed = timed;
    return true;
}

bool beginBodyRamp(Engine& engine, double bodyZ, double durationMs)
{
    TimedAnimation timed;
    timed.kind = TimedAnimationKind::BodyRamp;
    timed.start = engine.state.body;
    timed.target = timed.start;
    timed.target.body.xyz.z = bodyZ;
    timed.durationMs = std::max(1.0, durationMs);
    engine.timed = timed;
    return true;
}

bool beginBodyDeltaRamp(Engine& engine, double bodyZDelta, double durationMs)
{
    TimedAnimation timed;
    timed.kind = TimedAnimationKind::BodyRamp;
    timed.start = engine.state.body;
    timed.target = timed.start;
    timed.target.body.xyz.z += bodyZDelta;
    timed.durationMs = std::max(1.0, durationMs);
    engine.timed = timed;
    return true;
}

bool beginShapeRamp(Engine& engine,
                    double radius,
                    double z,
                    double cornerAngleDeg,
                    double elongation,
                    double durationMs,
                    const std::vector<int>& requested = {})
{
    TimedAnimation timed;
    timed.kind = TimedAnimationKind::ShapeRamp;
    timed.start = engine.state.body;
    timed.target = timed.start;
    const std::vector<int> allLegs(apk_model::ApkLegOrder.begin(), apk_model::ApkLegOrder.end());
    const std::vector<int>& legs = requested.empty() ? allLegs : requested;
    auto feet = neutralFeet(radius, z, cornerAngleDeg, elongation, allLegs);
    // Transform the body-relative neutral into the world frame of the CURRENT
    // body pose (same as neutralFootForBody, which the pose-ramp/home-pose use).
    // Without this the shape-ramp targets origin-relative spots, so after the
    // body has drifted from the origin (e.g. walking before a mode change) the
    // legs get dragged by the body offset and the transition is malformed.
    const apk_model::Pose& body = timed.start.body;
    for (int leg : legs) {
        if (leg < 0 || leg >= 6) continue;
        apk_model::Vec3 foot = feet[leg];
        apk_model::rotateDegrees(foot, body.uvw.x, 0.0, 0.0);
        foot.x += body.xyz.x;
        foot.y += body.xyz.y;
        timed.target.feet[leg] = foot;
    }
    timed.durationMs = std::max(1.0, durationMs);
    engine.timed = timed;
    return true;
}

std::array<int, 18> sampleTimedAnimation(Engine& engine, double elapsedMs)
{
    if (engine.timed.kind == TimedAnimationKind::None) {
        return toPulsesFromPose(engine);
    }

    double elapsed = std::max(0.0, elapsedMs);
    double t = std::min(1.0, elapsed / engine.timed.durationMs);
    apk_model::BodyState next = engine.timed.start;

    if (engine.timed.kind == TimedAnimationKind::CogLean) {
        // p3.a.M single-leg quad branch: layer2 = lerp(start, target, r5.g(t))
        // flushed every frame; the swing ramp follows as a separate phase.
        double eased = std::sin(t * M_PI / 2.0);
        engine.state.cog_layer = lerpPose(engine.timed.cogStart, engine.timed.cogTarget, eased);
        std::array<int, 18> pulses = toPulsesFromPose(engine);
        if (t >= 1.0) {
            engine.state.cog_layer = engine.timed.cogTarget;
            engine.timed = {};
        }
        return pulses;
    }

    if (engine.timed.kind == TimedAnimationKind::PoseRamp) {
        for (int leg : engine.timed.movingLegs) {
            next.feet[leg] = apk_model::swingTrajectory(
                    engine.timed.start.feet[leg],
                    engine.timed.target.feet[leg],
                    t,
                    engine.timed.lift);
        }
        if (engine.timed.layerBlend > 0.0) {
            apk_model::Pose layerTarget = originalAnimationPose(engine, t, 0);
            engine.layers[1] = lerpPose(engine.layers[1], layerTarget, engine.timed.layerBlend);
        }
        engine.state.body = next;
    } else {
        next.body = lerpPose(engine.timed.start.body, engine.timed.target.body, t);
        for (int leg = 0; leg < 6; ++leg) {
            next.feet[leg] = lerpVec(engine.timed.start.feet[leg], engine.timed.target.feet[leg], t);
        }
        engine.state.body = next;
    }

    std::array<int, 18> pulses = toPulsesFromPose(engine);
    if (t >= 1.0) {
        engine.state.body = engine.timed.target;
        engine.timed = {};
    }
    return pulses;
}

std::array<int, 18> stepSetPose(Engine& engine, apk_model::Pose target, double dtMs)
{
    normalizePoseVectors(target);
    scalePoseComponents(target, engine.config.femur_scale);
    apk_model::Pose delta = subtractPose(target, engine.layers[3]);
    scalePoseInPlace(delta, std::max(0.0, dtMs) / 1000.0);
    engine.setVelocity = apk_model::addPose(engine.setVelocity, delta);
    scalePoseInPlace(engine.setVelocity, 0.92);
    engine.layers[3] = apk_model::addPose(engine.layers[3], engine.setVelocity);
    clampPose(engine.layers[3], engine.config.femur_scale);
    return toPulsesFromPose(engine);
}

// Continuous angular sweep set-pose (the original's p3.a.G, worker case 2).
// dive/setrotate hold the stick and the body orbits at a speed set by the stick
// magnitude, advancing an angle each step and writing the rotating pose into
// layer 3 directly (no settling integrator). dR/dS are the original's
// aVar.R()/S() (= the primary stick pair). z5 picks the pose form: true = dive
// (y-translation + z-yaw), false = setrotate/flex (circular x/y translation).
std::array<int, 18> stepSetSweep(Engine& engine, double dR, double dS, bool z5, double dtMs)
{
    // Smooth the stick toward its target (the original's worker lerps the target
    // pose by 0.05 each step before handing it to G), so joystick moves ease in
    // instead of snapping.
    engine.sweepDR += 0.05 * (dR - engine.sweepDR);
    engine.sweepDS += 0.05 * (dS - engine.sweepDS);
    dR = engine.sweepDR;
    dS = engine.sweepDS;
    double mag = std::sqrt((dR * dR) + (dS * dS));
    double rate = std::min(1.0, std::max(-1.0, (dR + dS) * 8.0)) * 360.0 * mag;
    engine.sweepAngle += (std::max(0.0, dtMs) / 1000.0) * rate;
    while (engine.sweepAngle >= 360.0) engine.sweepAngle -= 360.0;
    while (engine.sweepAngle < 0.0) engine.sweepAngle += 360.0;
    double a = (engine.sweepAngle * M_PI) / 180.0;
    double sa = std::sin(a);
    double ca = std::cos(a);
    apk_model::Pose p = {};
    if (z5) {
        p.xyz = {dR * sa, dS * sa, 0.0};
        p.uvw = {0.0, dR * ca, -dS * ca};
    } else {
        p.xyz = {-dS * sa, dS * ca, 0.0};
        p.uvw = {0.0, dR * ca, -dR * sa};
    }
    normalizePoseVectors(p);
    // rotation scale (the original's j.f7126e uses min(R,S), so xyz is uniform
    // 60, not the static set-pose's 60/100/100).
    double sxyz = 60.0 * engine.config.femur_scale;
    double suvw = 18.0 * engine.config.femur_scale;
    p.xyz.x *= sxyz; p.xyz.y *= sxyz; p.xyz.z *= sxyz;
    p.uvw.x *= suvw; p.uvw.y *= suvw; p.uvw.z *= suvw;
    engine.layers[3] = p;
    return toPulsesFromPose(engine);
}

double levelLayerMagnitude(const Engine& engine)
{
    return poseMagnitude(engine.layers[3].xyz) + (poseMagnitude(engine.layers[3].uvw) * 4.0);
}

double layerMagnitude(const apk_model::Pose& pose)
{
    return poseMagnitude(pose.xyz) + (poseMagnitude(pose.uvw) * 4.0);
}

double beginWalkLayerFade(Engine& engine)
{
    apk_model::Pose combined = {};
    for (const auto& layer : engine.layers) {
        combined = apk_model::addPose(combined, layer);
    }
    // p3.a.p() folds ALL layers (incl. the quad CoG layer) into the fade.
    combined = apk_model::addPose(combined, engine.state.cog_layer);
    engine.state.cog_layer = {};
    engine.layers = {};
    engine.layers[0] = combined;
    engine.state.animation_layer = combined;
    engine.walkLayerFadeBody = engine.state.body;
    engine.walkLayerFadeActive = true;
    return layerMagnitude(combined);
}

std::array<int, 18> stepWalkLayerFade(Engine& engine, double amount)
{
    if (!engine.walkLayerFadeActive) {
        return toPulsesFromPose(engine);
    }
    engine.state.body = engine.walkLayerFadeBody;
    double magnitude = layerMagnitude(engine.layers[0]);
    if (magnitude > 0.0 && amount < magnitude) {
        scalePoseInPlace(engine.layers[0], (magnitude - std::max(0.0, amount)) / magnitude);
    }
    return toPulsesFromPose(engine);
}

std::array<int, 18> finishWalkLayerFade(Engine& engine)
{
    if (engine.walkLayerFadeActive) {
        engine.state.body = engine.walkLayerFadeBody;
    }
    engine.layers[0] = {};
    engine.state.animation_layer = {};
    engine.walkLayerFadeActive = false;
    return toPulsesFromPose(engine);
}

std::array<int, 18> applyLevelPose(Engine& engine, double x, double y)
{
    engine.layers[3].uvw.y = std::min(30.0, std::max(-30.0, x * 20.0));
    engine.layers[3].uvw.z = std::min(30.0, std::max(-30.0, y * 20.0));
    return toPulsesFromPose(engine);
}

std::array<int, 18> decayLevelPose(Engine& engine, double factor)
{
    scalePoseInPlace(engine.layers[3], factor);
    if (levelLayerMagnitude(engine) <= 0.1) {
        engine.layers[3] = {};
    }
    return toPulsesFromPose(engine);
}

std::array<int, 18> raiseCalibrationFeet(Engine& engine, double deltaZ)
{
    for (int leg : apk_model::ApkLegOrder) {
        engine.state.body.feet[leg].z += deltaZ;
    }
    return toPulsesFromPose(engine);
}

void lowerCalibrationUntouched(Engine& engine, const std::array<bool, 6>& contacted, double deltaZ)
{
    for (int leg : apk_model::ApkLegOrder) {
        if (!contacted[leg]) {
            engine.state.body.feet[leg].z += deltaZ;
        }
    }
}

std::array<bool, 6> jbooleanArrayToLegFlags(JNIEnv* env, jbooleanArray raw)
{
    std::array<bool, 6> flags = {};
    if (raw == nullptr) {
        return flags;
    }
    jsize size = std::min<jsize>(6, env->GetArrayLength(raw));
    std::array<jboolean, 6> values = {};
    if (size > 0) {
        env->GetBooleanArrayRegion(raw, 0, size, values.data());
    }
    for (jsize i = 0; i < size; ++i) {
        flags[static_cast<size_t>(i)] = values[static_cast<size_t>(i)] == JNI_TRUE;
    }
    return flags;
}

std::vector<int> jintArrayToLegs(JNIEnv* env, jintArray raw)
{
    std::vector<int> legs;
    if (raw == nullptr) {
        for (int leg : apk_model::ApkLegOrder) {
            legs.push_back(leg);
        }
        return legs;
    }
    jsize size = env->GetArrayLength(raw);
    std::vector<jint> values(static_cast<size_t>(size));
    if (size > 0) {
        env->GetIntArrayRegion(raw, 0, size, values.data());
    }
    for (jint value : values) {
        if (value >= 0 && value < 6) {
            legs.push_back(static_cast<int>(value));
        }
    }
    return legs;
}

std::string traceJson(const Engine& engine)
{
    const auto& step = engine.lastStep;
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{";
    out << "\"input\":{"
        << "\"java_gait\":" << engine.lastJavaGait
        << ",\"apk_gait\":" << engine.lastApkGait
        << ",\"animation\":" << engine.lastAnimation
        << ",\"dt_ms\":" << engine.lastDtMs
        << ",\"forward\":" << step.command.forward
        << ",\"strafe\":" << step.command.left
        << ",\"turn\":" << step.command.turn
        << "},";
    out << "\"phase\":" << step.phase
        << ",\"command_magnitude\":" << step.command_magnitude
        << ",\"swing_fraction\":" << step.swing_fraction
        << ",\"cycle_seconds\":" << step.cycle_seconds
        << ",\"body_ik_ok\":" << (step.body_ik_ok ? "true" : "false")
        << ",\"ik_ok\":" << (step.ik_ok ? "true" : "false")
        << ",";
    out << "\"velocity_per_second\":";
    appendVec(out, step.velocity_per_second);
    out << ",\"delta_motion\":";
    appendVec(out, step.delta_motion);
    out << ",\"body_delta\":";
    appendPose(out, step.body_delta);
    out << ",\"body_pose\":";
    appendPose(out, engine.state.body.body);
    out << ",\"animation_target\":";
    appendPose(out, step.animation_target);
    out << ",\"animation_layer\":";
    appendPose(out, step.animation_layer);
    out << ",\"swing_progress\":[";
    for (int leg = 0; leg < 6; ++leg) {
        if (leg > 0) out << ",";
        out << step.swing_progress[leg];
    }
    out << "],\"feet\":";
    appendFeet(out, engine.state.body.feet);
    out << ",\"angles_deg\":";
    appendAngles(out, step.angles_deg);
    out << ",\"pulses_by_pin\":[";
    for (size_t i = 0; i < engine.lastPulses.size(); ++i) {
        if (i > 0) out << ",";
        out << engine.lastPulses[i];
    }
    out << "],\"legs\":[";
    for (int leg = 0; leg < 6; ++leg) {
        if (leg > 0) out << ",";
        out << "{\"leg\":" << leg << ",\"swing\":{";
        const auto& swing = step.swings[leg];
        out << "\"used\":" << (swing.used ? "true" : "false")
            << ",\"clear\":" << (swing.clear ? "true" : "false")
            << ",\"progress\":" << swing.progress
            << ",\"lift\":" << swing.lift
            << ",\"from\":";
        appendVec(out, swing.from);
        out << ",\"to\":";
        appendVec(out, swing.to);
        out << ",\"result\":";
        appendVec(out, swing.result);
        out << ",\"committed\":";
        appendVec(out, swing.committed);
        out << "},\"servos\":[";
        for (int joint = 0; joint < 3; ++joint) {
            if (joint > 0) out << ",";
            int pin = apk_model::ServoPinByApkLeg[leg][joint];
            double angle = step.angles_deg[leg][joint];
            out << "{\"joint\":" << joint
                << ",\"pin\":" << pin
                << ",\"angle_deg\":" << angle
                << ",\"corrected_deg\":" << correctedAngleForTrace(angle, leg, joint)
                << ",\"pulse\":" << engine.lastPulses[pin]
                << "}";
        }
        out << "]}";
    }
    out << "]}";
    return out.str();
}

std::string compactTraceJson(const Engine& engine)
{
    const auto& step = engine.lastStep;
    std::ostringstream out;
    out << std::setprecision(17);
    std::ostringstream cmd;
    cmd << std::scientific << std::setprecision(3)
        << "[" << step.command.forward
        << ", " << step.command.left
        << ", " << step.command.turn << "]";
    out << "{";
    out << "\"phase\":" << step.phase
        << ",\"allow\":" << (engine.lastAllowNewAnchors ? "true" : "false")
        << ",\"gait\":" << engine.lastApkGait
        << ",\"style\":" << engine.lastAnimation
        << ",\"dt\":" << engine.lastDtMs
        << ",\"cmd\":\"" << cmd.str() << "\"";
    out << ",\"body\":["
        << engine.state.body.body.xyz.x << ","
        << engine.state.body.body.xyz.y << ","
        << engine.state.body.body.xyz.z << ","
        << engine.state.body.body.uvw.x << ","
        << engine.state.body.body.uvw.y << ","
        << engine.state.body.body.uvw.z << "]";
    out << ",\"layer\":["
        << engine.state.animation_layer.xyz.x << ","
        << engine.state.animation_layer.xyz.y << ","
        << engine.state.animation_layer.xyz.z << ","
        << engine.state.animation_layer.uvw.x << ","
        << engine.state.animation_layer.uvw.y << ","
        << engine.state.animation_layer.uvw.z << "]";
    out << ",\"anchors\":[";
    for (int leg = 0; leg < 6; ++leg) {
        if (leg > 0) out << ",";
        out << (engine.state.anchor_active[leg] ? "true" : "false");
    }
    out << "]}";
    return out.str();
}

} // namespace

extern "C" JNIEXPORT jlong JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeCreate(JNIEnv*, jclass)
{
    return reinterpret_cast<jlong>(new Engine());
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeDestroy(JNIEnv*, jclass, jlong handle)
{
    delete fromHandle(handle);
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeReset(JNIEnv*, jclass, jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine != nullptr) {
        apk_model::initializeWalkState(engine->config, engine->state);
        engine->layers = {};
        engine->timed = {};
        engine->setVelocity = {};
        toPulsesFromPose(*engine);
    }
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeHasActiveWalkAnchors(
        JNIEnv*, jclass, jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return JNI_FALSE;
    }
    for (bool active : engine->state.anchor_active) {
        if (active) return JNI_TRUE;
    }
    return JNI_FALSE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBeginWalkSession(
        JNIEnv*, jclass, jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return;
    }
    // z0.e keeps phase and anchors in each worker. A new worker starts at
    // phase zero from the robot pose left by the previous worker/home ramp.
    engine->state.phase = 0.0;
    engine->state.anchors = {};
    engine->state.anchor_active = {};
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeReseatFeetFromForwardKinematics(
        JNIEnv*, jclass, jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return;
    }
    // Original z0.a.a(null) -> z0.j.a(): on leg re-enable, recompute EVERY foot
    // from the current servo angles. Active legs round-trip to their existing
    // position; PARKED legs (whose angles were frozen at the tuck while the
    // body drifted through a walk) snap to their real body-framed location, so
    // a following shape-ramp starts from where the legs physically are.
    apk_model::forwardKinematics(engine->config, engine->lastAngles,
                                 engine->state.animation_layer, engine->state.body);
}

extern "C" JNIEXPORT jdouble JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBeginWalkLayerFade(
        JNIEnv*, jclass, jlong handle)
{
    auto* engine = fromHandle(handle);
    return engine == nullptr ? 0.0 : beginWalkLayerFade(*engine);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeStepWalkLayerFade(
        JNIEnv* env, jclass, jlong handle, jdouble amount)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    engine->lastPulses = stepWalkLayerFade(*engine, amount);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeFinishWalkLayerFade(
        JNIEnv* env, jclass, jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    engine->lastPulses = finishWalkLayerFade(*engine);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeEnterConstructorPose(
        JNIEnv* env,
        jclass,
        jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    std::array<std::array<double, 3>, 6> angles = {};
    for (int leg = 0; leg < 6; ++leg) {
        angles[leg] = {0.0, 90.0, 120.0};
    }
    engine->state.initialized = true;
    engine->state.body = {};
    engine->state.animation_layer = {};
    engine->layers = {};
    engine->timed = {};
    engine->setVelocity = {};
    engine->state.phase = 0.0;
    engine->state.anchors = {};
    engine->state.anchor_active = {};
    apk_model::forwardKinematics(engine->config, angles, engine->state.animation_layer, engine->state.body);
    engine->lastAngles = angles;
    engine->lastPulses = toPulses(angles);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativePoseRampToNeutral(
        JNIEnv* env,
        jclass,
        jlong handle,
        jintArray rawLegs,
        jdouble threshold,
        jdouble lift,
        jdouble layerBlend,
        jdouble durationMs,
        jdouble stepMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    std::vector<int> requested = jintArrayToLegs(env, rawLegs);
    apk_model::BodyState start = engine->state.body;
    apk_model::BodyState target = start;
    std::vector<int> moving;
    for (int leg : requested) {
        apk_model::Vec3 neutral = apk_model::neutralFootForBody(engine->config, start, {}, leg);
        target.feet[leg] = neutral;
        double dx = neutral.x - start.feet[leg].x;
        double dy = neutral.y - start.feet[leg].y;
        if (threshold < 0.0 || ((dx * dx) + (dy * dy)) > (threshold * threshold)) {
            moving.push_back(leg);
        }
    }

    if (moving.empty()) {
        return env->NewIntArray(0);
    }

    double duration = std::max(1.0, static_cast<double>(durationMs));
    double step = std::max(1.0, static_cast<double>(stepMs));
    std::vector<jint> frames;
    for (int frame = 0; ; ++frame) {
        double elapsed = std::min(duration, static_cast<double>(frame) * step);
        double t = std::min(1.0, elapsed / duration);
        apk_model::BodyState next = start;
        for (int leg : moving) {
            next.feet[leg] = apk_model::swingTrajectory(start.feet[leg], target.feet[leg], t, lift);
        }
        if (layerBlend > 0.0) {
            apk_model::Pose layerTarget = originalAnimationPose(*engine, t, 0);
            engine->layers[1] = lerpPose(engine->layers[1], layerTarget, layerBlend);
        }
        engine->state.body = next;
        appendFrame(frames, toPulsesFromPose(*engine));
        if (elapsed >= duration) break;
    }
    engine->state.body = target;
    return toFlatFrameArray(env, frames);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBodyZRamp(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble bodyZ,
        jdouble durationMs,
        jdouble stepMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    apk_model::BodyState start = engine->state.body;
    apk_model::BodyState target = start;
    target.body.xyz.z = bodyZ;
    double duration = std::max(1.0, static_cast<double>(durationMs));
    double step = std::max(1.0, static_cast<double>(stepMs));
    std::vector<jint> frames;
    for (int frame = 0; ; ++frame) {
        double elapsed = std::min(duration, static_cast<double>(frame) * step);
        double t = std::min(1.0, elapsed / duration);
        apk_model::BodyState next;
        next.body = lerpPose(start.body, target.body, t);
        for (int leg = 0; leg < 6; ++leg) {
            next.feet[leg] = lerpVec(start.feet[leg], target.feet[leg], t);
        }
        engine->state.body = next;
        appendFrame(frames, toPulsesFromPose(*engine));
        if (elapsed >= duration) break;
    }
    engine->state.body = target;
    return toFlatFrameArray(env, frames);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeShapeRamp(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble radius,
        jdouble z,
        jdouble cornerAngleDeg,
        jdouble elongation,
        jdouble durationMs,
        jdouble stepMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    apk_model::BodyState start = engine->state.body;
    apk_model::BodyState target = start;
    auto feet = neutralFeet(radius, z, cornerAngleDeg, elongation);
    for (int leg : apk_model::ApkLegOrder) {
        target.feet[leg] = feet[leg];
    }
    double duration = std::max(1.0, static_cast<double>(durationMs));
    double step = std::max(1.0, static_cast<double>(stepMs));
    std::vector<jint> frames;
    for (int frame = 0; ; ++frame) {
        double elapsed = std::min(duration, static_cast<double>(frame) * step);
        double t = std::min(1.0, elapsed / duration);
        apk_model::BodyState next;
        next.body = lerpPose(start.body, target.body, t);
        for (int leg = 0; leg < 6; ++leg) {
            next.feet[leg] = lerpVec(start.feet[leg], target.feet[leg], t);
        }
        engine->state.body = next;
        appendFrame(frames, toPulsesFromPose(*engine));
        if (elapsed >= duration) break;
    }
    engine->state.body = target;
    return toFlatFrameArray(env, frames);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeShapeRampForLegs(
        JNIEnv* env,
        jclass,
        jlong handle,
        jintArray rawLegs,
        jdouble radius,
        jdouble z,
        jdouble cornerAngleDeg,
        jdouble elongation,
        jdouble durationMs,
        jdouble stepMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    std::vector<int> requested = jintArrayToLegs(env, rawLegs);
    apk_model::BodyState start = engine->state.body;
    apk_model::BodyState target = start;
    auto feet = neutralFeet(radius, z, cornerAngleDeg, elongation);
    for (int leg : requested) {
        if (leg < 0 || leg >= 6) continue;
        target.feet[leg] = feet[leg];
    }
    double duration = std::max(1.0, static_cast<double>(durationMs));
    double step = std::max(1.0, static_cast<double>(stepMs));
    std::vector<jint> frames;
    for (int frame = 0; ; ++frame) {
        double elapsed = std::min(duration, static_cast<double>(frame) * step);
        double t = std::min(1.0, elapsed / duration);
        apk_model::BodyState next;
        next.body = lerpPose(start.body, target.body, t);
        for (int leg = 0; leg < 6; ++leg) {
            next.feet[leg] = lerpVec(start.feet[leg], target.feet[leg], t);
        }
        engine->state.body = next;
        appendFrame(frames, toPulsesFromPose(*engine));
        if (elapsed >= duration) break;
    }
    engine->state.body = target;
    return toFlatFrameArray(env, frames);
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBeginPoseRampToNeutral(
        JNIEnv* env,
        jclass,
        jlong handle,
        jintArray rawLegs,
        jdouble threshold,
        jdouble lift,
        jdouble layerBlend,
        jdouble durationMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return JNI_FALSE;
    }
    return beginPoseRamp(*engine,
                         jintArrayToLegs(env, rawLegs),
                         threshold,
                         lift,
                         layerBlend,
                         durationMs) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBeginBodyZRamp(
        JNIEnv*,
        jclass,
        jlong handle,
        jdouble bodyZ,
        jdouble durationMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return JNI_FALSE;
    }
    return beginBodyRamp(*engine, bodyZ, durationMs) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBeginBodyZDeltaRamp(
        JNIEnv*,
        jclass,
        jlong handle,
        jdouble bodyZDelta,
        jdouble durationMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return JNI_FALSE;
    }
    return beginBodyDeltaRamp(*engine, bodyZDelta, durationMs) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBeginShapeRamp(
        JNIEnv*,
        jclass,
        jlong handle,
        jdouble radius,
        jdouble z,
        jdouble cornerAngleDeg,
        jdouble elongation,
        jdouble durationMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return JNI_FALSE;
    }
    return beginShapeRamp(*engine, radius, z, cornerAngleDeg, elongation, durationMs) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBeginShapeRampForLegs(
        JNIEnv* env,
        jclass,
        jlong handle,
        jintArray rawLegs,
        jdouble radius,
        jdouble z,
        jdouble cornerAngleDeg,
        jdouble elongation,
        jdouble durationMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return JNI_FALSE;
    }
    return beginShapeRamp(*engine,
                          radius,
                          z,
                          cornerAngleDeg,
                          elongation,
                          durationMs,
                          jintArrayToLegs(env, rawLegs)) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeBeginCogLeanRamp(
        JNIEnv*,
        jclass,
        jlong handle,
        jint leg,
        jdouble durationMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return JNI_FALSE;
    }
    apk_model::Vec3 delta{};
    if (!apk_model::quadCogLeanDelta(engine->state.body, engine->activeLegs, leg, delta)) {
        return JNI_FALSE;
    }
    engine->timed = {};
    engine->timed.kind = TimedAnimationKind::CogLean;
    engine->timed.cogStart = engine->state.cog_layer;
    engine->timed.cogTarget = engine->state.cog_layer;
    engine->timed.cogTarget.xyz.x = delta.x;
    engine->timed.cogTarget.xyz.y = delta.y;
    engine->timed.durationMs = std::max(1.0, static_cast<double>(durationMs));
    return JNI_TRUE;
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeSampleTimedAnimation(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble elapsedMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    return toPulseArray(env, sampleTimedAnimation(*engine, elapsedMs));
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeConfigureMode(
        JNIEnv*,
        jclass,
        jlong handle,
        jdouble radius,
        jdouble cornerAngleDeg,
        jdouble elongation,
        jdouble legSittingZ,
        jdouble swingLift,
        jdouble walkAnimFactor)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return;
    }
    engine->config.leg_radius = radius;
    engine->config.corner_leg_angle_deg = cornerAngleDeg;
    engine->config.elongation = elongation;
    engine->config.leg_sitting_z = legSittingZ;
    // Per-mode gait params (original j.f7129h / j.f7131j set on mode change).
    engine->config.swing_lift = swingLift;
    engine->config.walk_anim_factor = walkAnimFactor;
    engine->config.neutral_feet = neutralFeet(radius, legSittingZ, cornerAngleDeg, elongation);
    engine->activeLegs = {true, true, true, true, true, true};
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeConfigureModeForLegs(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble radius,
        jdouble cornerAngleDeg,
        jdouble elongation,
        jdouble legSittingZ,
        jdouble swingLift,
        jdouble walkAnimFactor,
        jintArray rawActiveLegs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return;
    }
    std::vector<int> active = jintArrayToLegs(env, rawActiveLegs);
    if (active.empty()) {
        active.assign(apk_model::ApkLegOrder.begin(), apk_model::ApkLegOrder.end());
    }
    engine->config.leg_radius = radius;
    engine->config.corner_leg_angle_deg = cornerAngleDeg;
    engine->config.elongation = elongation;
    engine->config.leg_sitting_z = legSittingZ;
    engine->config.swing_lift = swingLift;
    engine->config.walk_anim_factor = walkAnimFactor;
    engine->config.neutral_feet = neutralFeet(radius, legSittingZ, cornerAngleDeg, elongation, active);
    engine->activeLegs = {false, false, false, false, false, false};
    for (int leg : active) {
        if (leg >= 0 && leg < 6) engine->activeLegs[leg] = true;
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeSeedFromPulses(
        JNIEnv* env,
        jclass,
        jlong handle,
        jintArray pulses,
        jdouble bodyZ)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr || pulses == nullptr) {
        return;
    }
    jsize size = env->GetArrayLength(pulses);
    if (size < 18) {
        return;
    }
    std::array<jint, 18> raw = {};
    env->GetIntArrayRegion(pulses, 0, 18, raw.data());

    apk_model::initializeWalkState(engine->config, engine->state);
    engine->state.body.body.xyz = {0.0, 0.0, bodyZ};
    engine->state.body.body.uvw = {0.0, 0.0, 0.0};
    engine->state.animation_layer = {};
    engine->layers = {};
    engine->timed = {};
    engine->setVelocity = {};
    engine->state.phase = 0.0;
    engine->state.anchors = {};
    engine->state.anchor_active = {};

    for (int index = 0; index < 18; ++index) {
        engine->lastPulses[index] = raw[index];
    }
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeStep(
        JNIEnv* env,
        jclass,
        jlong handle,
        jint javaGait,
        jint animation,
        jdouble forward,
        jdouble strafe,
        jdouble turn,
        jdouble dtMs,
        jboolean allowNewAnchors)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }

    apk_model::WalkCommand command;
    command.forward = forward;
    command.left = strafe;
    command.turn = turn;
    std::array<bool, 6> active = engine->activeLegs;
    int apkGait = apkGaitFromJava(javaGait);
    engine->state.animation_layer = engine->layers[0];
    auto step = apk_model::walkStep(engine->config,
                                    engine->state,
                                    command,
                                    apkGait,
                                    animation,
                                    dtMs,
                                    allowNewAnchors == JNI_TRUE,
                                    active);
    engine->lastJavaGait = javaGait;
    engine->lastApkGait = apkGait;
    engine->lastAnimation = animation;
    engine->lastDtMs = dtMs;
    engine->lastAllowNewAnchors = allowNewAnchors == JNI_TRUE;
    engine->lastStep = step;
    engine->layers[0] = engine->state.animation_layer;
    // Persist only the ACTIVE legs' angles: walkStep's IK skips parked legs, so
    // step.angles_deg holds stale zeros for them — keep their frozen tuck angles.
    for (int leg : apk_model::ApkLegOrder) {
        if (active[leg]) engine->lastAngles[leg] = step.angles_deg[leg];
    }
    engine->lastPulses = toPulses(step);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeStepSetPose(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble x,
        jdouble y,
        jdouble z,
        jdouble u,
        jdouble v,
        jdouble w,
        jdouble dtMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    apk_model::Pose target = {{x, y, z}, {u, v, w}};
    engine->lastPulses = stepSetPose(*engine, target, dtMs);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeClearSetPose(
        JNIEnv* env,
        jclass,
        jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    engine->layers[3] = {};
    engine->setVelocity = {};
    engine->sweepAngle = 0.0;
    engine->sweepDR = 0.0;
    engine->sweepDS = 0.0;
    engine->lastPulses = toPulsesFromPose(*engine);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeStepSetSweep(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble dR,
        jdouble dS,
        jboolean z5,
        jdouble dtMs)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    engine->lastPulses = stepSetSweep(*engine, dR, dS, z5 == JNI_TRUE, dtMs);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeApplyLevelPose(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble x,
        jdouble y)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    engine->lastPulses = applyLevelPose(*engine, x, y);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeDecayLevelPose(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble factor)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    engine->lastPulses = decayLevelPose(*engine, factor);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jdouble JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeLevelPoseMagnitude(
        JNIEnv*,
        jclass,
        jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return 0.0;
    }
    return levelLayerMagnitude(*engine);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeCalibrationCurrentPulses(
        JNIEnv* env,
        jclass,
        jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    engine->lastPulses = toPulsesFromPose(*engine);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeCalibrationRaiseAll(
        JNIEnv* env,
        jclass,
        jlong handle,
        jdouble deltaZ)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewIntArray(0);
    }
    engine->lastPulses = raiseCalibrationFeet(*engine, deltaZ);
    return toPulseArray(env, engine->lastPulses);
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeCalibrationLowerUntouched(
        JNIEnv* env,
        jclass,
        jlong handle,
        jbooleanArray contacted,
        jdouble deltaZ)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return;
    }
    lowerCalibrationUntouched(*engine, jbooleanArrayToLegFlags(env, contacted), deltaZ);
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeLastTraceJson(
        JNIEnv* env,
        jclass,
        jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewStringUTF("{}");
    }
    std::string json = traceJson(*engine);
    return env->NewStringUTF(json.c_str());
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeLastCompactTraceJson(
        JNIEnv* env,
        jclass,
        jlong handle)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) {
        return env->NewStringUTF("{}");
    }
    std::string json = compactTraceJson(*engine);
    return env->NewStringUTF(json.c_str());
}

extern "C" JNIEXPORT jint JNICALL
Java_com_makeyourpet_chicaserver_hardware_NativeTty_nativeOpenRaw(
        JNIEnv* env,
        jclass,
        jstring pathString)
{
    const char* path = env->GetStringUTFChars(pathString, nullptr);
    if (path == nullptr) return -EINVAL;
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    env->ReleaseStringUTFChars(pathString, path);
    if (fd < 0) return -errno;

    termios tty {};
    if (tcgetattr(fd, &tty) == 0) {
        cfmakeraw(&tty);
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
        tty.c_cflag &= ~CRTSCTS;
#endif
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        tcsetattr(fd, TCSANOW, &tty);
        tcflush(fd, TCIOFLUSH);
    }
#if defined(TIOCMBIS) && defined(TIOCM_DTR) && defined(TIOCM_RTS)
    int modemBits = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd, TIOCMBIS, &modemBits);
#endif
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
    return fd;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_makeyourpet_chicaserver_hardware_NativeTty_nativeRead(
        JNIEnv* env,
        jclass,
        jint fd,
        jbyteArray buffer,
        jint offset,
        jint length)
{
    if (fd < 0 || offset < 0 || length < 0) return -EINVAL;
    std::vector<jbyte> temp(static_cast<size_t>(length));
    ssize_t count = read(fd, temp.data(), static_cast<size_t>(length));
    if (count < 0) return -errno;
    if (count > 0) {
        env->SetByteArrayRegion(buffer, offset, static_cast<jsize>(count), temp.data());
    }
    return static_cast<jint>(count);
}

extern "C" JNIEXPORT jint JNICALL
Java_com_makeyourpet_chicaserver_hardware_NativeTty_nativeWrite(
        JNIEnv* env,
        jclass,
        jint fd,
        jbyteArray buffer,
        jint offset,
        jint length)
{
    if (fd < 0 || offset < 0 || length < 0) return -EINVAL;
    std::vector<jbyte> temp(static_cast<size_t>(length));
    env->GetByteArrayRegion(buffer, offset, static_cast<jsize>(length), temp.data());
    ssize_t count = write(fd, temp.data(), static_cast<size_t>(length));
    if (count < 0) return -errno;
    return static_cast<jint>(count);
}

extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_hardware_NativeTty_nativeClose(
        JNIEnv*,
        jclass,
        jint fd)
{
    if (fd >= 0) close(fd);
}

// Push the full servo config (per-servo calibration, attach angles, pin map)
// parsed from chica.config into the angle->pulse conversion. Global, mirroring
// the original z0.h static config. cal36 = [leg][joint][lo,hi] flattened,
// coxa6 = per-leg coxa attach, pin18 = [leg][joint] board pins flattened.
extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeSetServoConfig(
        JNIEnv* env,
        jclass,
        jintArray cal36,
        jdoubleArray coxa6,
        jdouble femurAttach,
        jdouble tibiaAttach,
        jintArray pin18)
{
    if (env->GetArrayLength(cal36) < 36 || env->GetArrayLength(coxa6) < 6
            || env->GetArrayLength(pin18) < 18) {
        return;
    }
    jint* cal = env->GetIntArrayElements(cal36, nullptr);
    jdouble* coxa = env->GetDoubleArrayElements(coxa6, nullptr);
    jint* pin = env->GetIntArrayElements(pin18, nullptr);

    ChicaServoConfig cfg{};
    for (int leg = 0; leg < 6; ++leg) {
        cfg.coxaAttach[leg] = coxa[leg];
        for (int joint = 0; joint < 3; ++joint) {
            int idx = (leg * 3 + joint);
            cfg.calibration[leg][joint][0] = cal[idx * 2];
            cfg.calibration[leg][joint][1] = cal[idx * 2 + 1];
            cfg.pin[leg][joint] = pin[idx];
        }
    }
    cfg.femurAttach = femurAttach;
    cfg.tibiaAttach = tibiaAttach;
    chica_apk_set_servo_config(cfg);

    env->ReleaseIntArrayElements(cal36, cal, JNI_ABORT);
    env->ReleaseDoubleArrayElements(coxa6, coxa, JNI_ABORT);
    env->ReleaseIntArrayElements(pin18, pin, JNI_ABORT);
}

// Push the body geometry (leg segment lengths, body dimensions, connection/
// sitting Z) from chica.config into the engine, mirroring a2.n.f populating the
// z0.h geometry fields. Recomputes the coxa mount points and neutral feet so the
// IK uses the configured frame. Defaults match makeDefaultConfig so an
// unconfigured engine is unchanged.
extern "C" JNIEXPORT void JNICALL
Java_com_makeyourpet_chicaserver_gait_ChicaGaitEngine_nativeConfigureGeometry(
        JNIEnv*,
        jclass,
        jlong handle,
        jdouble coxaLen,
        jdouble femurLen,
        jdouble tibiaLen,
        jdouble l1ToR1,
        jdouble l1ToL3,
        jdouble l2ToR2,
        jdouble legConnectionZ,
        jdouble legSittingZ)
{
    auto* engine = fromHandle(handle);
    if (engine == nullptr) return;
    apk_model::RobotConfig& c = engine->config;
    c.coxa_len = coxaLen;
    c.femur_len = femurLen;
    c.tibia_len = tibiaLen;
    c.femur_scale = ((femurLen + 80.0) / 2.0) / 80.0;  // z0.j.d
    c.l1_to_r1 = l1ToR1;
    c.l1_to_l3 = l1ToL3;
    c.l2_to_r2 = l2ToR2;
    c.leg_connection_z = legConnectionZ;
    c.leg_sitting_z = legSittingZ;

    double hfw = l1ToR1 / 2.0;
    double hl = l1ToL3 / 2.0;
    double hmw = l2ToR2 / 2.0;
    double z = legConnectionZ;
    c.mounts[0] = {-hfw, hl, z};
    c.mounts[1] = {-hmw, 0.0, z};
    c.mounts[2] = {-hfw, -hl, z};
    c.mounts[3] = {hfw, hl, z};
    c.mounts[4] = {hmw, 0.0, z};
    c.mounts[5] = {hfw, -hl, z};

    c.neutral_feet = neutralFeet(c.leg_radius, c.leg_sitting_z,
                                 c.corner_leg_angle_deg, c.elongation);
}
