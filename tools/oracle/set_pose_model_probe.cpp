// Compare the shipped helper with independently written APK B/G formulae.
// References: p3.a.B/G/F/q and z0.j.d. IK/PWM conversion is shared here;
// original-APK captures provide the separate evidence for those operations.
#include "../../app/src/main/cpp/chica_gait_jni.cpp"
#include <cstdio>
#include <cstdlib>

using Values = std::array<double, 6>;
static constexpr double pi = 3.141592653589793;

static Values values(const apk_model::Pose& p) {
    return {p.xyz.x, p.xyz.y, p.xyz.z, p.uvw.x, p.uvw.y, p.uvw.z};
}
static apk_model::Pose pose(const Values& v) {
    return {{v[0], v[1], v[2]}, {v[3], v[4], v[5]}};
}
static void normalize(Values& v) {
    for (int k : {0, 3}) {
        double length = std::sqrt(v[k] * v[k] + v[k + 1] * v[k + 1] + v[k + 2] * v[k + 2]);
        if (length > 1.0) for (int i = k; i < k + 3; ++i) v[i] *= 1.0 / length;
    }
}
static Values limits(double scale) {
    return {60 * scale, 100 * scale, 100 * scale, 28 * scale, 18 * scale, 18 * scale};
}
static Values staticStep(Values target, Values& layer, Values& velocity, double scale, double dt) {
    normalize(target);
    Values lim = limits(scale);
    for (int i = 0; i < 6; ++i) {
        target[i] *= lim[i];
        velocity[i] = (velocity[i] + (-layer[i] + target[i]) * (dt / 1000.0)) * 0.92;
        layer[i] = std::min(lim[i], std::max(-lim[i], layer[i] + velocity[i]));
    }
    return layer;
}
static Values sweepStep(double x, double y, bool dive, double scale, double dt, double& angle) {
    angle += (dt / 1000.0) * std::min(1.0, std::max(-1.0, (x + y) * 8.0))
             * 360.0 * std::sqrt(y * y + x * x);
    if (angle >= 360.0) angle -= 360.0;
    else if (angle < 0.0) angle += 360.0;
    double a = pi * angle / 180.0;
    Values target = dive ? Values{x * std::sin(a), y * std::sin(a), 0, 0,
                                  x * std::cos(a), -y * std::cos(a)}
                         : Values{-y * std::sin(a), y * std::cos(a), 0, 0,
                                  x * std::cos(a), -x * std::sin(a)};
    normalize(target);
    Values scaling = {60 * scale, 60 * scale, 100 * scale, 28 * scale, 18 * scale, 18 * scale};
    Values lim = limits(scale);
    for (int i = 0; i < 6; ++i) target[i] = std::min(lim[i], std::max(-lim[i], target[i] * scaling[i]));
    return target;
}
static void init(Engine& e, double scale) {
    e.config.femur_scale = scale;
    e.state.body.body.xyz.z = 40;
    e.state.body.feet = e.config.neutral_feet;
    toPulsesFromPose(e);
}

int main() {
    double maximum = 0;
    int frames = 0, mismatchedPulses = 0;
    auto compare = [&](Engine& actual, Engine& reference, const Values& expected,
                       const std::array<int, 18>& pulses) {
        Values observed = values(actual.layers[3]);
        for (int i = 0; i < 6; ++i) {
            double delta = std::abs(expected[i] - observed[i]);
            if (!std::isfinite(delta) || delta > 1e-10) {
                std::fprintf(stderr, "pose mismatch frame=%d axis=%d expected=%.17g actual=%.17g\n",
                             frames, i, expected[i], observed[i]);
                std::exit(1);
            }
            maximum = std::max(maximum, delta);
        }
        reference.layers[3] = pose(expected);
        if (pulses != toPulsesFromPose(reference)) ++mismatchedPulses;
        ++frames;
    };
    constexpr std::array<double, 8> times = {0, 1, 7, 10, 11, 13, 20, 45};
    constexpr std::array<Values, 7> targets = {{
        {0, 0, 0, 0, 0, 0}, {-.15, .2, 0, 0, 0, 0}, {0, 0, .15, .1, 0, 0},
        {0, 0, 0, 0, -.1, -.15}, {-.6, .8, 0, 0, .6, .8},
        {1.2, -.9, .7, -.5, .8, -.6}, {-.25, -.2, -.1, .3, .2, .1}
    }};
    for (double scale : {.5, 1., 1.25}) {
        Engine actual, reference; init(actual, scale); init(reference, scale);
        Values layer = {}, velocity = {};
        for (int f = 0; f < 6000; ++f) {
            Values input = targets[(f / 137) % targets.size()];
            double dt = times[f % times.size()];
            auto pulses = stepSetPose(actual, pose(input), dt);
            compare(actual, reference, staticStep(input, layer, velocity, scale, dt), pulses);
        }
        for (bool dive : {false, true}) {
            Engine sweep, oracle; init(sweep, scale); init(oracle, scale);
            double angle = 0;
            for (int f = 0; f < 6000; ++f) {
                // Already-filtered worker inputs; rate remains in the normal
                // domain where the APK performs a single angle wrap.
                Values input = targets[(f / 113) % targets.size()];
                double dt = times[f % times.size()];
                auto pulses = stepSetSweep(sweep, input[0], input[1], dive, dt, false);
                compare(sweep, oracle, sweepStep(input[0], input[1], dive, scale, dt, angle), pulses);
            }
        }
    }
    // p3.a.M leaves legs below its XY threshold unchanged, including at
    // terminal commit and in the following body-height animation.
    Engine home; init(home, 1);
    home.state.body.feet[0].x += 30;
    home.state.body.feet[1].x += 5;
    const auto untouched = home.state.body.feet[1];
    if (!beginPoseRamp(home, {0,1,2,3,4,5}, 10, 15, 0, 100)
            || home.timed.movingLegs != std::vector<int>{0}) return 1;
    for (double time : {0.,50.,100.}) {
        sampleTimedAnimation(home,time);
        const auto& f=home.state.body.feet[1];
        if (f.x != untouched.x || f.y != untouched.y || f.z != untouched.z) {
            std::fprintf(stderr,"M() changed an unselected leg at t=%g\n",time);
            return 1;
        }
    }
    beginBodyRamp(home,0,100); sampleTimedAnimation(home,100);
    if (home.state.body.feet[1].x != untouched.x) return 1;
    std::puts("APK M selected-leg terminal commit: passed (unselected foot remains unchanged)");
    std::printf("APK B/G formula comparison frames=%d pose_tolerance=1e-10 max_pose_delta=%.3g "
                "PWM_mismatched_frames=%d (shared IK/PWM; runtime timing not covered)\n",
                frames, maximum, mismatchedPulses);
    return mismatchedPulses == 0 ? 0 : 1;
}
