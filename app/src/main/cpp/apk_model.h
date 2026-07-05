#pragma once

#include <array>
#include <vector>

namespace apk_model {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Pose {
    Vec3 xyz;
    Vec3 uvw;
};

struct RobotConfig {
    double coxa_len = 43.0;
    double femur_len = 80.0;
    double tibia_len = 134.0;
    double l1_to_r1 = 126.0;
    double l1_to_l3 = 167.0;
    double l2_to_r2 = 163.0;
    double leg_connection_z = -10.0;
    double leg_sitting_z = -40.0;
    double leg_radius = 220.0;
    double corner_leg_angle_deg = 55.0;
    double elongation = 1.15;
    std::array<Vec3, 6> mounts = {};
    std::array<Vec3, 6> neutral_feet = {};
    // Femur-derived scale on the body pose/motion limits, mirroring the
    // original z0.j.d(((FEMUR_LEN+80)/2)/80). 1.0 for the stock 80mm femur.
    double femur_scale = 1.0;
    // Per-mode gait params (original z0.j statics set on every mode change):
    // j.f7129h = swing lift (MODE_* 5th value: STANDARD 40, QUADRUPED 35),
    // j.f7131j = walk animation factor (7th value: STANDARD 1.0, RACE 0.0).
    // The walk animation layer blends at walk_anim_factor / 12 per frame.
    double swing_lift = 40.0;
    double walk_anim_factor = 1.0;
};

struct BodyState {
    Pose body;
    std::array<Vec3, 6> feet = {};
};

struct WalkCommand {
    double forward = 0.0;
    double left = 0.0;
    double turn = 0.0;
};

struct WalkState {
    bool initialized = false;
    double phase = 0.0;
    BodyState body;
    Pose animation_layer;
    // Quad CoG layer (original layer[2], the second `if (i5 == 20)` block in
    // p3.a.g(): during each stance gap the body leans toward the point where
    // the line body->3-support-leg-centroid crosses the prev/next-foot
    // diagonal, unloading the next swing leg so it can physically lift).
    Pose cog_layer;
    std::array<Vec3, 6> anchors = {};
    std::array<bool, 6> anchor_active = {};
};

struct WalkStepResult {
    bool ik_ok = false;
    bool body_ik_ok = false;
    double phase = 0.0;
    double command_magnitude = 0.0;
    double swing_fraction = 0.0;
    double cycle_seconds = 0.0;
    WalkCommand command;
    Vec3 velocity_per_second;
    Vec3 delta_motion;
    Pose body_delta;
    Pose animation_target;
    Pose animation_layer;
    Pose cog_layer;
    std::array<double, 6> swing_progress = {};
    std::array<std::array<double, 3>, 6> angles_deg = {};
    struct SwingTrace {
        bool used = false;
        bool clear = false;
        int leg = -1;
        double progress = 0.0;
        double lift = 0.0;
        Vec3 from;
        Vec3 to;
        Vec3 result;
        Vec3 committed;
    };
    std::array<SwingTrace, 6> swings = {};
};

struct PhaseWindow {
    double start = 0.0;
    double end = 0.0;
};

constexpr std::array<int, 6> ApkLegOrder = {0, 3, 1, 4, 2, 5};
constexpr std::array<std::array<int, 3>, 6> ServoPinByApkLeg = {{
    {{15, 16, 17}},
    {{9, 10, 11}},
    {{3, 4, 5}},
    {{12, 13, 14}},
    {{6, 7, 8}},
    {{0, 1, 2}},
}};

RobotConfig makeDefaultConfig();
std::array<Vec3, 6> makeNeutralFeet(double radius,
                                     double z,
                                     double corner_angle_deg,
                                     double elongation,
                                     const std::vector<int>& active);
void rotateDegrees(Vec3& value, double x_deg, double y_deg, double z_deg);
Pose addPose(const Pose& a, const Pose& b);
Pose bodyPoseWithLayer(const BodyState& state, const Pose& layer);
double wrapPhase(double phase);
double gaitSwingFraction(int gait_id);
std::array<PhaseWindow, 6> gaitPhaseTable(int gait_id);
double swingProgress(double phase, double start, double end, double margin);
Vec3 swingTrajectory(const Vec3& from, const Vec3& to, double progress, double lift);
Vec3 neutralFootForBody(const RobotConfig& config,
                        const BodyState& state,
                        const Pose& body_delta,
                        int leg);
void initializeWalkState(const RobotConfig& config, WalkState& state);
bool quadCogLeanDelta(const BodyState& state,
                      const std::array<bool, 6>& active,
                      int leg,
                      Vec3& out);
WalkStepResult walkStep(const RobotConfig& config,
                        WalkState& state,
                        WalkCommand command,
                        int gait_id,
                        int animation_id,
                        double dt_ms_scaled,
                        bool allow_new_anchors,
                        const std::array<bool, 6>& active,
                        double phase_override = -1.0,
                        double cycle_override = -1.0);
void forwardKinematics(const RobotConfig& config,
                       const std::array<std::array<double, 3>, 6>& angles_deg,
                       const Pose& layer,
                       BodyState& state);
bool inverseKinematics(const RobotConfig& config,
                       const BodyState& state,
                       const Pose& layer,
                       const std::array<bool, 6>& active,
                       std::array<std::array<double, 3>, 6>& angles_deg);

} // namespace apk_model
