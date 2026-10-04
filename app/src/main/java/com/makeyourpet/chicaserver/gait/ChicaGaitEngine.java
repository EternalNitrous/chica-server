package com.makeyourpet.chicaserver.gait;

public final class ChicaGaitEngine implements AutoCloseable {
    static {
        System.loadLibrary("chica_gait");
    }

    private long nativeHandle;

    public ChicaGaitEngine() {
        nativeHandle = nativeCreate();
    }

    public synchronized int[] step(int javaGait,
                                   int animation,
                                   double forward,
                                   double strafe,
                                   double turn,
                                   double dtMs) {
        return step(javaGait, animation, forward, strafe, turn, dtMs, true);
    }

    public synchronized int[] step(int javaGait,
                                   int animation,
                                   double forward,
                                   double strafe,
                                   double turn,
                                   double dtMs,
                                   boolean allowNewAnchors) {
        ensureOpen();
        return nativeStep(nativeHandle, javaGait, animation, forward, strafe, turn, dtMs, allowNewAnchors);
    }

    public synchronized void reset() {
        ensureOpen();
        nativeReset(nativeHandle);
    }

    public synchronized boolean hasActiveWalkAnchors() {
        ensureOpen();
        return nativeHasActiveWalkAnchors(nativeHandle);
    }

    public synchronized void beginWalkSession() {
        ensureOpen();
        nativeBeginWalkSession(nativeHandle);
    }

    /**
     * Re-seat every foot from the current per-leg servo angles via forward
     * kinematics (original z0.a.a(null) -> z0.j.a() on leg re-enable). Parked
     * legs stay frozen at their tuck angles while the body drifts during a
     * walk, so their stored world position goes stale; FK snaps them back
     * under the current body. Call before the quad-exit shape ramp.
     */
    public synchronized void reseatFeetFromForwardKinematics() {
        ensureOpen();
        nativeReseatFeetFromForwardKinematics(nativeHandle);
    }

    public synchronized double beginWalkLayerFade() {
        ensureOpen();
        return nativeBeginWalkLayerFade(nativeHandle);
    }

    public synchronized int[] stepWalkLayerFade(double amount) {
        ensureOpen();
        return nativeStepWalkLayerFade(nativeHandle, amount);
    }

    public synchronized int[] finishWalkLayerFade() {
        ensureOpen();
        return nativeFinishWalkLayerFade(nativeHandle);
    }

    public synchronized int[] enterConstructorPose() {
        ensureOpen();
        return nativeEnterConstructorPose(nativeHandle);
    }

    public synchronized int[][] poseRampToNeutral(int[] legs,
                                                  double threshold,
                                                  double lift,
                                                  double layerBlend,
                                                  double durationMs,
                                                  double stepMs) {
        ensureOpen();
        return splitFrames(nativePoseRampToNeutral(nativeHandle, legs, threshold, lift, layerBlend, durationMs, stepMs));
    }

    public synchronized int[][] bodyZRamp(double bodyZ, double durationMs, double stepMs) {
        ensureOpen();
        return splitFrames(nativeBodyZRamp(nativeHandle, bodyZ, durationMs, stepMs));
    }

    public synchronized int[][] shapeRamp(double radius,
                                          double z,
                                          double cornerAngleDeg,
                                          double elongation,
                                          double durationMs,
                                          double stepMs) {
        ensureOpen();
        return splitFrames(nativeShapeRamp(nativeHandle, radius, z, cornerAngleDeg, elongation, durationMs, stepMs));
    }

    public synchronized int[][] shapeRampForLegs(int[] legs,
                                                 double radius,
                                                 double z,
                                                 double cornerAngleDeg,
                                                 double elongation,
                                                 double durationMs,
                                                 double stepMs) {
        ensureOpen();
        return splitFrames(nativeShapeRampForLegs(nativeHandle, legs, radius, z, cornerAngleDeg, elongation, durationMs, stepMs));
    }

    public synchronized boolean beginPoseRampToNeutral(int[] legs,
                                                       double threshold,
                                                       double lift,
                                                       double layerBlend,
                                                       double durationMs) {
        ensureOpen();
        return nativeBeginPoseRampToNeutral(nativeHandle, legs, threshold, lift, layerBlend, durationMs);
    }

    public synchronized boolean beginBodyZRamp(double bodyZ, double durationMs) {
        ensureOpen();
        return nativeBeginBodyZRamp(nativeHandle, bodyZ, durationMs);
    }

    public synchronized boolean beginBodyZDeltaRamp(double bodyZDelta, double durationMs) {
        ensureOpen();
        return nativeBeginBodyZDeltaRamp(nativeHandle, bodyZDelta, durationMs);
    }

    public synchronized boolean beginShapeRamp(double radius,
                                               double z,
                                               double cornerAngleDeg,
                                               double elongation,
                                               double durationMs) {
        ensureOpen();
        return nativeBeginShapeRamp(nativeHandle, radius, z, cornerAngleDeg, elongation, durationMs);
    }

    public synchronized boolean beginShapeRampForLegs(int[] legs,
                                                      double radius,
                                                      double z,
                                                      double cornerAngleDeg,
                                                      double elongation,
                                                      double durationMs) {
        ensureOpen();
        return nativeBeginShapeRampForLegs(nativeHandle, legs, radius, z, cornerAngleDeg, elongation, durationMs);
    }

    public synchronized int[] sampleTimedAnimation(double elapsedMs) {
        ensureOpen();
        return nativeSampleTimedAnimation(nativeHandle, elapsedMs);
    }

    public synchronized void configureMode(double radius,
                                           double cornerAngleDeg,
                                           double elongation,
                                           double legSittingZ,
                                           double swingLift,
                                           double walkAnimFactor) {
        ensureOpen();
        nativeConfigureMode(nativeHandle, radius, cornerAngleDeg, elongation, legSittingZ,
                swingLift, walkAnimFactor);
    }

    public synchronized void configureModeForLegs(double radius,
                                                  double cornerAngleDeg,
                                                  double elongation,
                                                  double legSittingZ,
                                                  double swingLift,
                                                  double walkAnimFactor,
                                                  int[] activeLegs) {
        ensureOpen();
        nativeConfigureModeForLegs(nativeHandle, radius, cornerAngleDeg, elongation, legSittingZ,
                swingLift, walkAnimFactor, activeLegs);
    }

    public synchronized boolean beginCogLeanRamp(int leg, double durationMs) {
        ensureOpen();
        return nativeBeginCogLeanRamp(nativeHandle, leg, durationMs);
    }

    public synchronized void seedFromPulses(int[] pulses, double bodyZ) {
        ensureOpen();
        nativeSeedFromPulses(nativeHandle, pulses, bodyZ);
    }

    public synchronized int[] stepSetPose(double x,
                                          double y,
                                          double z,
                                          double u,
                                          double v,
                                          double w,
                                          double dtMs) {
        ensureOpen();
        return nativeStepSetPose(nativeHandle, x, y, z, u, v, w, dtMs);
    }

    public synchronized int[] stepSetSweep(double dR, double dS, boolean z5, double dtMs) {
        ensureOpen();
        return nativeStepSetSweep(nativeHandle, dR, dS, z5, dtMs);
    }

    public synchronized int[] clearSetPose() {
        ensureOpen();
        return nativeClearSetPose(nativeHandle);
    }

    public synchronized int[] stepSetWorker(double[] workerState, double[] target, int sweepMode, double dtMs) {
        ensureOpen();
        return nativeStepSetWorker(nativeHandle, workerState, target, sweepMode, dtMs);
    }

    public synchronized double[] beginLayerFadeContext() {
        ensureOpen();
        return nativeBeginLayerFadeContext(nativeHandle);
    }

    public synchronized int[] stepLayerFadeContext(double[] context, double amount) {
        ensureOpen();
        return nativeLayerFadeContext(nativeHandle, context, amount, false);
    }

    public synchronized int[] finishLayerFadeContext(double[] context) {
        ensureOpen();
        return nativeLayerFadeContext(nativeHandle, context, 0.0d, true);
    }

    public synchronized void keepSetPose() {
        ensureOpen();
        nativeKeepSetPose(nativeHandle);
    }

    /**
     * Push the per-servo calibration, mechanical attach angles, and servo pin
     * map parsed from chica.config into the native angle->pulse conversion.
     * cal36 is [leg][joint][lo,hi] flattened (18*2), coxa6 the per-leg coxa
     * attach angles, pin18 the [leg][joint] board pins flattened (18).
     */
    public synchronized void setServoConfig(int[] cal36, double[] coxa6,
                                            double femurAttach, double tibiaAttach,
                                            int[] pin18) {
        nativeSetServoConfig(cal36, coxa6, femurAttach, tibiaAttach, pin18);
    }

    /**
     * Push the body geometry from chica.config (leg segment lengths, body
     * dimensions, connection/sitting Z) into the engine, mirroring the original
     * z0.h geometry fields populated by a2.n.f.
     */
    public synchronized void configureGeometry(double coxaLen, double femurLen, double tibiaLen,
                                               double l1ToR1, double l1ToL3, double l2ToR2,
                                               double legConnectionZ, double legSittingZ) {
        ensureOpen();
        nativeConfigureGeometry(nativeHandle, coxaLen, femurLen, tibiaLen,
                l1ToR1, l1ToL3, l2ToR2, legConnectionZ, legSittingZ);
    }

    public synchronized int[] applyLevelPose(double x, double y) {
        ensureOpen();
        return nativeApplyLevelPose(nativeHandle, x, y);
    }

    public synchronized int[] decayLevelPose(double factor) {
        ensureOpen();
        return nativeDecayLevelPose(nativeHandle, factor);
    }

    public synchronized double levelPoseMagnitude() {
        ensureOpen();
        return nativeLevelPoseMagnitude(nativeHandle);
    }

    public synchronized void beginCalibration() {
        ensureOpen();
        nativeBeginCalibration(nativeHandle);
    }

    public synchronized int[] calibrationCurrentPulses() {
        ensureOpen();
        return nativeCalibrationCurrentPulses(nativeHandle);
    }

    public synchronized int[] calibrationRaiseAll(double deltaZ) {
        ensureOpen();
        return nativeCalibrationRaiseAll(nativeHandle, deltaZ);
    }

    public synchronized void calibrationLowerUntouched(boolean[] contacted, double deltaZ) {
        ensureOpen();
        nativeCalibrationLowerUntouched(nativeHandle, contacted, deltaZ);
    }

    public synchronized String lastTraceJson() {
        ensureOpen();
        return nativeLastTraceJson(nativeHandle);
    }

    public synchronized String lastCompactTraceJson() {
        ensureOpen();
        return nativeLastCompactTraceJson(nativeHandle);
    }

    @Override
    public synchronized void close() {
        if (nativeHandle != 0L) {
            nativeDestroy(nativeHandle);
            nativeHandle = 0L;
        }
    }

    private void ensureOpen() {
        if (nativeHandle == 0L) throw new IllegalStateException("gait engine is closed");
    }

    private static int[][] splitFrames(int[] flat) {
        if (flat == null || flat.length == 0) return new int[0][0];
        int count = flat.length / 18;
        int[][] frames = new int[count][18];
        for (int frame = 0; frame < count; frame++) {
            System.arraycopy(flat, frame * 18, frames[frame], 0, 18);
        }
        return frames;
    }

    private static native long nativeCreate();
    private static native void nativeDestroy(long handle);
    private static native void nativeReset(long handle);
    private static native boolean nativeHasActiveWalkAnchors(long handle);
    private static native void nativeBeginWalkSession(long handle);
    private static native void nativeReseatFeetFromForwardKinematics(long handle);
    private static native double nativeBeginWalkLayerFade(long handle);
    private static native int[] nativeStepWalkLayerFade(long handle, double amount);
    private static native int[] nativeFinishWalkLayerFade(long handle);
    private static native int[] nativeEnterConstructorPose(long handle);
    private static native int[] nativePoseRampToNeutral(long handle,
                                                        int[] legs,
                                                        double threshold,
                                                        double lift,
                                                        double layerBlend,
                                                        double durationMs,
                                                        double stepMs);
    private static native int[] nativeBodyZRamp(long handle,
                                                double bodyZ,
                                                double durationMs,
                                                double stepMs);
    private static native int[] nativeShapeRamp(long handle,
                                                double radius,
                                                double z,
                                                double cornerAngleDeg,
                                                double elongation,
                                                double durationMs,
                                                double stepMs);
    private static native int[] nativeShapeRampForLegs(long handle,
                                                       int[] legs,
                                                       double radius,
                                                       double z,
                                                       double cornerAngleDeg,
                                                       double elongation,
                                                       double durationMs,
                                                       double stepMs);
    private static native boolean nativeBeginPoseRampToNeutral(long handle,
                                                               int[] legs,
                                                               double threshold,
                                                               double lift,
                                                               double layerBlend,
                                                               double durationMs);
    private static native boolean nativeBeginBodyZRamp(long handle,
                                                       double bodyZ,
                                                       double durationMs);
    private static native boolean nativeBeginBodyZDeltaRamp(long handle,
                                                            double bodyZDelta,
                                                            double durationMs);
    private static native boolean nativeBeginShapeRamp(long handle,
                                                       double radius,
                                                       double z,
                                                       double cornerAngleDeg,
                                                       double elongation,
                                                       double durationMs);
    private static native boolean nativeBeginShapeRampForLegs(long handle,
                                                              int[] legs,
                                                              double radius,
                                                              double z,
                                                              double cornerAngleDeg,
                                                              double elongation,
                                                              double durationMs);
    private static native boolean nativeBeginCogLeanRamp(long handle, int leg, double durationMs);
    private static native int[] nativeSampleTimedAnimation(long handle,
                                                           double elapsedMs);
    private static native void nativeConfigureMode(long handle,
                                                   double radius,
                                                   double cornerAngleDeg,
                                                   double elongation,
                                                   double legSittingZ,
                                                   double swingLift,
                                                   double walkAnimFactor);
    private static native void nativeConfigureModeForLegs(long handle,
                                                          double radius,
                                                          double cornerAngleDeg,
                                                          double elongation,
                                                          double legSittingZ,
                                                          double swingLift,
                                                          double walkAnimFactor,
                                                          int[] activeLegs);
    private static native void nativeSeedFromPulses(long handle, int[] pulses, double bodyZ);
    private static native int[] nativeStepSetPose(long handle,
                                                  double x,
                                                  double y,
                                                  double z,
                                                  double u,
                                                  double v,
                                                  double w,
                                                  double dtMs);
    private static native int[] nativeStepSetSweep(long handle,
                                                   double dR,
                                                   double dS,
                                                   boolean z5,
                                                   double dtMs);
    private static native int[] nativeClearSetPose(long handle);
    private static native int[] nativeStepSetWorker(long handle, double[] workerState, double[] target,
                                                   int sweepMode, double dtMs);
    private static native double[] nativeBeginLayerFadeContext(long handle);
    private static native int[] nativeLayerFadeContext(long handle, double[] context, double amount, boolean finish);
    private static native void nativeKeepSetPose(long handle);
    private static native void nativeSetServoConfig(int[] cal36, double[] coxa6,
                                                    double femurAttach, double tibiaAttach,
                                                    int[] pin18);
    private static native void nativeConfigureGeometry(long handle,
                                                       double coxaLen, double femurLen, double tibiaLen,
                                                       double l1ToR1, double l1ToL3, double l2ToR2,
                                                       double legConnectionZ, double legSittingZ);
    private static native int[] nativeApplyLevelPose(long handle,
                                                     double x,
                                                     double y);
    private static native int[] nativeDecayLevelPose(long handle,
                                                     double factor);
    private static native double nativeLevelPoseMagnitude(long handle);
    private static native void nativeBeginCalibration(long handle);
    private static native int[] nativeCalibrationCurrentPulses(long handle);
    private static native int[] nativeCalibrationRaiseAll(long handle,
                                                          double deltaZ);
    private static native void nativeCalibrationLowerUntouched(long handle,
                                                               boolean[] contacted,
                                                               double deltaZ);
    private static native int[] nativeStep(long handle,
                                           int javaGait,
                                           int animation,
                                           double forward,
                                           double strafe,
                                           double turn,
                                           double dtMs,
                                           boolean allowNewAnchors);
    private static native String nativeLastTraceJson(long handle);
    private static native String nativeLastCompactTraceJson(long handle);
}
