import com.makeyourpet.chicaserver.gait.ChicaGaitEngine;
import java.util.Arrays;

/** Exercise the shipped Java/JNI boundary, with B/G/p/N expectations from the APK.
 * This checks deterministic state handling; it does not emulate Android scheduling.
 */
public final class SetWorkerJniProbe {
    private static int checks;
    private static double[] array(ChicaGaitEngine engine, String key) {
        String trace = engine.lastCompactTraceJson();
        String start = "\"" + key + "\":[";
        int at = trace.indexOf(start) + start.length();
        return Arrays.stream(trace.substring(at, trace.indexOf(']', at)).split(","))
                .mapToDouble(Double::parseDouble).toArray();
    }
    private static void equal(double[] expected, double[] actual, String label) {
        if (expected.length != actual.length) throw new AssertionError(label + " length");
        for (int i = 0; i < expected.length; i++) {
            if (!Double.isFinite(actual[i]) || Math.abs(expected[i] - actual[i]) > 1e-10)
                throw new AssertionError(label + " axis=" + i + " expected=" + expected[i] + " actual=" + actual[i]);
        }
        checks++;
    }
    private static double norm(double[] v, int offset) {
        return Math.sqrt(v[offset]*v[offset] + v[offset+1]*v[offset+1] + v[offset+2]*v[offset+2]);
    }
    private static double magnitude(double[] v) { return norm(v, 0) + 4 * norm(v, 3); }
    private static void normalize(double[] v) {
        for (int offset : new int[]{0, 3}) {
            double n = norm(v, offset);
            if (n > 1) for (int i = offset; i < offset+3; i++) v[i] *= 1/n;
        }
    }
    private static double[] staticStep(double[] target, double[] layer, double[] state, double dt) {
        target = target.clone(); normalize(target);
        double[] limits = {60, 100, 100, 28, 18, 18};
        for (int i = 0; i < 6; i++) {
            state[i] = (state[i] + (-layer[i] + target[i]*limits[i])*dt/1000) * .92;
            layer[i] = Math.max(-limits[i], Math.min(limits[i], layer[i] + state[i]));
        }
        return layer;
    }
    private static double[] sweepStep(double[] target, double[] state, int mode, double dt) {
        double x=target[0], y=target[1];
        state[6] += dt/1000 * Math.max(-1, Math.min(1, (x+y)*8)) * 360 * Math.sqrt(x*x+y*y);
        if (state[6] >= 360) state[6] -= 360;
        else if (state[6] < 0) state[6] += 360;
        double a=state[6]*Math.PI/180;
        double[] layer = mode == 1
                ? new double[]{x*Math.sin(a), y*Math.sin(a), 0, 0, x*Math.cos(a), -y*Math.cos(a)}
                : new double[]{-y*Math.sin(a), y*Math.cos(a), 0, 0, x*Math.cos(a), -x*Math.sin(a)};
        normalize(layer);
        for (int i=0; i<6; i++) layer[i] *= i<3 ? 60 : 18;
        return layer;
    }
    private static void workers() {
        try (ChicaGaitEngine engine = new ChicaGaitEngine()) {
            double[][] actual = new double[4][7], expected = new double[4][7];
            double[] layer = new double[6];
            double[] times = {0, 1, 7, 10, 13, 45, 4000};
            for (int f=0; f<6000; f++) {
                int worker = f % 4, mode = worker < 2 ? 0 : worker-1;
                double[] target = {.2*Math.sin(f*.07), .3*Math.cos(f*.11), .05, -.02, .1, -.05};
                double dt = times[f % times.length];
                layer = mode == 0 ? staticStep(target, layer, expected[worker], dt)
                                  : sweepStep(target, expected[worker], mode, dt);
                double[][] before = Arrays.stream(actual).map(double[]::clone).toArray(double[][]::new);
                if (engine.stepSetWorker(actual[worker], target, mode, dt).length != 18)
                    throw new AssertionError("worker PWM width");
                equal(layer, array(engine, "layer"), "B/G shared layer");
                equal(expected[worker], actual[worker], "worker-private velocity/angle");
                for (int i=0; i<4; i++) if (i != worker) equal(before[i], actual[i], "other worker untouched");
            }
        }
    }
    private static void fadesAndKeep() {
        try (ChicaGaitEngine engine = new ChicaGaitEngine()) {
            double[] worker = new double[7], target = {.15,.2,0,0,.1,-.1};
            for (int i=0; i<80; i++) engine.stepSetWorker(worker, target, 0, 10);
            double[] kept = array(engine, "layer");
            engine.keepSetPose();
            equal(kept, array(engine, "layer"), "N keep combined pose");
            double[] next = new double[7];
            double[] zero = new double[6];
            engine.stepSetWorker(next, zero, 0, 0);
            equal(kept, array(engine, "layer"), "N clears layer3 without clearing saved layer0");
            double[] first = engine.beginLayerFadeContext();
            equal(kept, Arrays.copyOf(first, 6), "p folds saved pose");
            double[] snapshot = array(engine, "body");
            double[] second = engine.beginLayerFadeContext();
            double[] other = second.clone();
            double amount = .75, factor = (magnitude(kept)-amount)/magnitude(kept);
            double[] faded = kept.clone();
            for (int i=0; i<6; i++) faded[i] *= factor;
            engine.stepLayerFadeContext(first, amount);
            equal(faded, Arrays.copyOf(first, 6), "p private magnitude decrement");
            equal(other, second, "other p context untouched");
            engine.beginBodyZRamp(55, 100);
            engine.sampleTimedAnimation(100);
            engine.stepSetWorker(new double[7], new double[]{.1,.15,0,0,0,0}, 1, 10);
            double[] concurrent = array(engine, "layer");
            for (int i=0; i<6; i++) concurrent[i] -= faded[i];
            engine.finishLayerFadeContext(first);
            equal(concurrent, array(engine, "layer"), "p finish preserves a new worker layer3");
            equal(snapshot, array(engine, "body"), "p restores private body snapshot");
            equal(new double[6], Arrays.copyOf(first, 6), "p terminal layer0 zero");
        }
    }
    private static void calibration() {
        try (ChicaGaitEngine moved = new ChicaGaitEngine(); ChicaGaitEngine fresh = new ChicaGaitEngine()) {
            for (int i=0; i<150; i++) moved.step(5, 0, .2, .1, .05, 10);
            moved.finishLayerFadeContext(moved.beginLayerFadeContext());
            moved.beginCalibration(); fresh.beginCalibration();
            int[] actual = moved.calibrationCurrentPulses(), expected = fresh.calibrationCurrentPulses();
            if (!Arrays.equals(actual, expected)) throw new AssertionError("calibration retains gait drift");
            equal(new double[6], array(moved, "body"), "calibration body starts at origin");
            for (int i=0; i<100; i++) {
                // A touch may disappear on the next poll: no latched contact.
                boolean[] touches = {i%2==0, i%3==0, false, true, false, false};
                moved.calibrationLowerUntouched(touches, -.20000000298023224);
                fresh.calibrationLowerUntouched(touches, -.20000000298023224);
                if (!Arrays.equals(moved.calibrationCurrentPulses(), fresh.calibrationCurrentPulses()))
                    throw new AssertionError("calibration lowering");
                checks++;
            }
        }
    }
    public static void main(String[] args) {
        workers(); fadesAndKeep(); calibration();
        System.out.println("Java/JNI deterministic checks=" + checks + " passed; Android thread timing not covered");
    }
}
