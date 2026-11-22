package org.firstinspires.ftc.teamcode.pinpoint;

import java.util.Arrays;

public final class Maths {
    public static final double TURN = 2 * Math.PI;

    private Maths() {}

    public static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    public static double wrap(double angle) {
        double wrapped = angle % TURN;
        if (wrapped >= Math.PI) return wrapped - TURN;
        return wrapped < -Math.PI ? wrapped + TURN : wrapped;
    }

    public static double deadband(double command) {
        double magnitude = Math.abs(command);
        return magnitude < 0.06 ? 0 : Math.copySign((magnitude - 0.06) / 0.94, command);
    }

    public static double median(double[] values, int count) {
        if (count == 0) return Double.NaN;
        double[] ordered = Arrays.copyOf(values, count);
        Arrays.sort(ordered);
        return (ordered[(count - 1) / 2] + ordered[count / 2]) / 2;
    }

    public static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }

    public static double time() {
        return System.nanoTime() * 1e-9;
    }
}
