package org.firstinspires.ftc.teamcode.shooter_bayes;

import java.util.Arrays;

/** 飞轮标定用到的小工具，避免依赖外部库。 */
final class Maths {
    private Maths() {}

    static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    static double median(double[] values, int count) {
        if (count == 0) return Double.NaN;
        double[] ordered = Arrays.copyOf(values, count);
        Arrays.sort(ordered);
        return (ordered[(count - 1) / 2] + ordered[count / 2]) / 2;
    }

    static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }

    static double time() {
        return System.nanoTime() * 1e-9;
    }
}
