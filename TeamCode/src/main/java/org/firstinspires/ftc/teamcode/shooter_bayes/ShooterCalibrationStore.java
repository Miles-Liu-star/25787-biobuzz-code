package org.firstinspires.ftc.teamcode.shooter_bayes;

import android.content.Context;
import android.content.SharedPreferences;

/**
 * 用 Robot Controller 的 SharedPreferences 保存飞轮标定结果。
 * 换 Hub 或清除应用数据后需要重新跑 04。
 */
public final class ShooterCalibrationStore {
    public static final String FLYWHEEL_SCALE = "flywheel_scale";
    public static final String FLYWHEEL_MODEL = "flywheel_model";
    public static final String FLYWHEEL_FULL_POWER_MODEL = "flywheel_model_full_power_v1";
    public static final String FLYWHEEL_PID = "flywheel_pid";
    public static final String FLYWHEEL_PID_HISTORY = "flywheel_pid_full_power_history_v2";

    private final SharedPreferences prefs;

    public ShooterCalibrationStore(Context context) {
        prefs = context.getSharedPreferences("shooter_calibration_v1", Context.MODE_PRIVATE);
    }

    public double[] read(String key, double... fallback) {
        return readRecords(key, 1, fallback.length, fallback.length, fallback);
    }

    public double[] readHistory(String key, int stride, int rows) {
        return readRecords(key, 0, stride * rows, stride, new double[0]);
    }

    private double[] readRecords(String key, int minimum, int maximum, int stride, double[] fallback) {
        try {
            String raw = prefs.getString(key, "");
            if (raw.isEmpty() || stride <= 0) return fallback;
            String[] fields = raw.split(",");
            if (fields.length < minimum || fields.length > maximum || fields.length % stride != 0) {
                return fallback;
            }
            double[] values = new double[fields.length];
            for (int i = 0; i < fields.length; i++) values[i] = Double.parseDouble(fields[i]);
            return Maths.finite(values) ? values : fallback;
        } catch (IllegalArgumentException | ClassCastException ignored) {
            return fallback;
        }
    }

    public boolean save(String key, double... values) {
        if (!Maths.finite(values)) return false;
        StringBuilder result = new StringBuilder();
        for (double value : values) {
            if (result.length() > 0) result.append(',');
            result.append(value);
        }
        SharedPreferences.Editor edit = prefs.edit().putString(key, result.toString());
        // 满功率曲线发生变化时，旧的 PID 搜索结果不再可信。
        if (FLYWHEEL_FULL_POWER_MODEL.equals(key) && fullPowerModelChanged(values)) {
            edit.remove(FLYWHEEL_PID_HISTORY);
        }
        // RPM 比例变化超过 1% 时，历史代价也不可比。
        if (FLYWHEEL_SCALE.equals(key) && values.length == 1
                && Math.abs(values[0] - read(FLYWHEEL_SCALE, 0)[0]) > 0.01 * values[0]) {
            edit.remove(FLYWHEEL_PID_HISTORY);
        }
        return edit.commit();
    }

    public boolean has(String key) {
        return prefs.contains(key);
    }

    private boolean fullPowerModelChanged(double[] model) {
        if (model.length != 4) return true;
        double[] previous = read(FLYWHEEL_FULL_POWER_MODEL, 0, 0, 0, 0);
        return Math.abs(model[0] - previous[0]) > 0.015 * model[0]
                || Math.abs(model[3] - previous[3]) > 0.015 * model[3]
                || Math.abs(model[1] - previous[1]) > 0.02;
    }
}
