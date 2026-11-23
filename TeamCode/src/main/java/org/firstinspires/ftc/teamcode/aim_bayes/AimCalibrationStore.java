package org.firstinspires.ftc.teamcode.aim_bayes;

import android.content.Context;
import android.content.SharedPreferences;

/** Saves Aim PID gains and Bayesian search history on the Robot Controller. */
public final class AimCalibrationStore {
    // v1 was calibrated against cubed turn commands and the old turn sign.
    private static final String PID = "aim_pid_v2";
    private static final String PID_HISTORY = "aim_pid_history_v2";

    private final SharedPreferences preferences;

    public AimCalibrationStore(Context context) {
        preferences = context.getSharedPreferences("aim_calibration_v1", Context.MODE_PRIVATE);
    }

    /** Return calibrated P/I/D, or null when this controller has no usable calibration. */
    public double[] readGains() {
        try {
            String[] fields = preferences.getString(PID, "").split(",", -1);
            if (fields.length == 3) {
                double[] gains = {Double.parseDouble(fields[0]), Double.parseDouble(fields[1]),
                        Double.parseDouble(fields[2])};
                if (validGains(gains)) return gains;
            }
        } catch (IllegalArgumentException | ClassCastException ignored) {
            // Treat an unreadable calibration as missing.
        }
        return null;
    }

    public double[] readHistory(int stride, int rows) {
        try {
            String raw = preferences.getString(PID_HISTORY, "");
            if (raw.isEmpty()) return new double[0];
            String[] fields = raw.split(",");
            if (stride <= 0 || fields.length > stride * rows || fields.length % stride != 0) {
                return new double[0];
            }
            double[] values = new double[fields.length];
            for (int i = 0; i < fields.length; i++) values[i] = Double.parseDouble(fields[i]);
            for (double value : values) if (!Double.isFinite(value)) return new double[0];
            return values;
        } catch (IllegalArgumentException | ClassCastException ignored) {
            return new double[0];
        }
    }

    public boolean saveGains(double[] gains) {
        if (!validGains(gains)) return false;
        return preferences.edit().putString(PID, encode(gains)).commit();
    }

    public boolean saveHistory(double[] history) {
        if (history == null) return false;
        for (double value : history) if (!Double.isFinite(value)) return false;
        return preferences.edit().putString(PID_HISTORY, encode(history)).commit();
    }

    private static boolean validGains(double[] gains) {
        if (gains == null || gains.length != 3) return false;
        for (double gain : gains) if (!Double.isFinite(gain) || gain < 0) return false;
        return gains[0] > 0.0 || gains[1] > 0.0;
    }

    private static String encode(double[] values) {
        StringBuilder result = new StringBuilder();
        for (double value : values) {
            if (result.length() > 0) result.append(',');
            result.append(value);
        }
        return result.toString();
    }
}
