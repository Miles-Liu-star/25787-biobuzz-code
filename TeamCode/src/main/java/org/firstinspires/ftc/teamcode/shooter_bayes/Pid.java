package org.firstinspires.ftc.teamcode.shooter_bayes;

/**
 * 带前馈和抗积分饱和的 PID。
 * 微分项使用实测速度（measuredRate），因此目标阶跃不会造成微分冲击。
 */
final class Pid {
    private final double p, i, d;
    private double integral;

    Pid(double[] gains) {
        p = gains[0];
        i = gains[1];
        d = gains[2];
    }

    double update(double error, double measuredRate, double ff, double dt,
                  double minOutput, double maxOutput) {
        return update(error, measuredRate, ff, dt, minOutput, maxOutput, true);
    }

    double update(double error, double measuredRate, double ff, double dt,
                  double minOutput, double maxOutput, boolean integrate) {
        double base = p * error - d * measuredRate + ff;
        double trial = integrate ? Maths.clip(integral + i * error * dt, -0.20, 0.20) : integral;
        double requested = base + trial;
        if ((requested <= maxOutput || trial <= integral)
                && (requested >= minOutput || trial >= integral)) {
            integral = trial;
        }
        return Maths.clip(base + integral, minOutput, maxOutput);
    }

    void reset() {
        integral = 0;
    }
}
