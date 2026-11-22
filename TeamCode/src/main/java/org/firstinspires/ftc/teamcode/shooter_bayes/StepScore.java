package org.firstinspires.ftc.teamcode.shooter_bayes;

/** 对一次速度阶跃响应打分：误差面积、超调、稳态偏差、抖动和输出变化量。 */
final class StepScore {
    private final double duration;
    private double errorArea, overshoot, tailTime, tailError, tailSquare, effort, lastPower;
    private double lastSample, withinToleranceSince = Double.NaN;
    private double quietTime, quietError, quietSquare, quietAbsolute;
    private boolean endedEarly;

    StepScore(double duration) {
        this.duration = duration;
    }

    // 每个响应按阶跃幅度归一化，不同幅度的测试可以用同一套分数比较。
    void sample(double time, double dt, double progress, double power, boolean withinTolerance) {
        double error = 1 - progress;
        errorArea += Math.abs(error) * dt;
        overshoot = Math.max(overshoot, -error);
        effort += Math.abs(power - lastPower);
        lastPower = power;
        if (time > duration * 0.65) {
            tailTime += dt;
            tailError += error * dt;
            tailSquare += error * error * dt;
        }
        lastSample = time;
        if (withinTolerance) {
            if (!Double.isFinite(withinToleranceSince)) withinToleranceSince = time;
            quietTime += dt;
            quietError += error * dt;
            quietSquare += error * error * dt;
            quietAbsolute += Math.abs(error) * dt;
        } else {
            withinToleranceSince = Double.NaN;
            quietTime = quietError = quietSquare = quietAbsolute = 0;
        }
    }

    boolean finishIfSettled(double hold) {
        endedEarly = Double.isFinite(settlingTime(hold));
        return endedEarly;
    }

    double settlingTime(double hold) {
        return lastSample - withinToleranceSince >= hold
                ? withinToleranceSince + hold : Double.POSITIVE_INFINITY;
    }

    double scoreWithSettlingTime(double hold) {
        return 1.65 * settlingTime(hold) / duration + 0.50 * score();
    }

    double overshoot() {
        return overshoot;
    }

    double score() {
        if (endedEarly) {
            // 用已经稳定的窗口估算未测完的剩余部分。
            double area = errorArea + Math.max(0, duration - lastSample) * quietAbsolute / quietTime;
            return score(area, quietError / quietTime, quietSquare / quietTime);
        }
        if (tailTime < duration * 0.12) return Double.POSITIVE_INFINITY;
        return score(errorArea, tailError / tailTime, tailSquare / tailTime);
    }

    private double score(double area, double mean, double meanSquare) {
        double deviation = Math.sqrt(Math.max(0, meanSquare - mean * mean));
        return area / duration + 1.8 * Math.abs(mean) + 2 * deviation + 1.5 * overshoot + 0.07 * effort;
    }
}
