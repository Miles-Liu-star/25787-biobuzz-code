package org.firstinspires.ftc.teamcode.shooter_bayes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * 自测飞轮 转速和 PID。
 *
 * 流程：
 * 1. 标定“每飞轮转一圈的编码器计数”（RPM 比例）；
 * 2. 在 0.26 ~ 1.00 多个功率点测稳态转速，拟合功率—转速直线；
 * 3. 用贝叶斯搜索整定 P/I/D，并对候选参数做留出复测；
 * 4. 通过后把 PID / 模型 / 比例写入 Robot Controller 的应用数据，供正式 TeleOp 使用。
 *
 * 操作：START 进入自测；A 确认；B 跳过或停止当前动作。
 */
@TeleOp(name = "自测飞轮转速和 PID", group = "飞轮自测")
public final class FlywheelCalibration extends CalibrationOp {
    private static final double PID_HOLD = 0.25;
    private static final double STEP_DURATION = 3.1;
    private static final double REFERENCE_POWER = 0.70;
    private static final double FULL_POWER = FlywheelConfig.FLYWHEEL_MAX_POWER;
    // 你的飞轮惯量较大，功率曲线需要更长的加速和稳定时间。
    private static final double POWER_POINT_TIMEOUT = 8.0;
    private static final double REFERENCE_POINT_TIMEOUT = 6.0;
    private static final double FULL_POWER_TIMEOUT = 10.0;
    private static final double[] POWER_LEVELS =
            {0.26, 0.40, 0.55, REFERENCE_POWER, 0.85, 0.93, FULL_POWER};
    private int trials;
    private BayesianPidSearch completed;
    private double fullPowerSpeed, fullPowerRpm, peakPidPower;
    private String outcome;

    @Override
    String intro() {
        return "清空仓内和进弹通道，移开飞轮周围物品。自测中不会驱动挡弹舵机。";
    }

    @Override
    void calibrate() throws InterruptedException {
        Flywheel wheel = new Flywheel(hardwareMap, store);
        try {
            if (confirm(wheel.ticksPerRev > 0
                    ? "A 重测每转计数，B 沿用已有 RPM 比例。"
                    : "标定实际 RPM？在飞轮上标记起点。A 进入手转测量，B 用 ticks/s。")) {
                calibrateScale(wheel);
                wheel = new Flywheel(hardwareMap, store);
            }
            String prompt = wheel.fullPowerSpeed() > 0
                    ? "测 0.26 到 1.00 的功率曲线。A 重测，B 沿用已保存的满功率曲线。"
                    : "需要实测 1.00 才能开始 PID 标定。确认无球后按 A 测功率曲线。";
            while (confirm(prompt)) {
                boolean fitted = fitModel(wheel);
                if (!opModeIsActive() || motionAborted()) return;
                wheel = new Flywheel(hardwareMap, store);
                if (fitted || wheel.fullPowerSpeed() > 0) break;
                prompt = "满功率曲线未测成。A 重测 0.26 到 1.00，B 结束。";
            }
            if (wheel.fullPowerSpeed() <= 0) {
                telemetry.log().add("缺少 1.00 稳态转速。检查 %s 编码器和电池后重跑 04。",
                        FlywheelConfig.FLYWHEEL_ENCODER_MOTOR);
                return;
            }
            fullPowerSpeed = wheel.fullPowerSpeed();
            fullPowerRpm = wheel.ticksPerRev > 0 ? 60 * fullPowerSpeed / wheel.ticksPerRev : 0;
            double[] speeds = tuningSpeeds(wheel);
            if (!confirm("确认无球。A 开始 20–32 轮 PID 搜索和留出复测，B 跳过。",
                    () -> telemetry.addData("1.00 稳态 / PID 高速", "%s / %s",
                            speedText(fullPowerSpeed, fullPowerRpm),
                            speedText(speeds[2], testedRpm(speeds[2]))))) return;
            Flywheel tested = wheel;
            double[] current = store.read(ShooterCalibrationStore.FLYWHEEL_PID, Flywheel.DEFAULT_GAINS);
            BayesianPidSearch search = new BayesianPidSearch(
                    new double[]{Math.max(1.1, current[0] * 1.5), Math.max(1.5, current[1] * 1.5),
                            Math.max(0.080, current[2] * 1.5)},
                    new double[]{0.40, 0.26, 0.012});
            BayesianPidSearch.Selection best = search.tune(current,
                    (gains, verification, pattern) -> pidTrial(tested, current, gains, speeds, verification, pattern),
                    () -> !opModeIsActive() || motionAborted() || tested.faulted(),
                    store.readHistory(ShooterCalibrationStore.FLYWHEEL_PID_HISTORY,
                            BayesianPidSearch.HISTORY_STRIDE, BayesianPidSearch.HISTORY_ROWS));
            completed = search;
            if (opModeIsActive() && !motionAborted() && !tested.faulted() && search.searchCount() > 0) {
                save(ShooterCalibrationStore.FLYWHEEL_PID_HISTORY, search.history());
            }
            if (best != null && opModeIsActive() && !motionAborted()) {
                outcome = save(ShooterCalibrationStore.FLYWHEEL_PID, best.gains)
                        ? "复测通过，已保存 PID" : "PID 写入失败";
                telemetry.log().add("%s，代价 %.3f，P/I/D %.3f / %.3f / %.3f",
                        outcome, best.score, best.gains[0], best.gains[1], best.gains[2]);
            } else if (opModeIsActive() && !motionAborted()) {
                outcome = tested.faulted() ? "飞轮编码器异常，保留原 PID"
                        : search.searchCount() < BayesianPidSearch.MIN_ROUNDS
                        ? "有效搜索轮数不足，保留原 PID"
                        : "复测未通过，保留原 PID";
                telemetry.log().add("%s", outcome);
            }
        } finally {
            wheel.stop();
        }
    }

    @Override
    void showResults() {
        if (fullPowerSpeed > 0) telemetry.addData("1.00 稳态", speedText(fullPowerSpeed, fullPowerRpm));
        if (completed == null) return;
        telemetry.addData("结果", outcome == null ? "中止" : outcome);
        telemetry.addData("搜索", "历史 %d / 本轮 %d / 最低代价 %.3f",
                completed.historyCount(), completed.searchCount(), completed.bestSearchCost());
        telemetry.addData("PID 实测峰值功率", "%.2f", peakPidPower);
        for (BayesianPidSearch.Review row : completed.reviews()) {
            telemetry.addData(String.format(Locale.US, "P/I/D %.3f / %.3f / %.3f",
                            row.gains[0], row.gains[1], row.gains[2]), "%d/%d 组通过，%s", row.passed,
                    BayesianPidSearch.VERIFICATION_ROUNDS,
                    row.firstFailure == null ? "复测代价 " + row.score : row.firstFailure);
        }
    }

    private static String speedText(double ticks, double rpm) {
        return rpm > 0 ? String.format(Locale.US, "%.0f ticks/s (%.0f RPM)", ticks, rpm)
                : String.format(Locale.US, "%.0f ticks/s", ticks);
    }

    private double testedRpm(double speed) {
        return speed * fullPowerRpm / fullPowerSpeed;
    }

    private void calibrateScale(Flywheel wheel) throws InterruptedException {
        Measurements scales = new Measurements();
        for (int pass = 0; pass < 4 && opModeIsActive(); pass++) {
            int turns = pass % 2 == 0 ? 5 : 3;
            if (!confirm("标记已对齐？A 记下起点。")) break;
            int from = wheel.ticks();
            if (!confirm("沿同一方向用手转飞轮整 " + turns + " 圈，停在标记位置后按 A。")) break;
            double observed = Math.abs((double) (wheel.ticks() - from)) / turns;
            if (observed >= 1 && observed <= 500000) scales.add(observed);
            else telemetry.log().add("编码器变化无效，检查接线后继续。");
            if (scales.size() >= 2 && scales.relativeSpread() <= 0.04) {
                save(ShooterCalibrationStore.FLYWHEEL_SCALE, scales.median());
                telemetry.log().add("每飞轮转 %.1f ticks", scales.median());
                return;
            }
        }
        telemetry.log().add("RPM 比例缺少一致样本，保留之前的比例，有足够编码器数据仍可闭环。");
    }

    private boolean fitModel(Flywheel wheel) throws InterruptedException {
        Measurements.PowerFit fit = new Measurements.PowerFit();
        Measurements referenceSpeeds = new Measurements(), fullSpeeds = new Measurements();
        try {
            for (double power : POWER_LEVELS) {
                if (!active(wheel)) break;
                double speed = measurePlateau(wheel, power, POWER_POINT_TIMEOUT);
                fit.add(speed, power);
                if (power == REFERENCE_POWER) referenceSpeeds.add(speed);
                if (power == FULL_POWER) fullSpeeds.add(speed);
            }
            for (int repeat = 0; repeat < 2 && active(wheel); repeat++) {
                if (fullSpeeds.size() >= 2 && fullSpeeds.relativeSpread() <= 0.04) break;
                // 高功率点前先回到一个较低的占空比，排除电池/温度漂移。
                double reference = measurePlateau(wheel, REFERENCE_POWER, REFERENCE_POINT_TIMEOUT);
                referenceSpeeds.add(reference);
                fit.add(reference, REFERENCE_POWER);
                if (!active(wheel)) break;
                double speed = measurePlateau(wheel, FULL_POWER,
                        repeat == 0 ? FULL_POWER_TIMEOUT : FULL_POWER_TIMEOUT + 2.0);
                fit.add(speed, FULL_POWER);
                fullSpeeds.add(speed);
            }
        } finally {
            wheel.stop();
        }
        if (!active(wheel)) return false;
        double full = fullSpeeds.median();
        double[] coefficients = fit.fit(Math.max(8, full * 0.07));
        if (fullSpeeds.size() < 2 || fullSpeeds.relativeSpread() > 0.05 || !(full >= 40)
                || coefficients == null || coefficients[0] > 0.50) {
            telemetry.log().add("曲线未通过：1.00 稳态样本 %d 组，需至少 2 组一致样本。",
                    fullSpeeds.size());
            return false;
        }
        double reference = referenceSpeeds.size() > 0 ? referenceSpeeds.median()
                : (REFERENCE_POWER - coefficients[0]) / coefficients[1];
        if (!(reference >= 20 && reference < full)) return false;
        double slope = (FlywheelConfig.FLYWHEEL_MAX_POWER - coefficients[0]) * reference / full;
        telemetry.log().add("实测功率 1.00 稳态 %s，共 %d 组。",
                speedText(full, wheel.ticksPerRev > 0 ? 60 * full / wheel.ticksPerRev : 0), fullSpeeds.size());
        return save(ShooterCalibrationStore.FLYWHEEL_FULL_POWER_MODEL,
                reference, coefficients[0], slope, full);
    }

    private double measurePlateau(Flywheel wheel, double power, double timeout) throws InterruptedException {
        Plateau plateau = new Plateau();
        runFor(timeout, (t, dt) -> {
            wheel.sample(dt);
            if (wheel.faulted()) return false;
            // 只有两个电机都保持在目标输出后才开始采样。
            if (Math.abs(wheel.output - power) < 1e-6) plateau.sample(wheel.speed, dt);
            wheel.power(power, dt);
            show("飞轮功率曲线", "设定 %.2f / 实际 %.2f / %s",
                    power, wheel.output, speedText(wheel.speed,
                            wheel.ticksPerRev > 0 ? 60 * wheel.speed / wheel.ticksPerRev : 0));
            return !Double.isFinite(plateau.speed);
        });
        // 高阻尼飞轮在低占空比下可能无法克服静摩擦；稳定为零速是有效的低功率测量结果。
        // 参考功率及以上仍必须取得真实速度，不能用零速数据通过模型标定。
        if (!Double.isFinite(plateau.speed) && power < REFERENCE_POWER
                && !wheel.faulted() && wheel.speed < 20) {
            plateau.speed = 0;
            telemetry.log().add("功率 %.2f 未克服飞轮阻力，按 0 ticks/s 记录；该点不参与线性拟合。", power);
        }
        if (Double.isFinite(plateau.speed) && plateau.speed > 0) {
            telemetry.log().add("功率 %.2f，稳态 %.0f ticks/s", power, plateau.speed);
        }
        return plateau.speed;
    }

    private static final class Plateau {
        private double warmup, window, total, previous = Double.NaN;
        double speed = Double.NaN;

        void sample(double velocity, double dt) {
            warmup += dt;
            if (warmup < 1.50) return;
            window += dt;
            total += velocity * dt;
            if (window < 0.50) return;
            double mean = total / window;
            if (mean >= 20 && Math.abs(mean - previous) <= 0.022 * mean) speed = (mean + previous) / 2;
            previous = mean;
            window = total = 0;
        }
    }

    private double[] tuningSpeeds(Flywheel wheel) {
        double high = wheel.estimatedSpeedAtPower(0.96);
        double low = wheel.fullPowerSpeed() * 0.63, shoot = 0;
        if (wheel.rpmCalibrated()) {
            double idle = FlywheelConfig.FLYWHEEL_IDLE_RPM * wheel.ticksPerRev / 60;
            double requested = FlywheelConfig.FLYWHEEL_SHOOT_RPM * wheel.ticksPerRev / 60;
            if (idle < high * 0.90) low = idle;
            if (requested <= wheel.fullPowerSpeed()) {
                if (requested >= 0.98 * high) high = requested;
                else if (requested > low + 0.06 * wheel.reference()) shoot = requested;
                else telemetry.log().add("射击转速和测试低速过近，保留满功率附近的阶跃。");
            } else {
                telemetry.log().add("射击转速 %.0f RPM 超出实测满功率转速约 %.0f RPM。",
                        FlywheelConfig.FLYWHEEL_SHOOT_RPM, testedRpm(wheel.fullPowerSpeed()));
            }
        }
        return new double[]{low, shoot, high};
    }

    private BayesianPidSearch.Result pidTrial(Flywheel wheel, double[] current, double[] gains, double[] speeds,
                                               boolean verification, int pattern) throws InterruptedException {
        double low = speeds[0] * (verification ? 0.97 + 0.02 * pattern : 1);
        double high = speeds[2] * (verification && pattern == 0 ? 0.985 : 1);
        if (!active(wheel)) return null;
        wheel.setGains(current);
        trials++;
        try {
            prepareSpeed(wheel, low, false);
            if (active(wheel) && Math.abs(wheel.speed - low) > 0.09 * low) prepareSpeed(wheel, low, true);
            if (!active(wheel)) return null;
            if (Math.abs(wheel.speed - low) > 0.12 * low) {
                return BayesianPidSearch.Result.unavailable("准备低速未到位");
            }
            // 射击转速与满功率附近不同时，加测一次射击转速。
            double[] targets = speeds[1] > low + 0.05 * wheel.reference()
                    ? new double[]{speeds[1], low, high, low} : new double[]{high, low};
            return measureSteps(wheel, gains, targets, high, verification);
        } finally {
            wheel.stop();
        }
    }

    private void prepareSpeed(Flywheel wheel, double target, boolean feedforward) throws InterruptedException {
        runFor(feedforward ? 1.8 : 2.7, (t, dt) -> {
            wheel.sample(dt);
            if (wheel.faulted()) return false;
            if (feedforward) wheel.power(wheel.estimatedPowerForSpeed(target), dt);
            else wheel.regulate(target, dt);
            show("飞轮准备低速", "%s / %s", speedText(wheel.speed, testedRpm(wheel.speed)),
                    speedText(target, testedRpm(target)));
            return t < 0.22 || Math.abs(wheel.speed - target) > target * 0.045;
        });
    }

    private BayesianPidSearch.Result measureSteps(Flywheel wheel, double[] gains, double[] targets, double high,
                                                   boolean verification) throws InterruptedException {
        wheel.setGains(gains);
        double total = 0, worst = 0, margin = 0.11, overspeed = Math.max(high * 1.12, fullPowerSpeed * 1.08);
        boolean settled = true;
        String firstFailure = "";
        for (double target : targets) {
            double initial = wheel.speed;
            double step = target - initial;
            if (Math.abs(step) < wheel.reference() * 0.05) {
                total += 4;
                worst = Math.max(worst, 4);
                settled = false;
                if (firstFailure.isEmpty()) firstFailure = "未形成有效转速阶跃";
                continue;
            }
            StepScore response = new StepScore(STEP_DURATION);
            boolean[] runaway = {false};
            runFor(STEP_DURATION, (t, dt) -> {
                wheel.sample(dt);
                if (wheel.faulted()) return false;
                if (wheel.speed > overspeed) {
                    runaway[0] = true;
                    return false;
                }
                wheel.regulate(target, dt);
                peakPidPower = Math.max(peakPidPower, wheel.output);
                response.sample(t, dt, (wheel.speed - initial) / step, wheel.output,
                        Math.abs(wheel.speed - target)
                                <= target * FlywheelConfig.FLYWHEEL_SPEED_TOLERANCE);
                show((verification ? "飞轮 PID 复测 " : "飞轮 PID 搜索 ") + trials,
                        "%s / %s，功率 %.2f，峰值 %.2f",
                        speedText(wheel.speed, testedRpm(wheel.speed)),
                        speedText(target, testedRpm(target)), wheel.output, peakPidPower);
                return verification || !response.finishIfSettled(0.55);
            });
            if (!active(wheel)) return null;
            if (runaway[0]) return new BayesianPidSearch.Result(10, -0.6, false, "转速保护");
            boolean arrived = Double.isFinite(response.settlingTime(PID_HOLD));
            margin = Math.min(margin, 0.11 - response.overshoot() * Math.abs(step) / target);
            if (firstFailure.isEmpty() && (!arrived || margin < 0)) {
                firstFailure = speedText(target, testedRpm(target)) + (margin < 0 ? " 超调超限"
                        : String.format(Locale.US, " 未稳速，末端 %+.1f%%", 100 * (wheel.speed / target - 1)));
            }
            double cost = Math.min(9, arrived ? response.scoreWithSettlingTime(PID_HOLD) : 2.6 + response.score());
            total += cost;
            worst = Math.max(worst, cost);
            settled &= arrived;
        }
        double score = 0.70 * total / targets.length + 0.30 * worst;
        telemetry.log().add("飞轮第 %d 轮 %.3f，超调裕量 %+.2f%s", trials, score, margin,
                settled && margin >= 0 ? "，稳速" : "");
        return new BayesianPidSearch.Result(score, margin, settled, firstFailure);
    }

    private boolean active(Flywheel wheel) {
        return opModeIsActive() && !motionAborted() && !wheel.faulted();
    }
}
