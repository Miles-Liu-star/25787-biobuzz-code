package org.firstinspires.ftc.teamcode.aim_bayes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pinpoint.PinPointConfig;
import org.firstinspires.ftc.teamcode.pinpoint.PinpointXYReader;
import org.firstinspires.ftc.teamcode.pinpoint.Maths;
import org.firstinspires.ftc.teamcode.shooter_bayes.BayesianPidSearch;
import org.firstinspires.ftc.teamcode.subsystems.AimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.List;
import java.util.Locale;

/** Physical turn-response calibration for AimSubsystem. Keep the robot in a clear area. */
@TeleOp(name = "自测自动瞄准 PID", group = "瞄准自测")
public final class AimPidCalibration extends LinearOpMode {
    private static final double[] SEARCH_SEED = {1.2, 0.05, 0.25};
    private static final double STEP_DURATION_SECONDS = 2.6;
    private static final double SETTLE_HOLD_SECONDS = 0.22;
    private static final double SETTLE_ANGLE_RADIANS = Math.toRadians(2.5);
    private static final double SETTLE_RATE_RADIANS = 0.16;
    private static final double MAX_OVERSHOOT = 0.20;
    // These are broad fault guards, not tracking tolerances; the chassis should still turn in place.
    private static final double MAX_POSITION_DRIFT_INCHES = 900.0 / 25.4;
    private static final double MAX_HEADING_DEVIATION_RADIANS = Math.toRadians(179);

    private AimCalibrationStore store;
    private PinpointXYReader pinpoint;
    private IMU imu;
    private DriveSubsystem drive;
    private AimSubsystem aim;
    private boolean anchored;
    private boolean sensorFault;
    private boolean driverMoved;
    private String sensorFaultReason = "";
    private int trialCount;
    private double peakTurnCommand;
    private double trialBaseHeading;
    private double trialBasePinpointHeading;
    private double trialBaseImuYaw;
    private double hubHeadingOffset;
    private double maximumPinpointDeviation;
    private double maximumImuDeviation;
    private boolean headingWarningLogged;
    private String outcome = "等待开始";
    private BayesianPidSearch completedSearch;

    @Override
    public void runOpMode() throws InterruptedException {
        store = new AimCalibrationStore(hardwareMap.appContext);
        pinpoint = new PinpointXYReader(hardwareMap);
        aim = new AimSubsystem();
        double[] storedGains = store.readGains();
        double[] initialGains = storedGains == null ? SEARCH_SEED : storedGains;
        aim.setGains(initialGains[0], initialGains[1], initialGains[2]);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
        drive = new DriveSubsystem(hardwareMap, telemetry);

        try {
            initializeBeforeStart();
            waitForStart();
            if (!opModeIsActive()) return;
            if (!anchorAfterStart()) {
                outcome = "Pinpoint 定位无效，未开始搜索";
                return;
            }
            if (!confirmSearch()) {
                outcome = "已跳过";
                return;
            }
            tune();
        } catch (RuntimeException failure) {
            outcome = "校准异常：" + failure.getClass().getSimpleName();
            telemetry.log().add("自动瞄准 PID 校准中断：%s", failure.getMessage());
        } finally {
            drive.stop();
        }

        while (opModeIsActive()) {
            showResults();
            telemetry.update();
            sleep(100);
        }
    }

    private void initializeBeforeStart() throws InterruptedException {
        boolean started = false;
        double nextStartAttempt = 0;
        while (!isStarted() && !isStopRequested()) {
            if (!started && Maths.time() >= nextStartAttempt) {
                started = pinpoint.start();
                nextStartAttempt = Maths.time() + 1.0;
            }
            pinpoint.update();
            if (started && !anchored && !pinpoint.needsReanchor()) anchored = anchorAtConfiguredStart();
            telemetry.addLine("瞄准 PID 贝叶斯搜索：原地往返转向阶跃测试");
            telemetry.addLine("机器人放在标准起点，周围至少留出 1 m 空间；测试中不要碰手柄摇杆。");
            telemetry.addLine("START 后按 A 开始，B 停止搜索；测试会驱动底盘原地转向。");
            telemetry.addData("Pinpoint", pinpoint.getStatus());
            telemetry.addData("定位有效", pinpoint.isPoseValidForControl());
            telemetry.update();
            sleep(25);
        }
    }

    private boolean anchorAfterStart() throws InterruptedException {
        double deadline = Maths.time() + 3.0;
        while (opModeIsActive() && Maths.time() < deadline) {
            pinpoint.update();
            if (!pinpoint.needsReanchor() && !anchored) anchored = anchorAtConfiguredStart();
            if (anchored && pinpoint.isPoseValidForControl()) return true;
            telemetry.addData("等待定位", "%s / 已锚定 %s", pinpoint.getStatus(), anchored);
            telemetry.update();
            sleep(20);
        }
        return anchored && pinpoint.isPoseValidForControl();
    }

    private boolean anchorAtConfiguredStart() {
        boolean success = pinpoint.setFieldOrigin(PinPointConfig.CENTER_FROM_REAR_EDGE_IN,
                PinPointConfig.CENTER_FROM_LEFT_EDGE_IN, 0) && pinpoint.isPoseValidForControl();
        if (success) alignHubHeadingToPinpoint();
        return success;
    }

    private boolean confirmSearch() throws InterruptedException {
        while (opModeIsActive() && (gamepad1.a || gamepad1.b)) sleep(20);
        while (opModeIsActive()) {
            pinpoint.update();
            telemetry.addLine("确认机器人四周安全，手柄摇杆保持不动。");
            telemetry.addData("搜索起点 PID", gainsText(aim.getGains()));
            telemetry.addLine("A 开始 20–32 轮贝叶斯搜索和复测；B 跳过。");
            telemetry.update();
            if (gamepad1.b) return false;
            if (gamepad1.a) return true;
            sleep(25);
        }
        return false;
    }

    private void tune() throws InterruptedException {
        double[] current = aim.getGains();
        BayesianPidSearch search = new BayesianPidSearch(
                new double[]{4.8, 1.5, 1.8},
                new double[]{0.70, 0.12, 0.20});
        BayesianPidSearch.Selection best = search.tune(current,
                (gains, verification, pattern) -> pidTrial(gains, verification, pattern),
                () -> !opModeIsActive() || gamepad1.b || sensorFault || driverMoved,
                store.readHistory(BayesianPidSearch.HISTORY_STRIDE, BayesianPidSearch.HISTORY_ROWS));
        completedSearch = search;

        if (opModeIsActive() && !gamepad1.b && !sensorFault && !driverMoved && search.searchCount() > 0) {
            store.saveHistory(search.history());
        }
        if (best != null && opModeIsActive() && !gamepad1.b && !sensorFault && !driverMoved) {
            outcome = store.saveGains(best.gains) ? "复测通过，PID 已保存并供正式瞄准使用" : "PID 写入失败";
            aim.setGains(best.gains[0], best.gains[1], best.gains[2]);
            telemetry.log().add("%s，代价 %.3f，P/I/D %.4f / %.4f / %.4f", outcome,
                    best.score, best.gains[0], best.gains[1], best.gains[2]);
        } else if (sensorFault) {
            outcome = "安全中止：" + sensorFaultReason + "，保留之前 PID";
        } else if (driverMoved || gamepad1.b || !opModeIsActive()) {
            outcome = "已中止，保留之前 PID";
        } else if (search.searchCount() < BayesianPidSearch.MIN_ROUNDS) {
            outcome = "有效搜索轮数不足，保留之前 PID";
        } else {
            outcome = "复测未通过，保留之前 PID";
        }
    }

    private BayesianPidSearch.Result pidTrial(double[] gains, boolean verification, int pattern)
            throws InterruptedException {
        if (!safeToMove()) return null;
        aim.setGains(gains[0], gains[1], gains[2]);
        double startX = pinpoint.getXInches(), startY = pinpoint.getYInches();
        double baseHeading = getControlHeading();
        trialBaseHeading = baseHeading;
        trialBasePinpointHeading = pinpoint.getHeading();

        setTargetBearing(baseHeading);
        if (!runFixedTarget(0.35, startX, startY, "瞄准 PID 准备")) return null;
        baseHeading = getControlHeading();
        trialBaseHeading = baseHeading;
        trialBasePinpointHeading = pinpoint.getHeading();
        trialBaseImuYaw = readImuYaw();
        headingWarningLogged = false;

        double degrees = pattern % 3 == 0 ? 22 : pattern % 3 == 1 ? 28 : 34;
        double step = Math.toRadians(degrees) * ((pattern & 1) == 0 ? 1 : -1);
        StepResult outward = measureStep(Maths.wrap(baseHeading + step), startX, startY,
                baseHeading, step, verification, pattern);
        if (outward == null || !safeToMove()) return null;

        double currentHeading = getControlHeading();
        double returnStep = Maths.wrap(baseHeading - currentHeading);
        StepResult returning = measureStep(baseHeading, startX, startY, currentHeading,
                returnStep, verification, pattern);
        if (returning == null) return null;

        double margin = MAX_OVERSHOOT - Math.max(outward.overshoot, returning.overshoot);
        boolean settled = outward.settled && returning.settled;
        double cost = 0.55 * (outward.cost + returning.cost) + 0.45 * Math.max(outward.cost, returning.cost);
        String reason = "";
        if (!outward.settled) reason = "正向阶跃未停稳";
        else if (!returning.settled) reason = "回程阶跃未停稳";
        else if (margin < 0) reason = "超调超限";
        trialCount++;
        telemetry.log().add("瞄准第 %d 轮 %.3f，超调裕量 %+.2f%s", trialCount, cost, margin,
                settled && margin >= 0 ? "，双向停稳" : "");
        return new BayesianPidSearch.Result(cost, margin, settled, reason);
    }

    private StepResult measureStep(double targetHeading, double originX, double originY,
                                   double phaseStartHeading, double requestedStep,
                                   boolean verification, int pattern) throws InterruptedException {
        if (Math.abs(requestedStep) < Math.toRadians(6)) return new StepResult(0, 0, true);
        setTargetBearing(targetHeading);
        StepScore score = new StepScore(STEP_DURATION_SECONDS, requestedStep);
        double startTime = Maths.time(), previousTime = startTime;
        double withinSeconds = 0;
        double settlingTime = Double.NaN;
        boolean settled = false;
        while (opModeIsActive() && !gamepad1.b) {
            double now = Maths.time(), elapsed = now - startTime;
            if (elapsed >= STEP_DURATION_SECONDS) break;
            if (!controlTick(originX, originY)) return null;
            double dt = Maths.clip(now - previousTime, 0.005, 0.25);
            previousTime = now;
            double error = wrapRadians(targetHeading - getControlHeading());
            double angularVelocity = readImuYawRate();
            boolean within = Math.abs(error) <= SETTLE_ANGLE_RADIANS
                    && Math.abs(angularVelocity) <= SETTLE_RATE_RADIANS;
            withinSeconds = within ? withinSeconds + dt : 0;
            score.sample(elapsed, dt, error);
            if (withinSeconds >= SETTLE_HOLD_SECONDS) {
                settled = true;
                settlingTime = elapsed;
                break;
            }
            showTrial(verification, pattern, elapsed, error, angularVelocity, gainsText(aim.getGains()));
            sleep(20);
        }
        if (!opModeIsActive() || gamepad1.b || sensorFault || driverMoved) return null;
        return new StepResult(score.cost(settled, settlingTime), score.overshoot(), settled);
    }

    private boolean runFixedTarget(double duration, double originX, double originY, String label)
            throws InterruptedException {
        double start = Maths.time();
        while (opModeIsActive() && !gamepad1.b && Maths.time() - start < duration) {
            if (!controlTick(originX, originY)) return false;
            showTrial(false, 0, Maths.time() - start,
                    wrapRadians(aim.getTargetHeadingRadians() - getControlHeading()),
                    readImuYawRate(), label);
            sleep(20);
        }
        return opModeIsActive() && !gamepad1.b && !sensorFault && !driverMoved;
    }

    private boolean controlTick(double originX, double originY) {
        if (!opModeIsActive() || gamepad1.b) return false;
        if (Math.abs(gamepad1.left_stick_x) > 0.05 || Math.abs(gamepad1.left_stick_y) > 0.05
                || Math.abs(gamepad1.right_stick_x) > 0.05) {
            driverMoved = true;
            drive.stop();
            return false;
        }
        pinpoint.update();
        if (!pinpoint.isPoseValidForControl()) {
            abortForSafety(pinpoint.needsReanchor()
                    ? "Pinpoint 要求重新锚定" : "Pinpoint 位置数据过期或不可用");
            return false;
        }
        double driftX = pinpoint.getXInches() - originX, driftY = pinpoint.getYInches() - originY;
        if (Math.hypot(driftX, driftY) > MAX_POSITION_DRIFT_INCHES) {
            abortForSafety(String.format(Locale.US, "平移漂移超过 %.1f in", MAX_POSITION_DRIFT_INCHES));
            return false;
        }
        double heading = getControlHeading();
        double pinpointDeviation = wrapRadians(pinpoint.getHeading() - trialBasePinpointHeading);
        double imuDeviation = wrapRadians(readImuYaw() - trialBaseImuYaw);
        maximumPinpointDeviation = Math.max(maximumPinpointDeviation, Math.abs(pinpointDeviation));
        maximumImuDeviation = Math.max(maximumImuDeviation, Math.abs(imuDeviation));
        if (Math.abs(pinpointDeviation) > Math.toRadians(45) && !headingWarningLogged) {
            headingWarningLogged = true;
            telemetry.log().add("航向偏差诊断：Pinpoint 起始/当前/变化 %.1f° / %.1f° / %+.1f°，REV IMU 变化 %+.1f°",
                    Math.toDegrees(trialBaseHeading), Math.toDegrees(heading),
                    Math.toDegrees(pinpointDeviation), Math.toDegrees(imuDeviation));
        }
        if (Math.abs(pinpointDeviation) > MAX_HEADING_DEVIATION_RADIANS) {
            abortForSafety(String.format(Locale.US,
                    "Pinpoint 航向偏离 %.1f°（起始 %.1f° / 当前 %.1f°），REV IMU 变化 %.1f°",
                    Math.toDegrees(pinpointDeviation), Math.toDegrees(trialBaseHeading),
                    Math.toDegrees(heading), Math.toDegrees(imuDeviation)));
            return false;
        }
        double turn = aim.calculateTurn(pinpoint.getXInches(), pinpoint.getYInches(), heading, readImuYawRate());
        peakTurnCommand = Math.max(peakTurnCommand, Math.abs(turn));
        drive.update(gamepad1, heading, true, turn);
        return true;
    }

    private void setTargetBearing(double bearingRadians) {
        double radius = 1000.0 / 25.4;
        aim.setTargetPosition(pinpoint.getXInches() + radius * Math.cos(bearingRadians),
                pinpoint.getYInches() + radius * Math.sin(bearingRadians));
    }

    private boolean safeToMove() {
        if (!opModeIsActive() || gamepad1.b || sensorFault || driverMoved) return false;
        pinpoint.update();
        if (!pinpoint.isPoseValidForControl()) {
            abortForSafety(pinpoint.needsReanchor()
                    ? "Pinpoint 要求重新锚定" : "Pinpoint 位置数据过期或不可用");
            return false;
        }
        return true;
    }

    private void abortForSafety(String reason) {
        sensorFault = true;
        sensorFaultReason = reason;
        drive.stop();
        telemetry.log().add("自动瞄准校准安全中止：%s", reason);
    }

    private double readImuYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private double readImuYawRate() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    private double getControlHeading() {
        return wrapRadians(readImuYaw() + hubHeadingOffset);
    }

    private void alignHubHeadingToPinpoint() {
        if (pinpoint.isHeadingUsable()) {
            hubHeadingOffset = wrapRadians(pinpoint.getHeading() - readImuYaw());
        }
    }

    private void showTrial(boolean verification, int pattern, double elapsed, double error,
                           double angularVelocity, String label) {
        telemetry.addData(verification ? "复测" : "搜索", "%d / %d", trialCount + 1, pattern + 1);
        telemetry.addData("阶段 / 时间", "%s / %.2f s", label, elapsed);
        telemetry.addData("航向误差 / 角速度", "%+.1f° / %+.2f rad/s",
                Math.toDegrees(error), angularVelocity);
        telemetry.addData("候选 PID", gainsText(aim.getGains()));
        telemetry.addData("峰值转向命令", "%.2f", peakTurnCommand);
        telemetry.addLine("B 停止；触碰摇杆会立即停止本轮");
        telemetry.update();
    }

    private void showResults() {
        telemetry.addData("结果", outcome);
        telemetry.addData("搜索历史 / 本轮", completedSearch == null ? "尚未搜索"
                : completedSearch.historyCount() + " / " + completedSearch.searchCount());
        telemetry.addData("当前 PID", gainsText(aim.getGains()));
        telemetry.addData("峰值转向命令", "%.2f", peakTurnCommand);
        telemetry.addData("最大航向变化 Pinpoint / REV", "%.1f° / %.1f°",
                Math.toDegrees(maximumPinpointDeviation), Math.toDegrees(maximumImuDeviation));
        if (completedSearch != null) {
            telemetry.addData("最低搜索代价", "%.3f", completedSearch.bestSearchCost());
            List<BayesianPidSearch.Review> reviews = completedSearch.reviews();
            for (BayesianPidSearch.Review review : reviews) {
                telemetry.addData(String.format(Locale.US, "复测 P/I/D %.3f / %.3f / %.3f",
                                review.gains[0], review.gains[1], review.gains[2]), "%d/%d 组通过，%s",
                        review.passed, BayesianPidSearch.VERIFICATION_ROUNDS,
                        review.firstFailure == null ? String.format(Locale.US, "代价 %.3f", review.score)
                                : review.firstFailure);
            }
        }
    }

    private static String gainsText(double[] gains) {
        return String.format(Locale.US, "P %.4f / I %.4f / D %.4f", gains[0], gains[1], gains[2]);
    }

    private static double wrapRadians(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static final class StepResult {
        final double cost, overshoot;
        final boolean settled;

        StepResult(double cost, double overshoot, boolean settled) {
            this.cost = cost;
            this.overshoot = overshoot;
            this.settled = settled;
        }
    }

    private static final class StepScore {
        private final double duration, step;
        private double errorArea, overshoot, tailSquare, tailTime;
        private double lastError;

        StepScore(double duration, double step) {
            this.duration = duration;
            this.step = step;
        }

        void sample(double time, double dt, double angleError) {
            double normalizedError = angleError / step;
            double progress = 1.0 - normalizedError;
            errorArea += Math.abs(normalizedError) * dt;
            overshoot = Math.max(overshoot, Math.max(0, progress - 1));
            if (time >= duration * 0.65) {
                tailSquare += normalizedError * normalizedError * dt;
                tailTime += dt;
            }
            lastError = normalizedError;
        }

        double overshoot() {
            return overshoot;
        }

        double cost(boolean settled, double settlingTime) {
            double area = errorArea / duration;
            double tail = settled ? 0 : tailTime > 0 ? Math.sqrt(tailSquare / tailTime) : Math.abs(lastError);
            double settlingPenalty = settled ? settlingTime / duration : 1.5;
            return Math.min(10, 0.60 * settlingPenalty + 0.45 * area
                    + 0.65 * overshoot + 0.30 * tail);
        }
    }
}
