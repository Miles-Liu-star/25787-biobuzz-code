package org.firstinspires.ftc.teamcode.automatic;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//减了路线，但是一直没测好，有各种突发原因
@Autonomous(name = "红方自动solo", group = "Red")
public class RedAuto2 extends LinearOpMode {

    private static final double AUTO_END_SEC = 29.5;
    private static final double PARK_RESERVE_SEC = 4.5;
    private static final double WORK_END_SEC = AUTO_END_SEC - PARK_RESERVE_SEC;

    private static final double[] PATH_TIMEOUT_SEC = {4, 7, 3, 3, 5, 3, 7, 3.5};
    private static final double[] WORK_ESTIMATE_SEC = {0.0, 2.5, 4.5, 2.5, 1.5, 4.0, 1.6, 4.5};

    private static final double END_POS_TOL = 2.0;
    private static final double CONTINUE_POS_TOL = 8.0;
    private static final double PARK_ENTRY_POS_TOL = 8.0;

    private static final double SHOT_POS_TOL = 2.0;
    private static final double SHOT_HEADING_TOL = 10.0;
    private static final double FIRST_SHOT_POS_TOL = 4.0;
    private static final double FIRST_SHOT_HEADING_TOL = 20.0;
    private static final long SHOVEL_SETTLE_MS = 500;
    private static final long EXTRA_INTAKE_MS = 600;
    private static final int MAX_INVALID_SAMPLES = 3;
    private static final double TARGET_RPM = 3375.0;
    private static final double RPM_TOL = 80.0;

    private static final long BEFORE_GATE_MS = 200;
    private static final long GATE_SETTLE_MS = 300;
    private static final long FEED_MS = 2000;
    private static final double SHOT_ACTION_SEC = (BEFORE_GATE_MS + GATE_SETTLE_MS + FEED_MS) / 1000.0;
    private static final double SPINUP_WAIT_SEC = 1.5;

    private final PoseFactory poses = PoseFactory.degrees();

    // === 坐标点，严格对齐《路线（更新1：11）》 ===
    private final Pose start = poses.of(58.77, 9.949, -90);
    private final Pose p1 = poses.of(8.558, 9.1458, -90);
    private final Pose c1 = poses.of(5.7019, 25.0337, 0);
    private final Pose p2 = poses.of(48.5176, 117.3348, -250);
    private final Pose c2a = poses.of(26.2678, 33.8776, 0);
    private final Pose c2b = poses.of(17.6894, 93.8953, 0);
    private final Pose p3 = poses.of(47.0386, 132.2078, 90);
    private final Pose p4 = poses.of(48.4032, 117.1751, -250);
    private final Pose p5 = poses.of(26.0462, 46.6055, -180);
    private final Pose c5 = poses.of(21.1146, 93.96, 0);
    private final Pose p6 = poses.of(9.4364, 46.5721, -179.8849);
    private final Pose p7 = poses.of(48.4339, 117.1759, 110);
    private final Pose c7a = poses.of(28.8863, 45.1362, 0);
    private final Pose c7b = poses.of(29.102, 115.2073, 0);
    private final Pose p8 = poses.of(10, 89.4445, -0.0133);
    private final Pose c8 = poses.of(29.2477, 89.44, 0);

    private final ElapsedTime clock = new ElapsedTime();
    private Follower follower;
    private DcMotorEx shooter0, shooter1, intake, transfer;
    private Servo smallGate, bigGate, shovel, shovelLeft;
    private boolean spin, poseFault, rebaseNext;
    private int invalidSamples;
    private int activeLeg;
    private int timeoutCount, lastTimeout;
    private double rpm = Double.NaN;
    private double nextTelemetry;
    private String status = "初始化";
    private String workExit = "";

    @Override
    public void runOpMode() {
        try {
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            initMechanisms();

            while (opModeInInit()) {
                refreshStoppedPose();
                showPose();
                telemetry.addLine("等待开始：起点航向 -90°");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            clock.reset();
            runWorkRoute();
            workExit = status;
            doParking();
        } finally {
            try {
                Scheduler.reset();
            } finally {
                try {
                    stopDrive();
                } finally {
                    try {
                        stopMechanisms();
                    } finally {
                        setShovel(false);
                    }
                }
            }
        }
        telemetry.addData("工作结束状态", workExit);
        telemetry.addData("停车结束状态", status);
        telemetry.addData("超时次数 / 最近超时", "%d / %d", timeoutCount, lastTimeout);
        showPose();
        telemetry.update();
    }

    private void initMechanisms() {
        shooter0 = hardwareMap.get(DcMotorEx.class, "00");
        configureMotor(shooter0, DcMotor.ZeroPowerBehavior.FLOAT);
        shooter0.setDirection(DcMotor.Direction.FORWARD);
        shooter1 = hardwareMap.get(DcMotorEx.class, "01");
        configureMotor(shooter1, DcMotor.ZeroPowerBehavior.FLOAT);
        intake = hardwareMap.get(DcMotorEx.class, "02");
        configureMotor(intake, DcMotor.ZeroPowerBehavior.BRAKE);
        transfer = hardwareMap.get(DcMotorEx.class, "03");
        configureMotor(transfer, DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        smallGate = hardwareMap.get(Servo.class, "s01");
        bigGate = hardwareMap.get(Servo.class, "s1");
        shovel = hardwareMap.get(Servo.class, "s00");
        shovelLeft = hardwareMap.get(Servo.class, "s02");
        stopMechanisms();
        setShovel(false);
    }

    private void configureMotor(DcMotorEx motor, DcMotor.ZeroPowerBehavior zero) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(zero);
    }

    private void runWorkRoute() {
        shootIfTime(start, 0, FIRST_SHOT_POS_TOL, FIRST_SHOT_HEADING_TOL);

        if (!collectLeg(1, start, p1, false)) return;
        if (!driveAndShoot(2, p1, p2)) return;

        if (!atPose(follower.pose(), p2, END_POS_TOL, SHOT_HEADING_TOL)) {
            if (!forceHeadingRotate(p2)) return;
        }

        if (!collectLeg(3, p2, p3, true)) return;
        if (!driveAndShoot(4, p3, p4)) return;
        if (!collectLeg(5, p4, p5, true)) return;
        if (!followWorkLeg(6, p5, p6)) return;
        if (!driveAndShoot(7, p6, p7)) return;

        status = "Path1–7 已依次尝试";
    }

    private Path buildPath(int leg, Pose from) {
        switch (leg) {
            case 1:
                return Paths.line(from, p1).reverseTangent();
            case 2:
                return Paths.curve(from, c2a, c2b, p2).heading(Interpolator.linear(from, p2));
            case 3:
                return Paths.line(from, p3).heading(Interpolator.linear(from, p3));
            case 4:
                return Paths.line(from, p4).heading(Interpolator.linear(from, p4));
            case 5:
                return Paths.curve(from, c5, p5).heading(Interpolator.linear(from, p5));
            case 6:
                return Paths.line(from, p6).tangent();
            case 7:
                return Paths.curve(from, c7a, c7b, p7).heading(Interpolator.linear(from, p7));
            case 8:
                return Paths.curve(from, c8, p8).reverseTangent();
            default:
                throw new IllegalArgumentException("未知路径: " + leg);
        }
    }

    private boolean forceHeadingRotate(Pose targetPose) {
        if (!canRun(WORK_END_SEC)) return false;
        Pose actual = refreshStoppedPose();
        if (!checkPose(actual)) return false;

        Pose rotateTarget = poses.of(
                actual.x() + 0.5 * Math.cos(actual.heading()),
                actual.y() + 0.5 * Math.sin(actual.heading()),
                targetPose.heading()
        );
        Path rotatePath = Paths.line(actual, rotateTarget)
                .heading(Interpolator.linear(actual, rotateTarget));

        boolean[] finished = {false};
        Command command = sequential(follow(follower, rotatePath), instant(() -> finished[0] = true));
        ElapsedTime timer = new ElapsedTime();

        try {
            Scheduler.schedule(command);
            while (canRun(WORK_END_SEC) && !finished[0] && timer.seconds() < 2.0) {
                follower.update();
                if (!checkPose(follower.pose())) { stopDrive(); idle(); continue; }
                Scheduler.execute();
                updateFlywheel();
                report();
                idle();
            }
        } finally {
            if (Scheduler.isScheduled(command)) Scheduler.cancel(command);
            stopDrive();
        }
        Pose afterRotate = refreshStoppedPose();
        return atPose(afterRotate, rotateTarget, END_POS_TOL, SHOT_HEADING_TOL);
    }

    private boolean driveAndShoot(int leg, Pose nominalStart, Pose target) {
        if (!followWorkLeg(leg, nominalStart, target)) return false;
        shootIfTime(target, leg, SHOT_POS_TOL, SHOT_HEADING_TOL);
        return canRun(WORK_END_SEC);
    }

    private boolean followWorkLeg(int leg, Pose nominalStart, Pose target) {
        activeLeg = leg;
        if (!requireTime(WORK_END_SEC, "Path" + leg)) return false;

        Pose actual = refreshStoppedPose();
        if (!checkPose(actual)) return false;

        Pose from;
        if (rebaseNext) {
            from = actual;
            rebaseNext = false;
        } else {
            from = nominalStart;
        }

        Path path = buildPath(leg, from);
        return followPathWithTimeout(path, PATH_TIMEOUT_SEC[leg - 1], target);
    }

    private boolean followPathWithTimeout(Path path, double timeout, Pose target) {
        boolean[] finished = {false};
        Command command = sequential(follow(follower, path), instant(() -> finished[0] = true));
        ElapsedTime timer = new ElapsedTime();
        boolean timedOut = false;

        try {
            Scheduler.schedule(command);
            while (canRun(WORK_END_SEC) && !finished[0] && timer.seconds() < timeout) {
                follower.update();
                if (!checkPose(follower.pose())) { stopDrive(); idle(); continue; }
                Scheduler.execute();
                updateFlywheel();
                report();
                idle();
            }
        } finally {
            if (!finished[0] && timer.seconds() >= timeout) timedOut = true;
            if (Scheduler.isScheduled(command)) Scheduler.cancel(command);
            stopDrive();
        }

        if (timedOut) {
            timeoutCount++;
            lastTimeout = activeLeg;
        }

        if (!requireTime(WORK_END_SEC, "Path" + activeLeg + " 完成")) return false;
        Pose actual = refreshStoppedPose();
        if (!checkPose(actual)) return false;

        return near(actual, target, CONTINUE_POS_TOL);
    }

    private boolean collectLeg(int leg, Pose nominalStart, Pose target, boolean lowerShovel) {
        activeLeg = leg;
        if (!requireTime(WORK_END_SEC, "Path" + leg)) return false;
        try {
            if (lowerShovel) {
                status = "Path" + leg + "：放铲";
                setShovel(true);
                waitStopped(SHOVEL_SETTLE_MS, WORK_END_SEC);
                if (!requireTime(WORK_END_SEC, "放铲")) return false;
            }
            setGates(false);
            setSeries(1.0);

            if (!followWorkLeg(leg, nominalStart, target)) return false;

            status = "Path" + leg + "：补吸";
            for (int c = 0; c < 2; c++) {
                if (!bumpPath(-0.5, WORK_END_SEC)) return false;
                if (!bumpPath(1.0, WORK_END_SEC)) return false;
            }
            waitStopped(EXTRA_INTAKE_MS, WORK_END_SEC);
        } finally {
            setSeries(0);
            if (lowerShovel) setShovel(false);
        }

        rebaseNext = true;

        if (lowerShovel && canRun(WORK_END_SEC)) {
            status = "Path" + leg + "：收铲";
            waitStopped(SHOVEL_SETTLE_MS, WORK_END_SEC);
        }
        return requireTime(WORK_END_SEC, "Path" + leg + " 吸球后");
    }

    private boolean bumpPath(double offsetInches, double limit) {
        if (!canRun(limit)) return false;
        Pose currentPose = follower.pose();
        if (!valid(currentPose)) return false;

        double rad = currentPose.heading();
        Pose targetPose = poses.of(
                currentPose.x() + offsetInches * Math.cos(rad),
                currentPose.y() + offsetInches * Math.sin(rad),
                currentPose.heading()
        );
        Path bump = Paths.line(currentPose, targetPose)
                .heading(Interpolator.constant(currentPose.heading()));

        boolean[] finished = {false};
        Command cmd = sequential(follow(follower, bump), instant(() -> finished[0] = true));
        ElapsedTime t = new ElapsedTime();
        try {
            Scheduler.schedule(cmd);
            while (canRun(limit) && !finished[0] && t.seconds() < 1.0) {
                follower.update();
                if (!checkPose(follower.pose())) return false;
                Scheduler.execute();
                updateFlywheel();
                report();
                idle();
            }
        } finally {
            if (Scheduler.isScheduled(cmd)) Scheduler.cancel(cmd);
            stopDrive();
        }
        return canRun(limit);
    }

    private void shootIfTime(Pose target, int leg, double posTol, double headingTol) {
        spin = true;

        if (!requireTime(WORK_END_SEC, "射球")) return;

        Pose actual = refreshStoppedPose();
        if (!checkPose(actual)) return;
        if (!atPose(actual, target, posTol, headingTol)) {
            status = "跳过射球：位置或航向未到位";
            return;
        }

        double latestStart = WORK_END_SEC;
        for (int i = leg + 1; i <= 7; i++) latestStart -= WORK_ESTIMATE_SEC[i];
        latestStart -= SHOT_ACTION_SEC;

        if (!canRun(latestStart)) {
            status = "跳过射球：剩余时间不足";
            return;
        }

        status = "等待飞轮达速";
        updateFlywheel();
        double readyUntil = Math.min(latestStart, clock.seconds() + SPINUP_WAIT_SEC);
        while (canRun(readyUntil) && !atSpeed()) tickStopped();

        if (!requireTime(latestStart, "射球预留时间")) return;
        if (!atSpeed() || !atPose(follower.pose(), target, posTol, headingTol)) {
            status = "跳过射球：未达速或姿态偏离";
            return;
        }

        status = "射球";
        setSeries(0);
        waitStopped(BEFORE_GATE_MS, WORK_END_SEC);
        if (!requireTime(WORK_END_SEC, "开闸前")) return;

        setGates(true);
        waitStopped(GATE_SETTLE_MS, WORK_END_SEC);
        if (!requireTime(WORK_END_SEC, "送料前")) return;

        setSeries(1.0);
        waitStopped(FEED_MS, WORK_END_SEC);
        if (requireTime(WORK_END_SEC, "射球完成")) status = "射球动作完成";
        stopMechanisms();
    }

    private void doParking() {
        stopMechanisms();
        setShovel(false);
        if (!requireTime(AUTO_END_SEC, "停车")) return;

        Pose actual = refreshStoppedPose();
        if (!checkPose(actual)) return;

        if (!atPose(actual, p7, PARK_ENTRY_POS_TOL, 25.0)) {
            status = "未到 Path8 入口，原地停";
            return;
        }

        status = "停车前收铲";
        waitStopped(SHOVEL_SETTLE_MS, AUTO_END_SEC);

        actual = refreshStoppedPose();
        if (!checkPose(actual)) return;
        if (!atPose(actual, p7, PARK_ENTRY_POS_TOL, 25.0)) {
            status = "收铲后偏离入口，原地停";
            return;
        }

        if (!requireTime(AUTO_END_SEC, "Path8 出发")) return;
        Path dynamicPath = Paths.curve(actual, c8, p8).reverseTangent();
        if (followPathWithTimeout(dynamicPath, PATH_TIMEOUT_SEC[7], p8)) {
            status = "Path8 到达停车目标";
        }
    }

    private void updateFlywheel() {
        if (!spin || shooter0 == null) return;
        double raw = -shooter0.getVelocity() * 60.0 / 28.0;
        if (!Double.isFinite(raw)) {
            rpm = Double.NaN;
            shooter0.setPower(0);
            if (shooter1 != null) shooter1.setPower(0);
            return;
        }
        rpm = Double.isFinite(rpm) ? rpm + 0.4 * (raw - rpm) : raw;
        double power = Range.clip(0.00020 * TARGET_RPM + 0.0050 * (TARGET_RPM - rpm), -0.1, 1.0);
        shooter0.setPower(-power);
        if (shooter1 != null) shooter1.setPower(power);
    }

    private boolean atSpeed() {
        return Double.isFinite(rpm) && Math.abs(rpm - TARGET_RPM) <= RPM_TOL;
    }

    private void waitStopped(long millis, double limit) {
        ElapsedTime timer = new ElapsedTime();
        while (canRun(limit) && timer.milliseconds() < millis) tickStopped();
    }

    private void tickStopped() {
        if (!checkPose(refreshStoppedPose())) return;
        updateFlywheel();
        report();
        idle();
    }

    private Pose refreshStoppedPose() {
        if (follower == null) return null;
        follower.update();
        return follower.pose();
    }

    private boolean checkPose(Pose pose) {
        if (valid(pose)) {
            invalidSamples = 0;
            return true;
        }
        invalidSamples++;
        if (invalidSamples >= MAX_INVALID_SAMPLES) {
            poseFault = true;
            status = "定位连续无效，停止移动";
            stopDrive();
            stopMechanisms();
            return false;
        }
        return true;
    }

    private boolean canRun(double limit) {
        return !poseFault && opModeIsActive() && !isStopRequested() && clock.seconds() < limit;
    }

    private boolean requireTime(double limit, String action) {
        if (canRun(limit)) return true;
        if (poseFault) status = "定位无效，停止移动";
        else if (isStopRequested() || !opModeIsActive()) status = action + "：已停止";
        else status = action + "：时间不足";
        return false;
    }

    private boolean atPose(Pose actual, Pose target, double posTol, double headingTol) {
        if (!near(actual, target, posTol)) return false;
        double errorDeg = Math.toDegrees(actual.heading() - target.heading());
        return Math.abs(Math.IEEEremainder(errorDeg, 360.0)) <= headingTol;
    }

    private boolean near(Pose actual, Pose target, double tolerance) {
        return valid(actual) && valid(target) && Math.hypot(actual.x() - target.x(), actual.y() - target.y()) <= tolerance;
    }

    private boolean valid(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    private void setShovel(boolean down) {
        if (shovel != null) shovel.setPosition(down ? 0.48 : 0.0);
        if (shovelLeft != null) shovelLeft.setPosition(down ? 0.52 : 1.0);
    }

    private void setGates(boolean open) {
        if (smallGate != null) smallGate.setPosition(open ? 0.19 : 0.00);
        if (bigGate != null) bigGate.setPosition(open ? 0.00 : 0.09);
    }

    private void setSeries(double power) {
        if (intake != null) intake.setPower(power);
        if (transfer != null) transfer.setPower(power);
    }

    private void stopMechanisms() {
        spin = false;
        rpm = Double.NaN;
        setSeries(0);
        if (shooter0 != null) shooter0.setPower(0);
        if (shooter1 != null) shooter1.setPower(0);
        setGates(false);
    }

    private void stopDrive() {
        if (follower != null) {
            follower.manual(0, 0, 0);
        }
    }

    private void report() {
        double now = clock.seconds();
        if (now < nextTelemetry) return;
        nextTelemetry = now + 0.10;
        showPose();
        telemetry.addData("路径 / 超时次数", "%d / %d", activeLeg, timeoutCount);
        telemetry.addData("状态", status);
        telemetry.addData("飞轮 RPM", "%.0f / %.0f", rpm, TARGET_RPM);
        telemetry.update();
    }

    private void showPose() {
        Pose pose = follower == null ? null : follower.pose();
        if (valid(pose)) {
            telemetry.addData("X / Y / 航向", "%.2f / %.2f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
    }
}