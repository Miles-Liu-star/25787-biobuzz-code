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

//路线是ok的，测过，但路线长，到时间就停止，会超时
@Autonomous(name = "红方自动 无云台", group = "Red")
public class RedAuto1 extends LinearOpMode {
    /* 旧 Path8 已合并；新 Path8 回射球区，新 Path9 停车。 */
    private static final double[] PATH_TIMEOUT_SEC = {2, 3, 5, 3, 3, 5, 3, 5, 4};
    private static final double AUTO_LIMIT_SEC = 29.5;
    private static final double PARK_RESERVE_SEC = 4.0;
    private static final double POS_TOL = 2.0;
    private static final double HEADING_TOL = 10.0;
    private static final double TARGET_RPM = 3375.0;
    private static final double RPM_TOL = 80.0;
    /* 提前结束判定：位置到点且速度低于此值时不再死等 follow 完成 */
    private static final double EARLY_DONE_SPEED = 4.0;

    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(58.77, 9.949, -90);
    private final Pose p1 = poses.of(58.8005, 14.5069, -90.3838);
    private final Pose p2 = poses.of(8.558, 9.1458, -90);
    private final Pose c2 = poses.of(16.1563, 20.4927, 0);
    private final Pose p3 = poses.of(49.0542, 121.0074, 110);
    private final Pose c3 = poses.of(24.0053, 93.2037, 0);
    private final Pose h30 = poses.of(49.0542, 121.0074, -90);
    private final Pose h31 = poses.of(49.0542, 121.0074, -114.2824);
    /* 110° 的等效短角 -250°，避免 h31 到 p3 走 +224° 长角 */
    private final Pose h3End = poses.of(49.0542, 121.0074, -250);
    private final Pose p4 = poses.of(47.0386, 132.2078, 90);
    private final Pose p5 = poses.of(49.2088, 120.6891, 110);
    private final Pose p6 = poses.of(22.6759, 46.6055, -180);
    private final Pose c6 = poses.of(21.1146, 93.96, 0);
    private final Pose p7 = poses.of(9.4364, 46.5721, -179.8556);
    private final Pose p8 = poses.of(49.08, 120.9256, 110);
    private final Pose c81 = poses.of(28.8863, 45.1362, 0);
    private final Pose c82 = poses.of(29.102, 115.2073, 0);
    private final Pose h80 = poses.of(49.08, 120.9256, -180);
    /* 110° 的等效短角 -250°，避免 h80 到 p8 走 +290° 长角 */
    private final Pose h8End = poses.of(49.08, 120.9256, -250);
    private final Pose p9 = poses.of(10, 89.4445, -0.0133);
    private final Pose c9 = poses.of(29.2477, 89.44, 0);

    private final DcMotor[] drive = new DcMotor[4];
    private final ElapsedTime clock = new ElapsedTime();
    private Follower follower;
    private DcMotorEx shooter0, shooter1, intake, transfer;
    private Servo smallGate, bigGate, pitch1, pitch2, shovel, shovelLeft;
    private boolean spin, rebaseNext, poseFault;
    private double rpm, nextTelemetry;
    private int activeLeg, timeoutCount, lastTimeout;
    private String status = "初始化";

    @Override
    public void runOpMode() {
        try {
            Scheduler.reset();
            for (int i = 0; i < drive.length; i++) {
                drive[i] = hardwareMap.get(DcMotor.class, Integer.toString(i));
            }
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            initMechanisms();
            while (opModeInInit()) {
                refreshStoppedPose();
                showPose();
                telemetry.addLine("等待开始：路线起点朝向 +90°");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            clock.reset();
            runRoute();
            if (!poseFault && clock.seconds() >= AUTO_LIMIT_SEC) status = "自动总限时到，停止";
        } finally {
            try {
                Scheduler.reset();
            } finally {
                stopDrive();
                stopMechanisms();
            }
        }
        telemetry.addData("结束状态", status);
        telemetry.addData("超时次数 / 最近超时段", "%d / %d", timeoutCount, lastTimeout);
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
        pitch1 = hardwareMap.get(Servo.class, "s2");
        pitch2 = hardwareMap.get(Servo.class, "s3");
        shovel = hardwareMap.get(Servo.class, "s00");
        shovelLeft = hardwareMap.get(Servo.class, "s02");
        setSeries(0);
        setGates(false);
        pitch1.setPosition(0.5);
        pitch2.setPosition(0.5);
        setShovel(false);
    }

    private void configureMotor(DcMotorEx motor, DcMotor.ZeroPowerBehavior zero) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(zero);
    }

    private void runRoute() {
        waitStopped(500, AUTO_LIMIT_SEC);
        spin = true;
        shootAfter(runLeg(1, start, p1), p1, AUTO_LIMIT_SEC);
        collectLeg(2, p1, p2);
        spin = true;
        shootAfter(runLeg(3, p2, p3), p3, AUTO_LIMIT_SEC);
        moveShovel(true);
        collectLeg(4, p3, p4);
        spin = true;
        boolean reached5 = runLeg(5, p4, p5);
        moveShovel(false);
        shootAfter(reached5, p5, AUTO_LIMIT_SEC);
        runLeg(6, p5, p6);
        moveShovel(true);
        collectLeg(7, p6, p7);
        moveShovel(false);
        spin = true;
        boolean reached8 = runLeg(8, p7, p8);
        shootAfter(reached8, p8, AUTO_LIMIT_SEC - PARK_RESERVE_SEC);

        if (!canRun(AUTO_LIMIT_SEC)) return;
        refreshStoppedPose();
        /* 停车路线从射球区出发，不能从场地另一端直接抄过去。 */
        if (!near(follower.pose(), p8, POS_TOL)) {
            status = "未到 Path9 停车入口，停止";
            return;
        }
        if (runLeg(9, p8, p9)) status = "Path9 停车完成";
    }

    private Path buildPath(int leg, Pose from) {
        switch (leg) {
            case 1:
                return Paths.line(from, p1).reverseTangent();
            case 2:
                return Paths.curve(from, c2, p2).heading(Interpolator.constant(p2));
            case 3:
                return Paths.curve(from, c3, p3).heading(Interpolator.piecewise()
                        .until(0.1373, Interpolator.constant(h30))
                        .until(0.7344, Interpolator.tangent.reverse())
                        .until(1.0, Interpolator.linear(h31, h3End)));
            case 4:
                return Paths.line(from, p4).heading(Interpolator.linear(p3, p4));
            case 5:
                return Paths.line(from, p5).heading(Interpolator.linear(p4, p5));
            case 6:
                return Paths.curve(from, c6, p6).heading(Interpolator.linear(p5, p6));
            case 7:
                return Paths.line(from, p7).tangent();
            case 8:
                return Paths.curve(from, c81, c82, p8).heading(Interpolator.piecewise()
                        .until(1.0, Interpolator.linear(h80, h8End)));
            case 9:
                return Paths.curve(from, c9, p9).reverseTangent();
            default:
                throw new IllegalArgumentException("未知路径: " + leg);
        }
    }

    private boolean runLeg(int leg, Pose nominalStart, Pose target) {
        if (!canRun(AUTO_LIMIT_SEC)) return false;
        activeLeg = leg;
        refreshStoppedPose();
        Pose actual = follower.pose();
        if (!valid(actual)) {
            poseFault = true;
            status = "定位无效，停止";
            return false;
        }
        /* 正常走文档路线；上一段没到位时，用实测坐标接入下一段。 */
        Pose from = rebaseNext ? actual : nominalStart;
        if (near(from, target, 0.05)) {
            rebaseNext = !near(actual, target, POS_TOL);
            status = rebaseNext ? "路径过短且未到点" : "已在目标点附近";
            return !rebaseNext;
        }
        boolean[] finished = {false};
        Command command = sequential(follow(follower, buildPath(leg, from)),
                instant(() -> finished[0] = true));
        ElapsedTime deadline = new ElapsedTime();
        Pose lastPose = null;
        ElapsedTime frameTime = new ElapsedTime();
        status = rebaseNext ? "从实测位置接入" : "跟随文档路线";
        try {
            Scheduler.schedule(command);
            while (canRun(AUTO_LIMIT_SEC) && !finished[0]
                    && deadline.seconds() < PATH_TIMEOUT_SEC[leg - 1]) {
                follower.update();
                Pose curr = follower.pose();
                if (!valid(curr)) {
                    poseFault = true;
                    status = "定位无效，停止";
                    break;
                }

                /* 位置到点 + 速度低就提前结束，避免末端 heading 未转完时死等 follow 完成 */
                double dt = Math.max(frameTime.seconds(), 0.001);
                frameTime.reset();
                if (lastPose != null) {
                    double speed = Math.hypot(curr.x() - lastPose.x(),
                            curr.y() - lastPose.y()) / dt;
                    if (near(curr, target, POS_TOL) && speed < EARLY_DONE_SPEED) {
                        finished[0] = true;
                        status = "Path" + leg + " 到点且速度低，提前结束";
                        break;
                    }
                }
                lastPose = curr;

                if (deadline.seconds() >= PATH_TIMEOUT_SEC[leg - 1] || !canRun(AUTO_LIMIT_SEC)) break;
                Scheduler.execute();
                updateFlywheel();
                report();
                idle();
            }
            boolean reached = finished[0] && canRun(AUTO_LIMIT_SEC)
                    && near(follower.pose(), target, POS_TOL);
            if (!finished[0] && canRun(AUTO_LIMIT_SEC)
                    && deadline.seconds() >= PATH_TIMEOUT_SEC[leg - 1]) {
                timeoutCount++;
                lastTimeout = leg;
                status = "Path" + leg + " 超时";
            } else if (finished[0] && !reached && canRun(AUTO_LIMIT_SEC)) {
                status = "Path" + leg + " 跟随结束但未到点";
            }
            rebaseNext = !reached;
            return reached;
        } finally {
            try {
                if (Scheduler.isScheduled(command)) Scheduler.cancel(command);
            } finally {
                stopDrive();
            }
        }
    }

    private void shootAfter(boolean reached, Pose target, double limit) {
        try {
            if (!canRun(AUTO_LIMIT_SEC)) return;
            refreshStoppedPose();
            if (!reached || !atShotPose(target)) {
                status = "跳过射球：路径或姿态未到位";
                return;
            }
            double readyUntil = Math.min(limit - 2.10, clock.seconds() + 1.5);
            if (!canRun(readyUntil)) {
                status = "跳过射球：时间不足";
                return;
            }
            status = "等待飞轮达速";
            while (canRun(readyUntil) && !atSpeed()) tickStopped();
            if (!canRun(limit - 2.10) || !atSpeed() || !atShotPose(target)) {
                status = "跳过射球：未达速、未到位或时间不足";
                return;
            }
            status = "射球";
            setSeries(0);
            setGates(true);
            waitStopped(100, limit);
            if (!canRun(limit) || !atShotPose(target)) return;
            setSeries(1.0);
            waitStopped(2000, limit);
        } finally {
            stopMechanisms();
        }
    }

    private void collectLeg(int leg, Pose from, Pose target) {
        if (!canRun(AUTO_LIMIT_SEC)) return;
        setGates(false);
        setSeries(1.0);
        try {
            if (runLeg(leg, from, target)) waitStopped(600, AUTO_LIMIT_SEC);
        } finally {
            setSeries(0);
        }
    }

    private void moveShovel(boolean down) {
        if (!canRun(AUTO_LIMIT_SEC)) return;
        setShovel(down);
        waitStopped(500, AUTO_LIMIT_SEC);
    }

    private void setShovel(boolean down) {
        shovel.setPosition(down ? 0.48 : 0.0);
        shovelLeft.setPosition(down ? 0.52 : 1.0);
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
        setSeries(0);
        if (shooter0 != null) shooter0.setPower(0);
        if (shooter1 != null) shooter1.setPower(0);
        setGates(false);
    }

    private void stopDrive() {
        for (DcMotor motor : drive) if (motor != null) motor.setPower(0);
    }

    private void refreshStoppedPose() {
        try {
            follower.update();
        } finally {
            stopDrive();
        }
    }

    private void updateFlywheel() {
        double raw = -shooter0.getVelocity() * 60.0 / 28.0;
        if (!Double.isFinite(raw)) {
            rpm = Double.NaN;
            shooter0.setPower(0);
            shooter1.setPower(0);
            return;
        }
        rpm = Double.isFinite(rpm) ? rpm + 0.4 * (raw - rpm) : raw;
        double power = spin ? Range.clip(0.00020 * TARGET_RPM
                + 0.0050 * (TARGET_RPM - rpm), -0.1, 1.0) : 0.0;
        shooter0.setPower(-power);
        shooter1.setPower(power);
    }

    private boolean atSpeed() {
        return Math.abs(rpm - TARGET_RPM) <= RPM_TOL;
    }

    private boolean atShotPose(Pose target) {
        Pose actual = follower.pose();
        if (!near(actual, target, POS_TOL)) return false;
        double delta = Math.toDegrees(actual.heading() - target.heading());
        return Math.abs(Math.IEEEremainder(delta, 360.0)) <= HEADING_TOL;
    }

    private boolean near(Pose actual, Pose target, double tolerance) {
        return valid(actual) && Math.hypot(actual.x() - target.x(), actual.y() - target.y()) <= tolerance;
    }

    private boolean valid(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private boolean canRun(double limit) {
        return !poseFault && opModeIsActive() && !isStopRequested() && clock.seconds() < limit;
    }

    private void waitStopped(long millis, double limit) {
        ElapsedTime timer = new ElapsedTime();
        while (canRun(limit) && timer.milliseconds() < millis) tickStopped();
    }

    private void tickStopped() {
        refreshStoppedPose();
        if (!valid(follower.pose())) {
            poseFault = true;
            status = "定位无效，停止";
            stopMechanisms();
            return;
        }
        updateFlywheel();
        report();
        idle();
    }

    private void report() {
        double now = clock.seconds();
        if (now < nextTelemetry) return;
        nextTelemetry = now + 0.10;
        showPose();
        telemetry.addData("路径 / 超时次数 / 最近超时", "%d / %d / %d", activeLeg, timeoutCount, lastTimeout);
        telemetry.addData("飞轮 RPM", "%.0f / %.0f", rpm, TARGET_RPM);
        telemetry.addData("状态", status);
        telemetry.addData("自动时间", "%.1f s", now);
        telemetry.update();
    }

    private void showPose() {
        Pose pose = follower.pose();
        if (valid(pose)) {
            telemetry.addData("X / Y / 航向", "%.2f / %.2f / %.1f°",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        } else {
            telemetry.addLine("未读取到有效位姿");
        }
    }
}

