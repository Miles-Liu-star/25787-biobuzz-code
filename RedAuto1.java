package org.firstinspires.ftc.teamcode.automatic;

/*
 * 硬件配置（与 Robot Controller 配置名称必须完全一致）：
 *   飞轮电机：00、01（ShooterSubsystem 内部读取）
 *   底盘：左前 0、左后 1、右前 2、右后 3（在 Constants.java 中配置）
 *   吸球电机：02    皮筋轮电机：03
 *   小球阻球器：s1（0 挡球，0.09 放球）
 *   铲板舵机：SHOVEL_NAME（扩展板上的舵机，名称和升/降位置须核对）
 *   大球阻球器 s01、左铲板 s02、pitch 舵机 s0/s2/s3：本程序不控制
 */

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
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

import org.firstinspires.ftc.teamcode.pedroRathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * 红方全自动路线
 *
 * - 射球前一段路径开始时预热飞轮，到点后达速才放球（带超时）。
 * - path4 / path7 起点放下铲板并开启吸球，边走边吸；
 *   path5 / path8 终点升起铲板。
 * - path7 保持 reverseTangent()，不得改为 tangent()。
 */
@Autonomous(name = "红方自动1", group = "Red")
public class RedAuto1 extends LinearOpMode {

    /* ---------------- 可调参数 ---------------- */

    // 必须与 ShooterSubsystem 中的 targetRPM 一致（靠场地边 1800）。
    public static double TARGET_RPM = 1800;
    // 认为“已达速”的允许误差（RPM）。
    public static double RPM_TOLERANCE = 80;
    // 到点后等待达速的最长时间（秒），超时后仍然放球。
    public static double SPINUP_TIMEOUT = 1.5;
    // 开始放球后持续送球的时间（秒）。
    public static double FEED_DURATION = 0.8;

    public static String SMALL_GATE_NAME = "s1";
    public static double SMALL_GATE_BLOCK = 0.0;
    public static double SMALL_GATE_RELEASE = 0.09;

    // 铲板舵机名称和位置须按实际配置核对（量程 0–0.635）。
    public static String SHOVEL_NAME = "s0expentionhub";
    public static double SHOVEL_UP = 0.0;
    public static double SHOVEL_DOWN = 0.49;
    public static double SHOVEL_MOVE_DELAY = 0.5;

    // 功率方向须先空载测试。
    public static double INTAKE_POWER = 1.0;
    public static double FEEDER_POWER = 1.0;
    public static double INTAKE_EXTRA_DELAY = 0.6;
    public static double START_DELAY = 0.5;

    /* ---------------- 硬件与状态 ---------------- */

    private Follower follower;
    private Servo shovel;
    private Servo smallGate;
    private DcMotor intake;
    private DcMotor feeder;
    private DcMotorEx shooterRead;   // 仅用于读取转速，与 ShooterSubsystem 的 00 是同一电机
    private ShooterSubsystem shooterSubsystem;

    private boolean shooterEnabled = false;
    private boolean feedRequested = false;
    private boolean feeding = false;
    private final ElapsedTime feedTimer = new ElapsedTime();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.0098, 14.4482, 89.9132);
    private final Pose point2 = poseFactory.of(8.7778, 8.3187, 90);
    private final Pose point2Control1 = poseFactory.of(16.1563, 20.4927, 0);
    private final Pose point3 = poseFactory.of(49.0542, 121.0074, -70);
    private final Pose point3Control1 = poseFactory.of(20.0759, 44.9294, 0);
    private final Pose point3Control2 = poseFactory.of(20.5513, 93.5285, 0);
    private final Pose point3Segment1Heading = poseFactory.of(49.0542, 121.0074, 90);
    private final Pose point3Segment3Start = poseFactory.of(49.0542, 121.0074, 66.237);
    private final Pose point3Segment3End = poseFactory.of(49.0542, 121.0074, -70);
    private final Pose point4 = poseFactory.of(47.0939, 133.2896, -90);
    private final Pose point5 = poseFactory.of(49.2088, 120.6891, -70.6268);
    private final Pose point5Segment1Heading = poseFactory.of(49.2088, 120.6891, -90);
    private final Pose point5Segment2Target = poseFactory.of(60, 90, 0);
    private final Pose point6 = poseFactory.of(22.6759, 46.6055, -88.1116);
    private final Pose point6Control1 = poseFactory.of(21.1146, 93.96, 0);
    private final Pose point6Segment1Heading = poseFactory.of(22.6759, 46.6055, -90);
    private final Pose point7 = poseFactory.of(8.0476, 46.5721, 0.1307);
    private final Pose point8 = poseFactory.of(27.7752, 46.4057, -0.4833);
    private final Pose point9 = poseFactory.of(49.08, 120.9256, -60);
    private final Pose point9Control1 = poseFactory.of(29.3975, 105.9785, 0);
    private final Pose point9Segment2Start = poseFactory.of(49.08, 120.9256, -105.0982);
    private final Pose point9Segment2End = poseFactory.of(49.08, 120.9256, -60);
    private final Pose point10 = poseFactory.of(10, 89.4445, 179.9867);
    private final Pose point10Control1 = poseFactory.of(29.2477, 89.44, 0);

    /* ---------------- 射球相关 ---------------- */

    /** 飞轮电机转速（RPM），与 ShooterSubsystem.FPD() 的计算方式一致。 */
    private double shooterRPM() {
        double tpr = shooterRead.getMotorType().getTicksPerRev();
        if (tpr <= 0) return 0;
        return Math.abs(shooterRead.getVelocity()) * 60.0 / tpr;
    }

    private boolean shooterAtSpeed() {
        return Math.abs(TARGET_RPM - shooterRPM()) <= RPM_TOLERANCE;
    }

    /** 预热：开启飞轮闭环，放在射球前那条路径之前。 */
    private Command spinUpCmd() {
        return command(() -> {
            if (!shooterEnabled) {
                shooterSubsystem.resetShooterPID();
                shooterEnabled = true;
            }
        });
    }

    /**
     * 射球：请求放球，主循环在达速（或超时）后开始送球。
     * 最多等待 SPINUP_TIMEOUT + FEED_DURATION 秒，送球结束后关闭机构。
     */
    private Command shootCmd() {
        return spinUpCmd()
                .andThen(command(() -> {
                    feedRequested = true;
                    feeding = false;
                    feedTimer.reset();
                }))
                .andThen(waitSeconds(SPINUP_TIMEOUT + FEED_DURATION))
                .andThen(command(this::stopShooting));
    }

    private void stopShooting() {
        feedRequested = false;
        feeding = false;
        shooterEnabled = false;
        feeder.setPower(0);
        smallGate.setPosition(SMALL_GATE_BLOCK);
        shooterSubsystem.stop();
        shooterSubsystem.resetShooterPID();
    }

    /** 主循环中调用：处理飞轮闭环与达速放球。 */
    private void updateShooter() {
        if (shooterEnabled) {
            shooterSubsystem.setShooterPower(shooterSubsystem.FPD());
        }
        if (feedRequested && !feeding) {
            if (shooterAtSpeed() || feedTimer.seconds() >= SPINUP_TIMEOUT) {
                feeding = true;
                feedTimer.reset();
                smallGate.setPosition(SMALL_GATE_RELEASE);
                feeder.setPower(FEEDER_POWER);
            }
        } else if (feedRequested && feeding && feedTimer.seconds() >= FEED_DURATION) {
            stopShooting();
        }
    }

    /* ---------------- 吸球 / 铲板 ---------------- */

    private Command intakeOnCmd() {
        return command(() -> {
            smallGate.setPosition(SMALL_GATE_BLOCK);
            intake.setPower(INTAKE_POWER);
        });
    }

    private Command intakeOffCmd() {
        return command(() -> intake.setPower(0));
    }

    private Command lowerShovelCmd() {
        return command(() -> shovel.setPosition(SHOVEL_DOWN))
                .andThen(waitSeconds(SHOVEL_MOVE_DELAY));
    }

    private Command raiseShovelCmd() {
        return command(() -> shovel.setPosition(SHOVEL_UP))
                .andThen(waitSeconds(SHOVEL_MOVE_DELAY));
    }

    /* ---------------- 自动序列 ---------------- */

    public Command autoRoutine() {
        return sequential(
                waitSeconds(START_DELAY),

                spinUpCmd(),
                follow(follower, path1()),
                shootCmd(),

                intakeOnCmd(),
                follow(follower, path2()),
                waitSeconds(INTAKE_EXTRA_DELAY),
                intakeOffCmd(),

                spinUpCmd(),
                follow(follower, path3()),
                shootCmd(),

                // path4 起点：放板并开始吸球，边走边吸
                lowerShovelCmd(),
                intakeOnCmd(),
                follow(follower, path4()),
                waitSeconds(INTAKE_EXTRA_DELAY),
                intakeOffCmd(),

                spinUpCmd(),
                follow(follower, path5()),
                // path5 终点：升板后射球
                raiseShovelCmd(),
                shootCmd(),

                follow(follower, path6()),

                // path7 起点：放板并开始吸球
                lowerShovelCmd(),
                intakeOnCmd(),
                follow(follower, path7()),
                waitSeconds(INTAKE_EXTRA_DELAY),
                intakeOffCmd(),

                follow(follower, path8()),
                // path8 终点：升板
                raiseShovelCmd(),

                spinUpCmd(),
                follow(follower, path9()),
                shootCmd(),

                follow(follower, path10())
        );
    }

    /* ---------------- 主程序 ---------------- */

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        shovel = hardwareMap.get(Servo.class, SHOVEL_NAME);
        smallGate = hardwareMap.get(Servo.class, SMALL_GATE_NAME);
        intake = hardwareMap.get(DcMotor.class, "02");
        feeder = hardwareMap.get(DcMotor.class, "03");
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        shooterRead = hardwareMap.get(DcMotorEx.class, "00");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        feeder.setPower(0);
        shooterSubsystem.stop();

        // 初始化安全位置：阻球器挡球、铲板升起（初始化时舵机会动，注意手离开）
        smallGate.setPosition(SMALL_GATE_BLOCK);
        shovel.setPosition(SHOVEL_UP);

        telemetry.addLine("初始化完成，等待开始");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try {
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();
                updateShooter();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading(deg)", Math.toDegrees(follower.pose().heading()));
                telemetry.addData("shooter RPM", "%.0f / %.0f", shooterRPM(), TARGET_RPM);
                telemetry.addData("feeding", feeding);
                if (follower.currentPath() != null) {
                    telemetry.addData("path index", follower.pathIndex());
                    telemetry.addData("distance remaining", follower.distanceToEndpoint());
                }
                telemetry.update();
            }
        } finally {
            intake.setPower(0);
            stopShooting();
        }
    }

    /* ---------------- 路径定义（保持原路线） ---------------- */

    public Path path1() {
        // 第一段行进时保持初始朝向，避免按路径切线计算产生微小角度跳变。
        return Paths.line(start, path1).constant(start);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3)
                .heading(Interpolator.piecewise()
                        .until(0.1373, Interpolator.constant(point3Segment1Heading))
                        .until(0.7344, Interpolator.tangent)
                        .until(1, Interpolator.linear(point3Segment3Start, point3Segment3End)));
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path5() {
        return Paths.line(point4, point5)
                .heading(Interpolator.piecewise()
                        .until(0.1657, Interpolator.constant(point5Segment1Heading))
                        .until(1, Interpolator.facingPoint(point5Segment2Target)));
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6)
                .heading(Interpolator.piecewise()
                        .until(0.0515, Interpolator.constant(point6Segment1Heading))
                        .until(1, Interpolator.tangent));
    }

    public Path path7() {
        // 必须使用 reverseTangent()，不要改成 tangent()
        return Paths.line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).tangent();
    }

    public Path path9() {
        return Paths.curve(point8, point9Control1, point9)
                .heading(Interpolator.piecewise()
                        .until(0.5949, Interpolator.tangent.reverse())
                        .until(1, Interpolator.linear(point9Segment2Start, point9Segment2End)));
    }

    public Path path10() {
        return Paths.curve(point9, point10Control1, point10).tangent();
    }
}
