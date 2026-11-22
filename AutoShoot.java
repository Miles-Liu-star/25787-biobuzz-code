package org.firstinspires.ftc.teamcode.AutoRed;

/* 吸球电机 02、皮筋轮 03；右侧铲板舵机 00。
 * 小球阻球器：0 挡球，0.09 放球；FTC 配置名称暂用 s1，运行前须核对。
 * 电机方向须先进行安全空载测试。 */

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.Servo;
// 如果 Constants 不在此包中，请根据实际项目修改此导入。
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * 红方全自动路线（修正版）
 *
 * 保留原有路径和射球、吸球时序；在 path4 和 path7 起点放板，
 * 在 path5 和 path8 终点升板。左侧舵机暂不控制；
 * 射球时飞轮与皮筋轮同时运行，小球阻球器放球；吸球时阻球器挡球。
 * path7 保持 reverseTangent()，不得改为 tangent()。
 */
@Autonomous(name = "红方自动1", group = "Red")
@Config
public class RedAuto1 extends LinearOpMode {

    // 飞轮与皮筋轮同时运行的时长，不保证飞轮已达到目标转速。
    public static double SHOOT_DELAY = 0.8;
    // 阻球器硬件配置名称需与 Driver Station 中的名称一致；暂用 s1。
    public static String SMALL_GATE_NAME = "s1";
    // 已确认：0 挡球、0.09 放球。
    public static double SMALL_GATE_BLOCK = 0.0;
    public static double SMALL_GATE_RELEASE = 0.09;
    // 参考手动程序的正功率；实际方向须先空载测试。
    public static double INTAKE_POWER = 1.0;
    public static double FEEDER_POWER = 1.0;
    public static double INTAKE_DELAY = 1.2;
    public static double START_DELAY = 0.5;
    // 放板动作等待时间（秒），须按实际机构测试调整。
    public static double BOARD_LOWER_DELAY = 0.5;
    // 升板动作等待时间（秒），须按实际机构测试调整。
    public static double BOARD_RAISE_DELAY = 0.5;

    private Follower follower;
    private Servo rightBoardServo;
    private DcMotor intake;
    private DcMotor middleTurn;
    private Servo smallGate;
    private ShooterSubsystem shooterSubsystem;
    private boolean shooterEnabled = false;
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

    /* ---------------- 动作命令 ---------------- */

    private Command shootCmd() {
        return command(() -> {
            shooterSubsystem.resetShooterPID();
            shooterEnabled = true;
            middleTurn.setPower(FEEDER_POWER);
            smallGate.setPosition(SMALL_GATE_RELEASE);
            telemetry.addLine("SHOOT: 飞轮、皮筋轮启动，阻球器放球");
        }).andThen(waitSeconds(SHOOT_DELAY))
                .andThen(command(() -> {
                    shooterEnabled = false;
                    middleTurn.setPower(0);
                    shooterSubsystem.stop();
                    shooterSubsystem.resetShooterPID();
                    smallGate.setPosition(SMALL_GATE_BLOCK);
                }));
    }

    private Command intakeOnCmd() {
        return command(() -> {
            telemetry.addLine("INTAKE ON: 阻球器挡球");
            smallGate.setPosition(SMALL_GATE_BLOCK);
            intake.setPower(INTAKE_POWER);
        });
    }

    private Command intakeOffCmd() {
        return command(() -> {
            telemetry.addLine("INTAKE OFF");
            intake.setPower(0);
        });
    }

    /** 右侧舵机放下铲板；左侧舵机 02 暂不发送控制命令。 */
    private Command lowerBoardCmd() {
        return command(() -> {
            rightBoardServo.setPosition(0.49);
            telemetry.addLine("BOARD LOWER: 右侧 00 -> 0.49");
        }).andThen(waitSeconds(BOARD_LOWER_DELAY));
    }

    /** 右侧舵机升起铲板；左侧舵机 02 暂不发送控制命令。 */
    private Command raiseBoardCmd() {
        return command(() -> {
            rightBoardServo.setPosition(0.0);
            telemetry.addLine("BOARD RAISE: 右侧 00 -> 0");
        }).andThen(waitSeconds(BOARD_RAISE_DELAY));
    }

    /* ---------------- 自动序列 ---------------- */

    public Command autoRoutine() {
        return sequential(
                waitSeconds(START_DELAY),

                follow(follower, path1()),
                shootCmd(),

                follow(follower, path2()),
                intakeOnCmd(),
                waitSeconds(INTAKE_DELAY),
                intakeOffCmd(),

                follow(follower, path3()),
                shootCmd(),

                // 到达 path4 起点，完成放板后再行驶 path4。
                lowerBoardCmd(),
                follow(follower, path4()),
                intakeOnCmd(),
                waitSeconds(INTAKE_DELAY),
                intakeOffCmd(),

                follow(follower, path5()),
                // path5 结束后升板，等待动作完成后再射球。
                raiseBoardCmd(),
                shootCmd(),

                follow(follower, path6()),

                // 到达 path7 起点，完成放板后再行驶 path7。
                lowerBoardCmd(),
                follow(follower, path7()),
                intakeOnCmd(),
                waitSeconds(INTAKE_DELAY),
                intakeOffCmd(),

                follow(follower, path8()),
                // path8 结束后升板，等待动作完成后再行驶 path9。
                raiseBoardCmd(),

                follow(follower, path9()),
                shootCmd(),

                follow(follower, path10())
        );
    }

    /* ---------------- 主循环 ---------------- */

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        rightBoardServo = hardwareMap.get(Servo.class, "00");
        intake = hardwareMap.get(DcMotor.class, "02");
        middleTurn = hardwareMap.get(DcMotor.class, "03");
        smallGate = hardwareMap.get(Servo.class, SMALL_GATE_NAME);
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        shooterSubsystem.stop();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        middleTurn.setPower(0);
        // 左侧舵机 02 暂不初始化或发送位置命令；这不保证其断电或可自由转动。
        // 初始化时不强制移动铲板或阻球器；阻球器将在吸球或射球命令中定位。

        waitForStart();
        if (isStopRequested()) return;
        try {
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();
                if (shooterEnabled) {
                    shooterSubsystem.setShooterPower(shooterSubsystem.FPD());
                }

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading(deg)", Math.toDegrees(follower.pose().heading()));

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
            }
        } finally {
            intake.setPower(0);
            middleTurn.setPower(0);
            shooterEnabled = false;
            shooterSubsystem.stop();
            shooterSubsystem.resetShooterPID();
            // 不在停止时强制移动铲板，以免产生意外动作。
        }
    }

    /* ---------------- 路径定义 ---------------- */

    public Path path1() {
        return Paths.line(start, path1).tangent();
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
        // 注意：此处必须使用 reverseTangent()，不要改成 tangent()
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
