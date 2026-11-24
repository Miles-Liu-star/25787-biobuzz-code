package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * 基础无头底盘 + 双电机 Shooter + Intake/通道 + 双闸门舵机 + 双俯仰舵机
 *
 * 控制说明：
 *   左摇杆移动   —— 场地坐标系前后 / 左右平移
 *   右摇杆左右   —— 原地旋转
 *   START        —— 重置无头
 *
 *   ↑ / ↓        —— Shooter 目标 RPM ±25
 *   ← / →        —— 小球通道俯仰 ±0.02（双舵机同轴反向）
 *   左 Bumper    —— ★ 同时控制两个闸门（s01 小球 + s1 大球），按住开，松开关
 *   右 Trigger   —— 吸球：intake/通道 线性转（按得越深越快）
 *   右 Bumper    —— 发射：打开两个闸门 → 0.1s 后两个电机满速转 2s → 关闸门
 *
 * 闸门：
 *   小球通道 s01：关闭 0.00，开启 0.19
 *   大球通道 s1 ：关闭 0.09，开启 0.00
 *
 * 俯仰（s2 / s3）：
 *   同轴反向：s2 = pitchPos，s3 = 1 - pitchPos
 *   初始 0.5，每次 ±0.02，范围 0.2~0.8
 *
 * 优先级：发射（RB）> 手动闸门（LB）> 吸球（RT）
 */
@TeleOp(name = "无头手动", group = "Test")
public class Teleop01 extends OpMode {

    /* ==================== 硬件 ==================== */
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private IMU imu;

    private DcMotorEx shooter0;
    private DcMotorEx shooter1;

    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;

    private Servo smallGateServo;   // s01 小球通道闸门
    private Servo bigGateServo;     // s1  大球通道闸门
    private Servo smallPitchServo1; // s2  小球通道俯仰舵机 1
    private Servo smallPitchServo2; // s3  小球通道俯仰舵机 2（反向）
    private Servo shovel_lr = null; //

    /* ==================== 参数 ==================== */
    private static final double STICK_DEADZONE   = 0.05;
    private static final double TRIGGER_DEADZONE = 0.01;

    // Shooter PD
    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_INIT = 3375.0;
    private static final double RPM_STEP = 25.0;
    private static final double MIN_RPM  = 0.0;
    private static final double MAX_RPM  = 6000.0;

    private double kP = 0.0004;
    private double kD = 0.000002;
    private double kF = 0.00020;
    private static final double MIN_POWER = -0.1;
    private static final double MAX_POWER = 1.0;

    // 闸门位置
    private static final double GATE_SMALL_CLOSED = 0.00;
    private static final double GATE_SMALL_OPEN   = 0.19;
    private static final double GATE_BIG_CLOSED   = 0.09;
    private static final double GATE_BIG_OPEN     = 0.00;

    // 俯仰舵机
    private static final double PITCH_INIT = 0.5;
    private static final double PITCH_STEP = 0.02;
    private static final double PITCH_MIN  = 0.2;
    private static final double PITCH_MAX  = 0.8;
    private static final boolean PITCH_MIRROR = true;
    private static final boolean PITCH_LEFT_DECREASES = true;

    // 发射时序
    private static final double FIRE_OPEN_DELAY_SEC = 0.1;
    private static final double FIRE_RUN_SEC        = 2.0;
    private static final double FIRE_MOTOR_POWER    = 1.0;

    /* ==================== 状态 ==================== */
    private boolean prevStart = false;
    private boolean prevUp    = false;
    private boolean prevDown  = false;
    private boolean prevLeft  = false;
    private boolean prevRight = false;
    private boolean prevRB    = false;

    private double targetRPM   = RPM_INIT;
    private double filteredRPM = 0;
    private double lastError   = 0;
    private double lastTime    = 0;

    private double pitchPos = PITCH_INIT;

    private boolean fireActive = false;
    private double  fireStartTime = 0;
    private boolean shovel_model = true;// true 为高 false 为低
    private boolean was_press_a = false;

    /* ==================== 初始化 ==================== */
    @Override
    public void init() {
        // 底盘
        frontLeft  = hardwareMap.get(DcMotorEx.class, "0");
        backLeft   = hardwareMap.get(DcMotorEx.class, "1");
        frontRight = hardwareMap.get(DcMotorEx.class, "2");
        backRight  = hardwareMap.get(DcMotorEx.class, "3");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Shooter
        shooter0 = hardwareMap.get(DcMotorEx.class, "00");
        shooter1 = hardwareMap.get(DcMotorEx.class, "01");
        shooter0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter0.setDirection(DcMotor.Direction.FORWARD);

        // Intake + 通道
        intakeMotor   = hardwareMap.get(DcMotorEx.class, "02");
        transferMotor = hardwareMap.get(DcMotorEx.class, "03");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        // 闸门舵机
        smallGateServo = hardwareMap.get(Servo.class, "s01");
        bigGateServo   = hardwareMap.get(Servo.class, "s1");
        smallGateServo.setPosition(GATE_SMALL_CLOSED);
        bigGateServo.setPosition(GATE_BIG_CLOSED);

        // 俯仰舵机
        smallPitchServo1 = hardwareMap.get(Servo.class, "s2");
        smallPitchServo2 = hardwareMap.get(Servo.class, "s3");
        pitchPos = PITCH_INIT;
        applyPitch();

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        telemetry.addLine("底盘 + Shooter + Intake + 闸门 + 俯仰 已初始化");
        telemetry.addLine("左摇杆移动 / 右摇杆旋转 / START 重置无头");
        telemetry.addLine("↑/↓ RPM  /  ←/→ 俯仰");
        telemetry.addLine("LB 两个闸门同时开关  /  RT 吸球  /  RB 发射");
        telemetry.update();
    }

    @Override
    public void start() {
        imu.resetYaw();
        lastTime = getRuntime();
        lastError = 0;
    }

    /* ==================== 主循环 ==================== */
    @Override
    public void loop() {
        double now = getRuntime();
        double dt = now - lastTime;
        if (dt < 1e-4) dt = 1e-4;
        lastTime = now;

        // ============ 1. 底盘 ============
        double drive  = deadzone(-gamepad1.left_stick_y);
        double strafe = deadzone( gamepad1.left_stick_x);
        double turn   = deadzone( gamepad1.right_stick_x);

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);

        double strafeRobot =  strafe * cos + drive * sin;
        double driveRobot  = -strafe * sin + drive * cos;

        double fl = driveRobot + strafeRobot + turn;
        double fr = driveRobot - strafeRobot - turn;
        double bl = driveRobot - strafeRobot + turn;
        double br = driveRobot + strafeRobot - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);

        // ============ 2. Shooter PD ============
        double rawRPM = -shooter0.getVelocity() * 60.0 / TICKS_PER_REV;
        filteredRPM += 0.4 * (rawRPM - filteredRPM);

        double error  = targetRPM - filteredRPM;
        double dError = (error - lastError) / dt;
        lastError = error;

        double power = clamp(kF * targetRPM + kP * error + kD * dError,
                MIN_POWER, MAX_POWER);

        shooter0.setPower(-power);
        shooter1.setPower(power);

        // ============ 3. ↑/↓ 调 RPM ============
        boolean up   = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up && !prevUp)     targetRPM = clamp(targetRPM + RPM_STEP, MIN_RPM, MAX_RPM);
        if (down && !prevDown) targetRPM = clamp(targetRPM - RPM_STEP, MIN_RPM, MAX_RPM);
        prevUp = up; prevDown = down;

        // ============ 4. ←/→ 调俯仰 ============
        boolean left  = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        if (left && !prevLeft) {
            double delta = PITCH_LEFT_DECREASES ? -PITCH_STEP : PITCH_STEP;
            pitchPos = clamp(pitchPos + delta, PITCH_MIN, PITCH_MAX);
            applyPitch();
        }
        if (right && !prevRight) {
            double delta = PITCH_LEFT_DECREASES ? PITCH_STEP : -PITCH_STEP;
            pitchPos = clamp(pitchPos + delta, PITCH_MIN, PITCH_MAX);
            applyPitch();
        }
        prevLeft = left; prevRight = right;

        // ============ 5. 扳机 / 发射 / 闸门 ============
        double rt = gamepad1.right_trigger;
        boolean rtPressed = rt > TRIGGER_DEADZONE;
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        // RB 边沿触发发射
        if (rb && !prevRB) {
            fireActive = true;
            fireStartTime = now;
        }
        prevRB = rb;

        double seriesPower = 0;
        boolean smallGateOpen;
        boolean bigGateOpen;

        if (fireActive) {
            // ---- 发射中：两个闸门都开 ----
            double elapsed = now - fireStartTime;
            smallGateOpen = true;
            bigGateOpen   = true;

            if (elapsed < FIRE_OPEN_DELAY_SEC) {
                seriesPower = 0;
            } else if (elapsed < FIRE_OPEN_DELAY_SEC + FIRE_RUN_SEC) {
                seriesPower = FIRE_MOTOR_POWER;
            } else {
                fireActive = false;
                smallGateOpen = false;
                bigGateOpen   = false;
                seriesPower = 0;
            }
        } else {
            // ---- 非发射：LB 同时控制两个闸门 ----
            smallGateOpen = lb;
            bigGateOpen   = lb;      // ★ 大球也跟随 LB

            if (rtPressed) {
                seriesPower = rt;
            }
        }

        // 应用闸门
        smallGateServo.setPosition(smallGateOpen ? GATE_SMALL_OPEN : GATE_SMALL_CLOSED);
        bigGateServo.setPosition(bigGateOpen   ? GATE_BIG_OPEN   : GATE_BIG_CLOSED);

        // 应用电机
        intakeMotor.setPower(seriesPower);
        transferMotor.setPower(seriesPower);

        // ============ 6. START 重置无头 ============
        boolean startBtn = gamepad1.start;
        if (startBtn && !prevStart) {
            imu.resetYaw();
        }
        prevStart = startBtn;

        //7.铲球
        if(gamepad1.a && !was_press_a)
        {
            shovel_model = !shovel_model;
        }
        was_press_a = gamepad1.a;
        if(shovel_model)
        {
            shovel_lr.setPosition(0);
        }
        else
        {
            shovel_lr.setPosition(0.48);
        }

        // ============ 8. 遥测 ============
        telemetry.addData("IMU Yaw (°)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("底盘 FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f", fl, bl, fr, br);
        telemetry.addData("Shooter 目标/实际", "%.0f / %.0f RPM", targetRPM, filteredRPM);
        telemetry.addData("Shooter 功率", "%.3f", power);
        telemetry.addData("俯仰位置", "%.2f", pitchPos);
        telemetry.addData("  s2 / s3", "%.2f / %.2f",
                smallPitchServo1.getPosition(), smallPitchServo2.getPosition());

        telemetry.addData("RT (吸球)", "%.2f", rt);
        telemetry.addData("LB (两个闸门)", lb ? "按住" : "松开");
        telemetry.addData("RB (发射)", fireActive ? "发射中" : "空闲");

        if (fireActive) {
            double elapsed = now - fireStartTime;
            String phase;
            if (elapsed < FIRE_OPEN_DELAY_SEC) phase = "等闸门";
            else if (elapsed < FIRE_OPEN_DELAY_SEC + FIRE_RUN_SEC) phase = "供弹中";
            else phase = "结束";
            telemetry.addData("  发射阶段", "%s (%.2f s)", phase, elapsed);
        }

        telemetry.addData("闸门 s01 / s1",
                (smallGateOpen ? "开" : "关") + " / " + (bigGateOpen ? "开" : "关"));
        telemetry.addData("Intake/通道 功率", "%.2f", seriesPower);
        telemetry.update();
    }

    /* ==================== 工具 ==================== */
    private void applyPitch() {
        smallPitchServo1.setPosition(pitchPos);
        smallPitchServo2.setPosition(PITCH_MIRROR ? 1.0 - pitchPos : pitchPos);
    }

    private static double deadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0.0 : v;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}