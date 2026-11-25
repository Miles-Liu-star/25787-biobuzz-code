package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.subsystems.AimSubsystem;
import org.firstinspires.ftc.teamcode.pinpoint.PinpointXYReader;
import org.firstinspires.ftc.teamcode.pinpoint.PinPointConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * 基础无头底盘 + 双电机 Shooter + Intake/通道 + 双闸门舵机 + 双俯仰舵机
 *
 * 控制说明：
 *   左摇杆移动   —— 场地坐标系前后 / 左右平移
 *   右摇杆左右   —— 原地旋转
 *   START        —— 重新锚定 Pinpoint + 重置无头
 *   Y            —— 手动重新锚定
 *
 *   X            —— ★ 自动瞄准（按住，同时不推右摇杆时生效；会覆盖转向）
 *   ↑ / ↓        —— Shooter 目标 RPM ±25
 *   ← / →        —— 小球通道俯仰 ±0.02
 *   LB           —— 两个闸门同时开关
 *   RT           —— 吸球
 *   RB           —— 发射
 */
@TeleOp(name = "Basic Field-Centric Drive", group = "Test")
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

    private Servo smallGateServo;
    private Servo bigGateServo;
    private Servo smallPitchServo1;
    private Servo smallPitchServo2;
    private Servo shovel_lr = null;

    /* ==================== 参数 ==================== */
    private static final double STICK_DEADZONE   = 0.05;
    private static final double TRIGGER_DEADZONE = 0.01;

    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_INIT = 3375.0;
    private static final double RPM_STEP = 25.0;
    private static final double MIN_RPM  = 0.0;
    private static final double MAX_RPM  = 6000.0;

    private double kP = 0.00050;
    private double kD = 0.0000;
    private double kF = 0.00020;
    private static final double MIN_POWER = -0.1;
    private static final double MAX_POWER = 1.0;

    private static final double GATE_SMALL_CLOSED = 0.00;
    private static final double GATE_SMALL_OPEN   = 0.19;
    private static final double GATE_BIG_CLOSED   = 0.09;
    private static final double GATE_BIG_OPEN     = 0.00;

    private static final double PITCH_INIT = 0.5;
    private static final double PITCH_STEP = 0.02;
    private static final double PITCH_MIN  = 0.2;
    private static final double PITCH_MAX  = 0.8;
    private static final boolean PITCH_MIRROR = true;
    private static final boolean PITCH_LEFT_DECREASES = true;

    /* ==================== ★ Pinpoint 自瞄方向修正 ==================== */
    /**
     * 只影响【自瞄】使用的 heading，不影响无头驱动、不影响位置坐标、不影响 headingOffset。
     *
     * ★ 现在 Pinpoint 位置/朝向都实时更新，实测应为 0。
     *   之前填 -π/2 是为了弥补位置不更新带来的错误，现在位置修好了，这个 90° 反而把车扭歪了 90°。
     *
     * 若以后换了场地/机器人的特殊需求，可在此微调：
     *   物理车头停的位置   | 改成
     *   恰好对准目标       | 0        ← 推荐，当前状态
     *   比目标偏左 90°     | +Math.PI / 2
     *   比目标偏右 90°     | -Math.PI / 2
     *   180° 反向          | Math.PI
     */
    private static final double AIM_PINPOINT_HEADING_FIX_RAD = 0.0;

    /* ==================== 自瞄参数 ==================== */
    private static final double AIM_TARGET_X = 72.0;
    private static final double AIM_TARGET_Y = -72.0;

    private static final double AIM_KP = 1.7;
    private static final double AIM_KI = 0.1;
    private static final double AIM_KD = 0.1;
    private static final double AIM_MAX_TURN      = 0.8;
    private static final double AIM_TOLERANCE_RAD = Math.toRadians(2.0);

    private static final double AIM_I_MAX    = 1.0;
    private static final double AIM_I_ZONE   = Math.toRadians(30.0);
    private static final double AIM_D_FILTER = 0.7;

    /* ==================== Pinpoint ==================== */
    private PinpointXYReader pinpoint = null;

    private static final double PINPOINT_START_RETRY_SEC = 1.0;

    private static final double FIELD_START_X = PinPointConfig.CENTER_FROM_REAR_EDGE_IN;
    private static final double FIELD_START_Y = PinPointConfig.CENTER_FROM_LEFT_EDGE_IN;
    private static final double FIELD_START_HEADING = 0.0;

    private boolean pinpointStarted    = false;
    private boolean pinpointAnchored   = false;
    private double  nextPinpointStartAttempt = 0.0;

    private double  headingOffset = 0.0;

    /* ==================== 发射时序 ==================== */
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
    private boolean prevY     = false;
    private boolean openAim   = false;

    private double targetRPM   = RPM_INIT;
    private double filteredRPM = 0;
    private double lastError   = 0;
    private double lastTime    = 0;

    private double pitchPos = PITCH_INIT;

    private boolean fireActive = false;
    private double  fireStartTime = 0;
    private boolean shovel_model = true;
    private boolean was_press_a = false;

    /* ==================== 自瞄 PID 状态 ==================== */
    private double  aimIntegral   = 0.0;
    private double  aimPrevError  = 0.0;
    private boolean aimPrevValid  = false;
    private double  aimDFiltered  = 0.0;
    private double  aimLastTurn   = 0.0;
    private double  aimLastError  = 0.0;

    /* ==================== 初始化 ==================== */
    @Override
    public void init() {
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

        shooter0 = hardwareMap.get(DcMotorEx.class, "00");
        shooter1 = hardwareMap.get(DcMotorEx.class, "01");
        shooter0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter0.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor   = hardwareMap.get(DcMotorEx.class, "02");
        transferMotor = hardwareMap.get(DcMotorEx.class, "03");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        smallGateServo = hardwareMap.get(Servo.class, "s01");
        bigGateServo   = hardwareMap.get(Servo.class, "s1");
        smallGateServo.setPosition(GATE_SMALL_CLOSED);
        bigGateServo.setPosition(GATE_BIG_CLOSED);

        smallPitchServo1 = hardwareMap.get(Servo.class, "s2");
        smallPitchServo2 = hardwareMap.get(Servo.class, "s3");
        pitchPos = PITCH_INIT;
        applyPitch();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        shovel_lr = hardwareMap.get(Servo.class, "s00");

        pinpoint = new PinpointXYReader(hardwareMap);

        telemetry.addLine("底盘 + Shooter + Intake + 闸门 + 俯仰 已初始化");
        telemetry.addLine("等待 Pinpoint 校准完成再按 START");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (pinpoint != null && !pinpointStarted) {
            if (getRuntime() >= nextPinpointStartAttempt) {
                pinpointStarted = pinpoint.start();
                nextPinpointStartAttempt = getRuntime() + PINPOINT_START_RETRY_SEC;
            }
        }
        if (pinpoint != null) pinpoint.update();

        telemetry.addData("Pinpoint 启动", pinpointStarted ? "✅ 成功" : "⏳ 重试中");
        if (pinpoint != null) {
            telemetry.addData("Pinpoint 状态", pinpoint.getStatus());
            telemetry.addData("固件 / 坏读", "V%d / %d",
                    pinpoint.getFirmwareVersion(), pinpoint.getBadReadCount());
        }
        telemetry.addLine("START 后按默认起始坐标锚定");
        telemetry.update();
    }

    @Override
    public void start() {
        if (pinpoint != null && !pinpointStarted) {
            pinpointStarted = pinpoint.start();
        }
        if (pinpoint != null) pinpoint.update();
        imu.resetYaw();
        anchorPinpoint();
        lastTime = getRuntime();
        lastError = 0;
        resetAimPID();
    }

    /* ==================== 主循环 ==================== */
    @Override
    public void loop() {
        double now = getRuntime();
        double dt = now - lastTime;
        if (dt < 1e-4) dt = 1e-4;
        lastTime = now;

        if (pinpoint != null && !pinpointStarted) {
            if (now >= nextPinpointStartAttempt) {
                pinpointStarted = pinpoint.start();
                nextPinpointStartAttempt = now + PINPOINT_START_RETRY_SEC;
            }
        }
        if (pinpoint != null) pinpoint.update();

        if (pinpoint != null && pinpointStarted && !pinpointAnchored
                && !pinpoint.needsReanchor()) {
            anchorPinpoint();
        }

        // ============ 0. 朝向（无头驱动用）============
        double rawYaw = readHubYaw();
        double controlHeading = normalizeAngle(rawYaw + headingOffset);

        boolean aimReady = pinpoint != null && pinpointStarted && pinpointAnchored
                && pinpoint.isPositionValid() && !pinpoint.needsReanchor();

        openAim = gamepad1.x && deadzone(gamepad1.right_stick_x) == 0 && aimReady;

        // ============ 1. 底盘 ============
        double drive  = deadzone(-gamepad1.left_stick_y);
        double strafe = deadzone( gamepad1.left_stick_x);
        double turn   = deadzone( gamepad1.right_stick_x);

        // ★ 自瞄：FIX = 0 → aimHeading 就是真实物理车头朝向
        if (openAim) {
            double aimHeading = normalizeAngle(controlHeading + AIM_PINPOINT_HEADING_FIX_RAD);
            turn = computeAimTurn(aimHeading, dt);
            aimLastTurn = turn;
        } else {
            resetAimPID();
        }

        // 无头驱动
        double cos = Math.cos(controlHeading);
        double sin = Math.sin(controlHeading);

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

        shooter0.setPower(power);
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

        if (rb && !prevRB) {
            fireActive = true;
            fireStartTime = now;
        }
        prevRB = rb;

        double seriesPower = 0;
        boolean smallGateOpen;
        boolean bigGateOpen;

        if (fireActive) {
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
            smallGateOpen = lb;
            bigGateOpen   = lb;
            if (rtPressed) {
                seriesPower = rt;
            }
        }

        smallGateServo.setPosition(smallGateOpen ? GATE_SMALL_OPEN : GATE_SMALL_CLOSED);
        bigGateServo.setPosition(bigGateOpen   ? GATE_BIG_OPEN   : GATE_BIG_CLOSED);

        intakeMotor.setPower(seriesPower);
        transferMotor.setPower(seriesPower);

        // ============ 6. START 重锚定 ============
        if (gamepad1.start && !prevStart) {
            imu.resetYaw();
            anchorPinpoint();
        }
        prevStart = gamepad1.start;

        if (gamepad1.y && !prevY) {
            imu.resetYaw();
            anchorPinpoint();
        }
        prevY = gamepad1.y;

        // 7. 铲球
        if (gamepad1.a && !was_press_a) {
            shovel_model = !shovel_model;
        }
        was_press_a = gamepad1.a;
        if (shovel_model) {
            shovel_lr.setPosition(0);
        } else {
            shovel_lr.setPosition(0.48);
        }

        // ============ 8. 遥测 ============
        telemetry.addData("IMU raw yaw (°)", Math.toDegrees(rawYaw));
        telemetry.addData("controlHeading (°)", Math.toDegrees(controlHeading));
        telemetry.addData("自瞄修正 (°)", Math.toDegrees(AIM_PINPOINT_HEADING_FIX_RAD));

        telemetry.addData("Shooter 目标/实际", "%.0f / %.0f RPM", targetRPM, filteredRPM);
        telemetry.addData("俯仰位置", "%.2f", pitchPos);
        telemetry.addData("RT (吸球)", "%.2f", rt);
        telemetry.addData("LB (两个闸门)", lb ? "按住" : "松开");
        telemetry.addData("RB (发射)", fireActive ? "发射中" : "空闲");
        telemetry.addData("闸门 s01 / s1",
                (smallGateOpen ? "开" : "关") + " / " + (bigGateOpen ? "开" : "关"));
        telemetry.addData("Intake/通道 功率", "%.2f", seriesPower);

        telemetry.addData("自瞄", openAim ? "开" : "关");
        if (pinpoint != null) {
            telemetry.addData("Pinpoint", pinpoint.getStatus());
            telemetry.addData("  启动 / 锚定", "%s / %s",
                    pinpointStarted ? "✅" : "❌",
                    pinpointAnchored ? "✅" : "❌");
            telemetry.addData("  位置有效", pinpoint.isPositionValid());
            telemetry.addData("  需要重锚", pinpoint.needsReanchor());

            double px = pinpoint.getXInches();
            double py = pinpoint.getYInches();
            double targetAngleDeg = Math.toDegrees(
                    Math.atan2(AIM_TARGET_Y - py, AIM_TARGET_X - px));
            double aimHeading = normalizeAngle(controlHeading + AIM_PINPOINT_HEADING_FIX_RAD);
            double errorDeg = Math.toDegrees(normalizeAngle(
                    Math.atan2(AIM_TARGET_Y - py, AIM_TARGET_X - px) - aimHeading));

            telemetry.addData("  机器人位置", "%.1f, %.1f", px, py);
            telemetry.addData("  目标坐标", "%.1f, %.1f", AIM_TARGET_X, AIM_TARGET_Y);
            telemetry.addData("  目标方位°", "%.1f", targetAngleDeg);
            telemetry.addData("  自瞄 heading°", "%.1f", Math.toDegrees(aimHeading));
            telemetry.addData("  角度误差°", "%.1f", errorDeg);
            telemetry.addData("  PID turn", "%.3f", aimLastTurn);
        }

        telemetry.update();
    }

    /* ==================== 工具 ==================== */
    private void applyPitch() {
        smallPitchServo1.setPosition(pitchPos);
        smallPitchServo2.setPosition(PITCH_MIRROR ? 1.0 - pitchPos : pitchPos);
    }

    private void anchorPinpoint() {
        if (pinpoint == null || !pinpointStarted) {
            pinpointAnchored = false;
            return;
        }
        pinpoint.update();
        boolean ok = pinpoint.setFieldOrigin(
                FIELD_START_X, FIELD_START_Y, FIELD_START_HEADING);
        if (ok) {
            pinpointAnchored = true;
            headingOffset = normalizeAngle(pinpoint.getHeading() - readHubYaw());
        } else {
            pinpointAnchored = false;
        }
    }

    private double readHubYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private double computeAimTurn(double currentHeadingRad, double dt) {
        if (pinpoint == null) return 0.0;

        double dx = AIM_TARGET_X - pinpoint.getXInches();
        double dy = AIM_TARGET_Y - pinpoint.getYInches();
        double targetAngle = Math.atan2(dy, dx);

        double error = normalizeAngle(targetAngle - currentHeadingRad);
        aimLastError = error;

        if (Math.abs(error) < AIM_TOLERANCE_RAD) {
            aimIntegral  = 0.0;
            aimPrevError = error;
            aimPrevValid = true;
            aimDFiltered = 0.0;
            return 0.0;
        }

        if (Math.abs(error) < AIM_I_ZONE) {
            aimIntegral += error * dt;
            aimIntegral = clamp(aimIntegral, -AIM_I_MAX, AIM_I_MAX);
        } else {
            aimIntegral = 0.0;
        }

        double dError = 0.0;
        if (aimPrevValid && dt > 1e-4) {
            dError = normalizeAngle(error - aimPrevError) / dt;
        }
        aimPrevError = error;
        aimPrevValid = true;

        aimDFiltered = AIM_D_FILTER * aimDFiltered + (1.0 - AIM_D_FILTER) * dError;

        double out = AIM_KP * error
                + AIM_KI * aimIntegral
                + AIM_KD * aimDFiltered;

        return -clamp(out, -AIM_MAX_TURN, AIM_MAX_TURN);
    }

    private void resetAimPID() {
        aimIntegral  = 0.0;
        aimPrevError = 0.0;
        aimPrevValid = false;
        aimDFiltered = 0.0;
        aimLastTurn  = 0.0;
        aimLastError = 0.0;
    }

    private static double normalizeAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double deadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0.0 : v;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}