package org.firstinspires.ftc.teamcode.teleop;

import android.content.SharedPreferences;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pinpoint.PinpointXYReader;
import org.firstinspires.ftc.teamcode.pinpoint.PinPointConfig;

/**
 * 手柄1 驾驶 + 发射；手柄2 实时校准
 *
 * ★ 航向锁定（Heading Lock）★
 *   推左摇杆前进/横移但不推右摇杆时自动保持车头朝向，抵消机械漂移。
 *
 *   关键：用 IMU 的 yaw 角速度判断车是否在旋转
 *   - 车在旋转（用户推右摇杆 / 惯性）→ 目标实时跟随，PID 不干扰
 *   - 车真正停稳 → 目标冻结，PID 开始保持
 *   这样就不会出现"转向结束后被往回拉"的问题。
 */
@TeleOp(name = "Driver + Tuner", group = "Competition")
public class Teleop2 extends OpMode {

    /* ==================== 硬件 ==================== */
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private IMU imu;
    private DcMotorEx shooter0, shooter1;
    private DcMotorEx intakeMotor, transferMotor;
    private Servo smallGateServo, bigGateServo;
    private Servo smallPitchServo1, smallPitchServo2;
    private Servo bigPitchServo;
    private Servo shovel_lr;

    /* ==================== 通用参数 ==================== */
    private static final double STICK_DEADZONE   = 0.05;
    private static final double TRIGGER_DEADZONE = 0.01;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MIN_RPM = 0.0, MAX_RPM = 6000.0;

    private double kP = 0.00050, kD = 0.0000, kF = 0.00020;
    private static final double MIN_POWER = -0.1, MAX_POWER = 1.0;

    /* ==================== 闸门 / 俯仰 ==================== */
    private static final double GATE_SMALL_CLOSED = 0.00;
    private static final double GATE_SMALL_OPEN   = 0.19;
    private static final double GATE_BIG_CLOSED   = 0.09;
    private static final double GATE_BIG_OPEN     = 0.00;
    private static final double PITCH_MIN = 0.0, PITCH_MAX = 1.0;
    private static final boolean PITCH_MIRROR = true;

    /* ==================== 发射参数 ==================== */
    private static final double RT_RELEASE_THRESHOLD = 0.50;
    private static final double RT_FIRE_THRESHOLD    = 0.85;

    private static final double RPM_ENTER_ERR      = 100.0;
    private static final double RPM_EXIT_ERR       = 200.0;
    private static final double ENTER_STABLE_TIME  = 0.10;

    /* ==================== LT 反转中转层功率 ==================== */
    private static final double TRANSFER_REVERSE_POWER = -0.60;

    /* ==================== ★ 航向锁定参数 ==================== */
    private static final double HEADING_LOCK_KP        = 1.5;
    private static final double HEADING_LOCK_MAX_TURN  = 0.35;
    private static final double HEADING_LOCK_DEADBAND  = Math.toRadians(1.0);

    // ★ 车头 yaw 角速度超过这个值就认为"车在旋转"，heading lock 暂停
    //   用户转向的 yawRate 通常 >1 rad/s；机械漂移 <0.1 rad/s
    //   0.30 rad/s ≈ 17°/s，能可靠区分"转向中" vs "静止中"
    private static final double HEADING_YAW_RATE_THRESHOLD = 0.30;

    /* ==================== 驾驶员微调步进 ==================== */
    private static final double DRIVER_RPM_STEP   = 25.0;
    private static final double DRIVER_PITCH_STEP = 0.01;

    /* ==================== 校准手柄步进 ==================== */
    private static final double TUNER_RPM_STEP         = 5.0;
    private static final double TUNER_SMALL_PITCH_STEP = 0.005;
    private static final double TUNER_BIG_PITCH_STEP   = 0.005;

    /* ==================== 半场判断 ==================== */
    private static final double FIELD_HALF = 72.0;

    /* ==================== Pinpoint ==================== */
    private PinpointXYReader pinpoint = null;
    private static final double PINPOINT_START_RETRY_SEC = 1.0;
    private static final double FIELD_START_X = PinPointConfig.CENTER_FROM_REAR_EDGE_IN;
    private static final double FIELD_START_Y = PinPointConfig.CENTER_FROM_LEFT_EDGE_IN;
    private static final double FIELD_START_HEADING = 0.0;

    private boolean pinpointStarted  = false;
    private boolean pinpointAnchored = false;
    private double  nextPinpointStartAttempt = 0.0;
    private double  headingOffset = 0.0;

    /* ==================== 校准数据 ==================== */
    private double calRpmRed        = 3700.0, calRpmBlue        = 3700.0;
    private double calSmallPitchRed = 0.47,   calSmallPitchBlue = 0.47;
    private double calBigPitchRed   = 0.15,   calBigPitchBlue   = 0.15;

    private static final String PREFS_NAME = "DecodeHiveCalibration";
    private static final String K_RPM_RED   = "rpm_red";
    private static final String K_RPM_BLUE  = "rpm_blue";
    private static final String K_PS_RED    = "small_pitch_red";
    private static final String K_PS_BLUE   = "small_pitch_blue";
    private static final String K_PB_RED    = "big_pitch_red";
    private static final String K_PB_BLUE   = "big_pitch_blue";

    /* ==================== 驾驶员临时微调 ==================== */
    private double driverRpmOffset        = 0.0;
    private double driverSmallPitchOffset = 0.0;
    private double driverBigPitchOffset   = 0.0;

    /* ==================== 航向锁定状态 ==================== */
    private double  headingLockTarget = 0.0;
    private boolean headingLockActive = false;

    /* ==================== 状态 ==================== */
    private double targetRPM = 3700.0, filteredRPM = 0;
    private double lastShooterError = 0, lastTime = 0;
    private double smallPitchPos = 0.47, bigPitchPos = 0.15;

    private boolean rpmReadyLatched = false;
    private boolean firingActive    = false;
    private double  stableTimer     = 0.0;

    private boolean feederOn = false;

    private boolean prevStart, prevY1, prevA1, prevB1;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevRB1;

    private boolean prevA2, prevB2, prevX2, prevY2;
    private boolean prevUp2, prevDown2, prevLeft2, prevRight2;
    private boolean prevLB2, prevRB2, prevRT2;

    private boolean editingRed = true;
    private boolean shovelHigh = true;

    /* ==================== init ==================== */
    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "0");
        backLeft   = hardwareMap.get(DcMotorEx.class, "1");
        frontRight = hardwareMap.get(DcMotorEx.class, "2");
        backRight  = hardwareMap.get(DcMotorEx.class, "3");
        for (DcMotorEx m : new DcMotorEx[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
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
        bigPitchServo    = hardwareMap.get(Servo.class, "s0");
        shovel_lr        = hardwareMap.get(Servo.class, "s00");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        pinpoint = new PinpointXYReader(hardwareMap);
        loadCalibration();
        applyPitch(0.5, 0.5);

        telemetry.addLine("等待 Pinpoint / START 进入");
        telemetry.addLine("手柄1: 驾驶(航向锁定)+发射+LB吸球");
        telemetry.addLine("手柄2: 校准（A红 B蓝 X保存 Y重载 RT开feeder）");
        telemetry.update();
    }

    private void loadCalibration() {
        SharedPreferences p = hardwareMap.appContext.getSharedPreferences(PREFS_NAME, 0);
        calRpmRed        = p.getFloat(K_RPM_RED,  (float) calRpmRed);
        calRpmBlue       = p.getFloat(K_RPM_BLUE, (float) calRpmBlue);
        calSmallPitchRed = p.getFloat(K_PS_RED,   (float) calSmallPitchRed);
        calSmallPitchBlue= p.getFloat(K_PS_BLUE,  (float) calSmallPitchBlue);
        calBigPitchRed   = p.getFloat(K_PB_RED,   (float) calBigPitchRed);
        calBigPitchBlue  = p.getFloat(K_PB_BLUE,  (float) calBigPitchBlue);
    }

    private void saveCalibration() {
        SharedPreferences p = hardwareMap.appContext.getSharedPreferences(PREFS_NAME, 0);
        p.edit()
                .putFloat(K_RPM_RED,  (float) calRpmRed)
                .putFloat(K_RPM_BLUE, (float) calRpmBlue)
                .putFloat(K_PS_RED,   (float) calSmallPitchRed)
                .putFloat(K_PS_BLUE,  (float) calSmallPitchBlue)
                .putFloat(K_PB_RED,   (float) calBigPitchRed)
                .putFloat(K_PB_BLUE,  (float) calBigPitchBlue)
                .apply();
    }

    /* ==================== init_loop ==================== */
    @Override
    public void init_loop() {
        if (pinpoint != null && !pinpointStarted && getRuntime() >= nextPinpointStartAttempt) {
            pinpointStarted = pinpoint.start();
            nextPinpointStartAttempt = getRuntime() + PINPOINT_START_RETRY_SEC;
        }
        if (pinpoint != null) pinpoint.update();
        telemetry.addData("Pinpoint 启动", pinpointStarted ? "✅" : "⏳ 重试中");
        if (pinpoint != null) telemetry.addData("状态", pinpoint.getStatus());
        telemetry.update();
    }

    /* ==================== start ==================== */
    @Override
    public void start() {
        if (pinpoint != null && !pinpointStarted) pinpointStarted = pinpoint.start();
        if (pinpoint != null) pinpoint.update();
        imu.resetYaw();
        anchorPinpoint();
        lastTime = getRuntime();
        loadCalibration();
        driverRpmOffset = 0;
        driverSmallPitchOffset = 0;
        driverBigPitchOffset = 0;
        headingLockActive = false;
        resetFiringState();
    }

    private void resetFiringState() {
        rpmReadyLatched = false;
        firingActive    = false;
        stableTimer     = 0;
    }

    /* ==================== loop ==================== */
    @Override
    public void loop() {
        double now = getRuntime();
        double dt = Math.max(now - lastTime, 1e-4);
        lastTime = now;

        /* ---------- 0. Pinpoint 更新 ---------- */
        if (pinpoint != null && !pinpointStarted && now >= nextPinpointStartAttempt) {
            pinpointStarted = pinpoint.start();
            nextPinpointStartAttempt = now + PINPOINT_START_RETRY_SEC;
        }
        if (pinpoint != null) pinpoint.update();
        if (pinpoint != null && pinpointStarted && !pinpointAnchored && !pinpoint.needsReanchor()) {
            anchorPinpoint();
        }

        double rawYaw = readHubYaw();
        double controlHeading = normalizeAngle(rawYaw + headingOffset);

        double px = pinpoint != null ? pinpoint.getXInches() : FIELD_START_X;
        double py = pinpoint != null ? pinpoint.getYInches() : FIELD_START_Y;

        /* ---------- 1. 手柄2 校准操作（即时生效） ---------- */
        updateTunerFromGamepad2();

        /* ---------- 2. 半场判断 → 基础 cal 参数 ---------- */
        boolean onRedSide = (py < FIELD_HALF);
        double baseRpm        = onRedSide ? calRpmRed        : calRpmBlue;
        double baseSmallPitch = onRedSide ? calSmallPitchRed : calSmallPitchBlue;
        double baseBigPitch   = onRedSide ? calBigPitchRed   : calBigPitchBlue;

        /* ---------- 3. 手柄1 微调（改 offset，不改 cal） ---------- */
        if (gamepad1.dpad_up && !prevDpadUp)         driverRpmOffset        += DRIVER_RPM_STEP;
        if (gamepad1.dpad_down && !prevDpadDown)     driverRpmOffset        -= DRIVER_RPM_STEP;
        if (gamepad1.dpad_left && !prevDpadLeft)     driverSmallPitchOffset -= DRIVER_PITCH_STEP;
        if (gamepad1.dpad_right && !prevDpadRight)   driverSmallPitchOffset += DRIVER_PITCH_STEP;
        if (gamepad1.right_bumper && !prevRB1)       driverBigPitchOffset   += DRIVER_PITCH_STEP;
        if (gamepad1.b && !prevB1)                   driverBigPitchOffset   -= DRIVER_PITCH_STEP;
        prevDpadUp    = gamepad1.dpad_up;
        prevDpadDown  = gamepad1.dpad_down;
        prevDpadLeft  = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevRB1       = gamepad1.right_bumper;
        prevB1        = gamepad1.b;

        targetRPM     = clamp(baseRpm        + driverRpmOffset,        MIN_RPM, MAX_RPM);
        smallPitchPos = clamp(baseSmallPitch + driverSmallPitchOffset, PITCH_MIN, PITCH_MAX);
        bigPitchPos   = clamp(baseBigPitch   + driverBigPitchOffset,   PITCH_MIN, PITCH_MAX);

        /* ---------- 4. 底盘 + ★ 航向锁定 ---------- */
        double drive  = deadzone(-gamepad1.left_stick_y);
        double strafe = deadzone( gamepad1.left_stick_x);
        double turn   = deadzone( gamepad1.right_stick_x);

        boolean turnInput = Math.abs(turn) > 0.0;
        boolean moveInput = Math.abs(drive) > 0.0 || Math.abs(strafe) > 0.0;

        // ★ 读取 yaw 角速度判断车是否在旋转
        double yawRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        boolean headingSpinning = Math.abs(yawRate) > HEADING_YAW_RATE_THRESHOLD;

        if (turnInput || headingSpinning) {
            // 车在旋转（用户推右摇杆 或 靠惯性转）：目标实时跟随，不激活 PID
            headingLockActive = false;
            headingLockTarget = controlHeading;
        } else if (moveInput) {
            // 车稳定 + 有移动：激活 heading lock，目标冻结
            headingLockActive = true;
            double hErr = normalizeAngle(headingLockTarget - controlHeading);
            if (Math.abs(hErr) > HEADING_LOCK_DEADBAND) {
                turn = -clamp(HEADING_LOCK_KP * hErr,
                        -HEADING_LOCK_MAX_TURN, HEADING_LOCK_MAX_TURN);
            } else {
                turn = 0.0;
            }
        } else {
            // 完全静止：解锁
            headingLockActive = false;
        }

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
        if (max > 1.0) { fl/=max; fr/=max; bl/=max; br/=max; }

        frontLeft.setPower(fl);  backLeft.setPower(bl);
        frontRight.setPower(fr); backRight.setPower(br);

        /* ---------- 5. Shooter PD ---------- */
        double rawRPM = Math.abs(shooter0.getVelocity()) * 60.0 / TICKS_PER_REV;
        filteredRPM += 0.4 * (rawRPM - filteredRPM);
        double sErr  = targetRPM - filteredRPM;
        double sDErr = (sErr - lastShooterError) / dt;
        lastShooterError = sErr;
        double sPower = clamp(kF*targetRPM + kP*sErr + kD*sDErr, MIN_POWER, MAX_POWER);
        shooter0.setPower(sPower);
        shooter1.setPower(sPower);

        /* ---------- 6. 发射状态机 ---------- */
        double rpmAbsErr = Math.abs(sErr);

        if (rpmReadyLatched) {
            if (rpmAbsErr > RPM_EXIT_ERR) rpmReadyLatched = false;
        } else {
            if (rpmAbsErr < RPM_ENTER_ERR) rpmReadyLatched = true;
        }

        double rt = gamepad1.right_trigger;
        boolean rtFireRequest = rt >= RT_FIRE_THRESHOLD;
        boolean rtHeld        = rt >= RT_RELEASE_THRESHOLD;

        if (!firingActive) {
            boolean enterable = rtFireRequest && rpmReadyLatched;
            if (enterable) {
                stableTimer += dt;
                if (stableTimer >= ENTER_STABLE_TIME) {
                    firingActive = true;
                    stableTimer  = 0;
                }
            } else {
                stableTimer = 0;
            }
        } else {
            if (!rtHeld) {
                firingActive = false;
            }
        }

        /* ---------- 7. 应用闸门 + 供球 ---------- */
        smallGateServo.setPosition(firingActive ? GATE_SMALL_OPEN : GATE_SMALL_CLOSED);
        bigGateServo.setPosition(firingActive ? GATE_BIG_OPEN   : GATE_BIG_CLOSED);

        boolean lbHeld      = gamepad1.left_bumper;
        boolean ltReverse   = gamepad1.left_trigger > TRIGGER_DEADZONE;

        double intakePower = 0.0, transferPower = 0.0;
        if (firingActive) {
            intakePower   = 1.0;
            transferPower = 1.0;
        } else if (ltReverse) {
            intakePower   = 0.0;
            transferPower = TRANSFER_REVERSE_POWER;
        } else if (lbHeld) {
            intakePower   = 1.0;
            transferPower = 1.0;
        } else if (feederOn) {
            intakePower   = 1.0;
            transferPower = 1.0;
        }

        intakeMotor.setPower(intakePower);
        transferMotor.setPower(transferPower);

        /* ---------- 8. 俯仰舵机 ---------- */
        applyPitch(smallPitchPos, bigPitchPos);

        /* ---------- 9. START / Y / A ---------- */
        if (gamepad1.start && !prevStart) {
            imu.resetYaw();
            anchorPinpoint();
            driverRpmOffset = 0;
            driverSmallPitchOffset = 0;
            driverBigPitchOffset = 0;
            headingLockActive = false;
        }
        prevStart = gamepad1.start;

        if (gamepad1.y && !prevY1) { imu.resetYaw(); anchorPinpoint(); headingLockActive = false; }
        prevY1 = gamepad1.y;

        if (gamepad1.a && !prevA1) shovelHigh = !shovelHigh;
        prevA1 = gamepad1.a;
        shovel_lr.setPosition(shovelHigh ? 0 : 0.48);

        /* ---------- 10. 遥测 ---------- */
        telemetry.addData("半场", onRedSide ? "🔴 红" : "🔵 蓝");
        telemetry.addData("位置", "%.1f, %.1f", px, py);

        telemetry.addData("车头 heading°", "%.1f", Math.toDegrees(controlHeading));
        telemetry.addData("yawRate°/s", "%.1f", Math.toDegrees(yawRate));
        telemetry.addData("★ 航向锁定", headingLockActive
                ? String.format("✅ 目标 %.1f° (误差 %.1f°)",
                Math.toDegrees(headingLockTarget),
                Math.toDegrees(normalizeAngle(headingLockTarget - controlHeading)))
                : (headingSpinning ? "⏸ 车在转" : "❌ 解锁"));
        telemetry.addData("turn 输出", "%.2f", turn);

        telemetry.addData("RT", "%.2f", rt);
        telemetry.addData("LT 反转功率", "%.2f", TRANSFER_REVERSE_POWER);
        telemetry.addData("LB (吸球)", lbHeld ? "★ 按住 ★" : "松开");
        telemetry.addData("转速就绪锁存", rpmReadyLatched ? "✅" : "❌");
        telemetry.addData("发射中", firingActive ? "★ 是 ★" : "否");
        telemetry.addData("稳定计时", "%.2f / %.2f", stableTimer, ENTER_STABLE_TIME);

        telemetry.addData("RPM", "%.0f / %.0f", targetRPM, filteredRPM);
        telemetry.addData("小球俯仰", "%.3f", smallPitchPos);
        telemetry.addData("大球俯仰", "%.3f", bigPitchPos);
        telemetry.addData("Intake / Transfer", "%.2f / %.2f", intakePower, transferPower);
        telemetry.addData("Feeder (手柄2 RT)", feederOn ? "★ 开 ★" : "关");

        telemetry.addLine("──────────");
        telemetry.addData("手柄2 编辑", editingRed ? "🔴 红" : "🔵 蓝");
        telemetry.addData("红方 cal", "RPM %.0f | 小%.4f | 大%.4f",
                calRpmRed, calSmallPitchRed, calBigPitchRed);
        telemetry.addData("蓝方 cal", "RPM %.0f | 小%.4f | 大%.4f",
                calRpmBlue, calSmallPitchBlue, calBigPitchBlue);
        telemetry.addData("驾驶员临时偏移", "RPM %+.0f | 小%+.3f | 大%+.3f",
                driverRpmOffset, driverSmallPitchOffset, driverBigPitchOffset);

        telemetry.update();
    }

    /* ==================== 手柄2 校准（每帧调用） ==================== */
    private void updateTunerFromGamepad2() {
        if (gamepad2.a && !prevA2) editingRed = true;
        if (gamepad2.b && !prevB2) editingRed = false;
        prevA2 = gamepad2.a;
        prevB2 = gamepad2.b;

        boolean rt2 = gamepad2.right_trigger > 0.5;
        if (rt2 && !prevRT2) feederOn = !feederOn;
        prevRT2 = rt2;

        if (editingRed) {
            if (gamepad2.dpad_up    && !prevUp2)    calRpmRed        = clamp(calRpmRed        + TUNER_RPM_STEP,         0, 6000);
            if (gamepad2.dpad_down  && !prevDown2)  calRpmRed        = clamp(calRpmRed        - TUNER_RPM_STEP,         0, 6000);
            if (gamepad2.dpad_left  && !prevLeft2)  calSmallPitchRed = clamp(calSmallPitchRed - TUNER_SMALL_PITCH_STEP, PITCH_MIN, PITCH_MAX);
            if (gamepad2.dpad_right && !prevRight2) calSmallPitchRed = clamp(calSmallPitchRed + TUNER_SMALL_PITCH_STEP, PITCH_MIN, PITCH_MAX);
            if (gamepad2.left_bumper  && !prevLB2)  calBigPitchRed   = clamp(calBigPitchRed   - TUNER_BIG_PITCH_STEP,   PITCH_MIN, PITCH_MAX);
            if (gamepad2.right_bumper && !prevRB2)  calBigPitchRed   = clamp(calBigPitchRed   + TUNER_BIG_PITCH_STEP,   PITCH_MIN, PITCH_MAX);

        } else {
            if (gamepad2.dpad_up    && !prevUp2)    calRpmBlue        = clamp(calRpmBlue        + TUNER_RPM_STEP,         0, 6000);
            if (gamepad2.dpad_down  && !prevDown2)  calRpmBlue        = clamp(calRpmBlue        - TUNER_RPM_STEP,         0, 6000);
            if (gamepad2.dpad_left  && !prevLeft2)  calSmallPitchBlue = clamp(calSmallPitchBlue - TUNER_SMALL_PITCH_STEP, PITCH_MIN, PITCH_MAX);
            if (gamepad2.dpad_right && !prevRight2) calSmallPitchBlue = clamp(calSmallPitchBlue + TUNER_SMALL_PITCH_STEP, PITCH_MIN, PITCH_MAX);
            if (gamepad2.left_bumper  && !prevLB2)  calBigPitchBlue   = clamp(calBigPitchBlue   - TUNER_BIG_PITCH_STEP,   PITCH_MIN, PITCH_MAX);
            if (gamepad2.right_bumper && !prevRB2)  calBigPitchBlue   = clamp(calBigPitchBlue   + TUNER_BIG_PITCH_STEP,   PITCH_MIN, PITCH_MAX);
        }

        prevUp2    = gamepad2.dpad_up;
        prevDown2  = gamepad2.dpad_down;
        prevLeft2  = gamepad2.dpad_left;
        prevRight2 = gamepad2.dpad_right;
        prevLB2    = gamepad2.left_bumper;
        prevRB2    = gamepad2.right_bumper;

        if (gamepad2.x && !prevX2) saveCalibration();
        prevX2 = gamepad2.x;
        if (gamepad2.y && !prevY2) loadCalibration();
        prevY2 = gamepad2.y;
    }

    /* ==================== 工具 ==================== */
    private void applyPitch(double small, double big) {
        smallPitchServo1.setPosition(small);
        smallPitchServo2.setPosition(PITCH_MIRROR ? 1.0 - small : small);
        if (bigPitchServo != null) bigPitchServo.setPosition(big);
    }

    private void anchorPinpoint() {
        if (pinpoint == null || !pinpointStarted) { pinpointAnchored = false; return; }
        pinpoint.update();
        boolean ok = pinpoint.setFieldOrigin(FIELD_START_X, FIELD_START_Y, FIELD_START_HEADING);
        if (ok) {
            pinpointAnchored = true;
            headingOffset = normalizeAngle(pinpoint.getHeading() - readHubYaw());
        } else pinpointAnchored = false;
    }

    private double readHubYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private static double normalizeAngle(double a) {
        while (a >  Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
    private static double deadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0 : v;
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}