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
import org.firstinspires.ftc.teamcode.subsystems.table;

@TeleOp(name = "Aim + Driver + Tuner", group = "Competition")
public class Teleop01 extends OpMode {

    /* ==================== 硬件 ==================== */
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private IMU imu;
    private DcMotorEx shooter0, shooter1;
    private DcMotorEx intakeMotor, transferMotor;
    private Servo smallGateServo, bigGateServo;
    private Servo smallPitchServo1, smallPitchServo2, bigPitchServo;
    private Servo shovel_lr;

    /* ==================== 通用参数 ==================== */
    private static final double STICK_DEADZONE   = 0.05;
    private static final double TRIGGER_DEADZONE = 0.01;
    private static final double TICKS_PER_REV = 28.0;

    private double kP = 0.00199, kD = 0.00024, kF = 0.00020;
    //    private double kP = 0.00050, kD = 0.0000, kF = 0.00020;
    private static final double MIN_POWER = -0.1, MAX_POWER = 1.0;

    private static table Table = null;

    /* ==================== 闸门 / 俯仰 ==================== */
    private static final double GATE_SMALL_CLOSED = 0.00;
    private static final double GATE_SMALL_OPEN   = 0.19;
    private static final double GATE_BIG_CLOSED   = 0.09;
    private static final double GATE_BIG_OPEN     = 0.00;
    private static final double PITCH_MIN = 0.0, PITCH_MAX = 1.0;
    private static final boolean PITCH_MIRROR = true;

    /* ==================== 航向锁定 ==================== */
    private static final double HEADING_LOCK_KP        = 1.5;
    private static final double HEADING_LOCK_MAX_TURN  = 0.35;
    private static final double HEADING_LOCK_DEADBAND  = Math.toRadians(1.0);
    private static final double HEADING_YAW_RATE_THRESHOLD = 0.30;
    private double  headingLockTarget = 0.0;
    private boolean headingLockActive = false;

    /* ==================== 扳机阈值（RT 两段式） ==================== */
    private static final double RT_AIM_THRESHOLD     = 0.50;
    private static final double RT_RELEASE_THRESHOLD = 0.50;
    private static final double RT_FIRE_THRESHOLD    = 0.85;

    /* ==================== 瞄准点 ==================== */
    private static final double[][] AIM_POINTS = {
            {73.2, 48},
            {73.2, 144-48}
    };

    /* ==================== 自瞄 PID ==================== */
    private double AIM_KP = 1.56;
    private double AIM_KD = 0.0965;//0.11

    private static final double AIM_KI = 0;
    private static final double AIM_MAX_TURN      = 0.8;
    private static final double AIM_TOLERANCE_RAD = Math.toRadians(2.0);
    private static final double AIM_I_MAX   = 1.0;
    private static final double AIM_I_ZONE  = Math.toRadians(30.0);
    private static final double AIM_D_FILTER = 0.7;

    /* ==================== 发射参数 ==================== */
    private static final double RPM_ENTER_ERR      = 150.0;//100
    private static final double RPM_EXIT_ERR       = 200.0;
    private static final double ENTER_STABLE_TIME  = 0.10;

    /* ==================== 防抖参数 ==================== */
    private static final double AIM_ENTER_DEG       = 3.0;
    private static final double AIM_EXIT_DEG        = 8.0;
    private static final double FIRE_ABORT_DEG      = 15.0;
    private static final double FIRE_ABORT_TIME     = 0.30;

    private static final double TRANSFER_REVERSE_POWER = -0.35;

    /* ==================== gamepad2 扳机反转（一按即开） ==================== */
    private static final double GP2_REVERSE_POWER     = 0.8;
    private static final double GP2_TRIGGER_THRESHOLD = 0.5;
    private  final double preA = 0.5;

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

    //gamepad1
    private boolean prevStart, prevY1, prevA1, prevB1;
    // ★ gamepad1 dpad 边沿检测（Shooter PD 调参）
    private boolean prevUp1, prevDown1, prevLeft1, prevRight1;

    //gamepad2
    private boolean prevA2, prevB2, prevX2, prevY2;
    private boolean prevUp2, prevDown2, prevLeft2, prevRight2;
    private boolean prevLB2, prevRB2;

    // gamepad2 扳机反转状态（按住即反转）
    private boolean gp2LTReverseActive = false;
    private boolean gp2RTReverseActive = false;

    // ★ X 切换是否使用打表
    private boolean useTable = false;//false

    private static final double TUNER_RPM_STEP         = 5.0;
    private static final double TUNER_SMALL_PITCH_STEP = 0.005;
    private static final double TUNER_BIG_PITCH_STEP   = 0.005;

    // ★ gamepad1 dpad 调 Shooter kP/kD；gamepad2 A/B 调 kF
    private static final double TUNER_SHOOTER_PD_STEP = 0.00001;
    private static final double SHOOTER_KP_MIN = 0.0, SHOOTER_KP_MAX = 0.005;
    private static final double SHOOTER_KD_MIN = 0.0, SHOOTER_KD_MAX = 0.005;
    private static final double SHOOTER_KF_MIN = 0.0, SHOOTER_KF_MAX = 0.005;

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

    /* ==================== 状态 ==================== */
    private double targetRPM = 3700.0, filteredRPM = 0;
    private double lastShooterError = 0, lastTime = 0;
    private double smallPitchPos = 0.47, bigPitchPos = 0.15;

    // 自瞄 PID
    private double aimIntegral = 0, aimPrevError = 0, aimDFiltered = 0;
    private boolean aimPrevValid = false;
    private double lastAimErrorRad = 0.0;

    // 防抖状态
    private boolean aimAlignedLatched = false;
    private boolean rpmReadyLatched   = false;
    private boolean firingActive      = false;
    private double  stableTimer       = 0.0;
    private double  fireAbortTimer    = 0.0;
    private boolean shovelHigh = true;

    // 当前瞄准点
    private double currentTargetX = 0, currentTargetY = 0;
    private int currentTargetIdx = 0;

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
        bigPitchServo = hardwareMap.get(Servo.class, "s0");
        shovel_lr = hardwareMap.get(Servo.class, "s00");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        pinpoint = new PinpointXYReader(hardwareMap);
        loadCalibration();
        applyPitch(smallPitchPos, bigPitchPos);

        Table = new table();

        telemetry.addLine("等待 Pinpoint 启动 / START 进入");
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

    @Override
    public void start() {
        if (pinpoint != null && !pinpointStarted) pinpointStarted = pinpoint.start();
        if (pinpoint != null) pinpoint.update();
        imu.resetYaw();
        anchorPinpoint();
        lastTime = getRuntime();
        loadCalibration();
        applyPitch(smallPitchPos, bigPitchPos);
        headingLockActive = false;
        resetAimPID();
        resetFireStateMachine();
    }

    private void resetAimPID() {
        aimIntegral = 0; aimPrevError = 0; aimPrevValid = false; aimDFiltered = 0;
        lastAimErrorRad = 0;
    }

    private void resetFireStateMachine() {
        aimAlignedLatched = false;
        rpmReadyLatched   = false;
        firingActive      = false;
        stableTimer       = 0;
        fireAbortTimer    = 0;
    }

    /* ==================== loop ==================== */
    @Override
    public void loop() {
        double now = getRuntime();
        double dt = Math.max(now - lastTime, 1e-4);
        lastTime = now;

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

        // gamepad2 微调 + 扳机反转 + A/B 调 kF + X 切换 useTable
        updateTunerFromGamepad2();
        // ★ gamepad1 dpad 调 Shooter kP/kD
        updateShooterPdFromGamepad1();

        /* ============ 1. 选最近的瞄准点 ============ */
        double px = pinpoint != null ? pinpoint.getXInches() : FIELD_START_X;
        double py = pinpoint != null ? pinpoint.getYInches() : FIELD_START_Y;
        double bestDist = Double.MAX_VALUE;
        currentTargetX = AIM_POINTS[0][0];
        currentTargetY = AIM_POINTS[0][1];
        currentTargetIdx = 0;
        for (int i = 0; i < AIM_POINTS.length; i++) {
            double d = Math.hypot(px - AIM_POINTS[i][0], py - AIM_POINTS[i][1]);
            if (d < bestDist) {
                bestDist = d;
                currentTargetX = AIM_POINTS[i][0];
                currentTargetY = AIM_POINTS[i][1];
                currentTargetIdx = i;
            }
        }

        /* ============ 2. 参数来源：打表 或 硬切红/蓝 ============ */
        if (useTable) {
            // ★ 打表模式：高斯加权查表
            table.Result r = Table.weighted(px, py);
            if (r.valid) {
                currentTargetX = r.x;
                currentTargetY = r.y;
                targetRPM     = clamp(r.shooterRPM,  0, 6000);
                smallPitchPos = clamp(r.pitchSmall, PITCH_MIN, PITCH_MAX);
                bigPitchPos   = clamp(r.pitchBig,   PITCH_MIN, PITCH_MAX);
                bestDist      = r.bestDist;
            }
            // r.valid == false 时保持上一帧参数
        } else {
            // 原来的硬切红/蓝参数
            if (currentTargetIdx == 0) {
                targetRPM = calRpmRed;
                smallPitchPos = calSmallPitchRed;
                bigPitchPos   = calBigPitchRed;
            } else {
                targetRPM = calRpmBlue;
                smallPitchPos = calSmallPitchBlue;
                bigPitchPos   = calBigPitchBlue;
            }
        }

        /* ============ 3. 底盘 + 自瞄/航向锁定 ============ */
        double drive  = deadzone(-gamepad1.left_stick_y);
        double strafe = deadzone( gamepad1.left_stick_x);
        double turn   = deadzone( gamepad1.right_stick_x);

        boolean rtAiming = gamepad1.right_trigger > RT_AIM_THRESHOLD;
        boolean aimActive = rtAiming && pinpoint != null && pinpointAnchored
                && pinpoint.isPositionValid() && !pinpoint.needsReanchor();

        if (aimActive) {
            // ===== 自瞄接管 turn，同时禁用 heading lock =====
            headingLockActive = false;
            double aimHeading = normalizeAngle(controlHeading + Math.PI);
            turn = computeAimTurn(currentTargetX, currentTargetY, aimHeading, dt);
        } else {
            // ===== 原 heading lock 逻辑 =====
            resetAimPID();

            boolean turnInput = Math.abs(turn) > 0.0;
            boolean moveInput = Math.abs(drive) > 0.0 || Math.abs(strafe) > 0.0;

            double yawRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
            boolean headingSpinning = Math.abs(yawRate) > HEADING_YAW_RATE_THRESHOLD;

            if (turnInput || headingSpinning) {
                headingLockActive = false;
                headingLockTarget = controlHeading;
            } else if (moveInput) {
                headingLockActive = true;
                double hErr = normalizeAngle(headingLockTarget - controlHeading);
                if (Math.abs(hErr) > HEADING_LOCK_DEADBAND) {
                    turn = -clamp(HEADING_LOCK_KP * hErr,
                            -HEADING_LOCK_MAX_TURN, HEADING_LOCK_MAX_TURN);
                } else {
                    turn = 0.0;
                }
            } else {
                headingLockActive = false;
            }
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

        /* ============ 4. Shooter PD ============ */
        double rawRPM = Math.abs(shooter0.getVelocity()) * 60.0 / TICKS_PER_REV;
        filteredRPM += 0.4 * (rawRPM - filteredRPM);
        double sErr = targetRPM - filteredRPM;
        double sDErr = (sErr - lastShooterError) / dt;
        lastShooterError = sErr;
        double sPower = clamp(kF*targetRPM + kP*sErr + kD*sDErr, MIN_POWER, MAX_POWER);
        //电量补偿
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double compensatedPower = sPower * (12.0 / batteryVoltage);
        compensatedPower = clamp(compensatedPower, -1.0, 1.0); // 安全限幅

        shooter0.setPower(compensatedPower);
        shooter1.setPower(compensatedPower);

        /* ============ 6. 发射锁存状态机（RT>=0.85 射击） ============ */
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
            boolean enterable = rtFireRequest && rpmReadyLatched && lastAimErrorRad<0.07;//shoterror  && lastAimErrorRad<0.07
            if (enterable || (gamepad1.right_bumper && rpmReadyLatched)) { //(gamepad1.right_bumper && rpmReadyLatched)
                stableTimer += dt;
                if (stableTimer >= ENTER_STABLE_TIME || (gamepad1.right_bumper && rpmReadyLatched)) {
                    firingActive = true;
                    stableTimer  = 0;
                }
            } else {
                stableTimer = 0;
            }
        } else {
            if (!rtHeld && !gamepad1.right_bumper) {
                firingActive = false;
            }
        }

        /* ---------- 7. 闸门 + 供球 ---------- */
        smallGateServo.setPosition(firingActive ? GATE_SMALL_OPEN : GATE_SMALL_CLOSED);
        bigGateServo.setPosition(firingActive ? GATE_BIG_OPEN   : GATE_BIG_CLOSED);

        boolean lbHeld    = gamepad1.left_bumper;
        boolean ltReverse = gamepad1.left_trigger > TRIGGER_DEADZONE;

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
        }

        if (gp2LTReverseActive) intakePower   = -GP2_REVERSE_POWER;
        if (gp2RTReverseActive) transferPower = -GP2_REVERSE_POWER;

        intakeMotor.setPower(intakePower);
        transferMotor.setPower(transferPower);

        applyPitch(smallPitchPos, bigPitchPos);

        /* ============ START / Y 重锚定 ============ */
        if (gamepad1.start && !prevStart) {
            imu.resetYaw();
            anchorPinpoint();
            headingLockActive = false;
        }
        prevStart = gamepad1.start;

        if (gamepad1.y && !prevY1) {
            imu.resetYaw();
            anchorPinpoint();
            headingLockActive = false;
        }
        prevY1 = gamepad1.y;

        if (gamepad1.a && !prevA1) shovelHigh = !shovelHigh;
        prevA1 = gamepad1.a;
        shovel_lr.setPosition(shovelHigh ? 0 : 0.48);

        /* ============ 遥测 ============ */
        telemetry.addData("瞄准点 #", "%d (%.1f, %.1f)", currentTargetIdx, currentTargetX, currentTargetY);
        telemetry.addData("位置", "%.1f, %.1f", px, py);
        telemetry.addData("到目标距离", "%.1f in", bestDist);
        telemetry.addData("Table 模式", useTable ? "★ 开 ★ (X 切换)" : "关 (X 切换)");

        telemetry.addData("RT", "%.2f", rt);
        telemetry.addData("自瞄(RT>0.5)", aimActive ? "★ 开 ★" : "关");
        telemetry.addData("航向锁定", headingLockActive ? "🔒 锁定" : "未激活");
        telemetry.addData("转速就绪锁存", rpmReadyLatched ? "✅" : "❌");
        telemetry.addData("发射中(RT>=0.85)", firingActive ? "★ 是 ★" : "否");
        telemetry.addData("稳定计时", "%.2f / %.2f", stableTimer, ENTER_STABLE_TIME);

        telemetry.addData("RPM", "%.0f / %.0f", targetRPM, filteredRPM);
        telemetry.addData("小球俯仰", "%.3f", smallPitchPos);
        telemetry.addData("大球俯仰", "%.3f", bigPitchPos);
        telemetry.addData("Intake / Transfer", "%.2f / %.2f", intakePower, transferPower);
        telemetry.addData("闸门", firingActive ? "开/开" : "关/关");
        telemetry.addData("GP2 反转", "LT:%s RT:%s",
                gp2LTReverseActive ? "是" : "否",
                gp2RTReverseActive ? "是" : "否");
        telemetry.addData("Aim PID", "KP=%.6f  KD=%.6f", AIM_KP, AIM_KD);
        telemetry.addData("Shooter PD", "kP=%.5f  kD=%.5f  kF=%.5f", kP, kD, kF);
        telemetry.addData("Shooter 调参", "dpad↑↓ kP  dpad←→ kD  A/B kF");

        telemetry.update();
    }

    /* ==================== gamepad2 微调 ==================== */
    private void updateTunerFromGamepad2() {
        gp2LTReverseActive = gamepad2.left_trigger  > GP2_TRIGGER_THRESHOLD;
        gp2RTReverseActive = gamepad2.right_trigger > GP2_TRIGGER_THRESHOLD;

        if (gamepad2.dpad_up    && !prevUp2)    calRpmRed        = clamp(calRpmRed        + TUNER_RPM_STEP,         0, 6000);
        if (gamepad2.dpad_down  && !prevDown2)  calRpmRed        = clamp(calRpmRed        - TUNER_RPM_STEP,         0, 6000);
        if (gamepad2.dpad_left  && !prevLeft2)  calSmallPitchRed = clamp(calSmallPitchRed - TUNER_SMALL_PITCH_STEP, PITCH_MIN, PITCH_MAX);
        if (gamepad2.dpad_right && !prevRight2) calSmallPitchRed = clamp(calSmallPitchRed + TUNER_SMALL_PITCH_STEP, PITCH_MIN, PITCH_MAX);
        if (gamepad2.left_bumper  && !prevLB2)  calBigPitchRed   = clamp(calBigPitchRed   - TUNER_BIG_PITCH_STEP,   PITCH_MIN, PITCH_MAX);
        if (gamepad2.right_bumper && !prevRB2)  calBigPitchRed   = clamp(calBigPitchRed   + TUNER_BIG_PITCH_STEP,   PITCH_MIN, PITCH_MAX);
        if (gamepad2.y) imu.resetYaw();

        // ★ A 加 kF，B 减 kF，边沿触发
        if (gamepad2.a && !prevA2) {
            kF = clamp(kF + TUNER_SHOOTER_PD_STEP, SHOOTER_KF_MIN, SHOOTER_KF_MAX);
        }
        if (gamepad2.b && !prevB2) {
            kF = clamp(kF - TUNER_SHOOTER_PD_STEP, SHOOTER_KF_MIN, SHOOTER_KF_MAX);
        }

        // ★ X 切换是否使用打表
        if (gamepad2.x && !prevX2) useTable = !useTable;

        prevUp2    = gamepad2.dpad_up;
        prevDown2  = gamepad2.dpad_down;
        prevLeft2  = gamepad2.dpad_left;
        prevRight2 = gamepad2.dpad_right;
        prevLB2    = gamepad2.left_bumper;
        prevRB2    = gamepad2.right_bumper;
        prevA2     = gamepad2.a;
        prevB2     = gamepad2.b;
        prevX2     = gamepad2.x;
    }

    /* ==================== ★ gamepad1 dpad 调 Shooter kP/kD ==================== */
    private void updateShooterPdFromGamepad1() {
        // 上/下：kP
        if (gamepad1.dpad_up    && !prevUp1) {
            AIM_KP = clamp(AIM_KP + 0.05, SHOOTER_KP_MIN, 3);

        }
        if (gamepad1.dpad_down  && !prevDown1) {
            AIM_KP = clamp(AIM_KP - 0.05, SHOOTER_KP_MIN, 3);
        }
        // 右/左：kD
        if (gamepad1.dpad_right && !prevRight1) {
            AIM_KD = clamp(AIM_KD + 0.005, SHOOTER_KD_MIN, 1);
        }
        if (gamepad1.dpad_left  && !prevLeft1) {
            AIM_KD = clamp(AIM_KD - 0.005, SHOOTER_KD_MIN, 1);
        }

//        if (gamepad1.dpad_up    && !prevUp1) {
//            kP = clamp(kP + TUNER_SHOOTER_PD_STEP, SHOOTER_KP_MIN, SHOOTER_KP_MAX);
//
//        }
//        if (gamepad1.dpad_down  && !prevDown1) {
//            kP = clamp(kP - TUNER_SHOOTER_PD_STEP, SHOOTER_KP_MIN, SHOOTER_KP_MAX);
//        }
//        // 右/左：kD
//        if (gamepad1.dpad_right && !prevRight1) {
//            kD = clamp(kD + TUNER_SHOOTER_PD_STEP, SHOOTER_KD_MIN, SHOOTER_KD_MAX);
//        }
//        if (gamepad1.dpad_left  && !prevLeft1) {
//            kD = clamp(kD - TUNER_SHOOTER_PD_STEP, SHOOTER_KD_MIN, SHOOTER_KD_MAX);
//        }

        prevUp1    = gamepad1.dpad_up;
        prevDown1  = gamepad1.dpad_down;
        prevLeft1  = gamepad1.dpad_left;
        prevRight1 = gamepad1.dpad_right;
    }

    /* ==================== 工具 ==================== */
    private void applyPitch(double small, double big) {
        smallPitchServo1.setPosition(small);
        smallPitchServo2.setPosition(PITCH_MIRROR ? 1.0 - small : small);
        bigPitchServo.setPosition(big);}

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

    private double computeAimTurn(double tx, double ty, double heading, double dt) {
        if (pinpoint == null) { lastAimErrorRad = 0; return 0; }
        double dx = tx - pinpoint.getXInches();
        double dy = ty - pinpoint.getYInches();
        double targetAngle = Math.atan2(dy, dx);
        double error = normalizeAngle(targetAngle - heading);
        lastAimErrorRad = error;

        if (Math.abs(error) < AIM_TOLERANCE_RAD) {
            aimIntegral = 0; aimPrevError = error; aimPrevValid = true; aimDFiltered = 0;
            return 0.0;
        }
        if (Math.abs(error) < AIM_I_ZONE) {
            aimIntegral += error * dt;
            aimIntegral = clamp(aimIntegral, -AIM_I_MAX, AIM_I_MAX);
        } else aimIntegral = 0;

        double dErr = 0;
        if (aimPrevValid && dt > 1e-4) dErr = normalizeAngle(error - aimPrevError) / dt;
        aimPrevError = error; aimPrevValid = true;
        aimDFiltered = AIM_D_FILTER*aimDFiltered + (1-AIM_D_FILTER)*dErr;

        double out = AIM_KP*error + AIM_KI*aimIntegral + AIM_KD*aimDFiltered;
        return -clamp(out, -AIM_MAX_TURN, AIM_MAX_TURN);
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