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

@TeleOp(name = "Driver - Hive Aim (Rear Shooter)", group = "Competition")
public class Teleop01 extends OpMode {

    /* ==================== 硬件 ==================== */
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private IMU imu;
    private DcMotorEx shooter0, shooter1;
    private DcMotorEx intakeMotor, transferMotor;
    private Servo smallGateServo, bigGateServo;
    private Servo smallPitchServo1, smallPitchServo2;
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

    /* ==================== 瞄准点（实测后补全） ==================== */
    private static final double[][] AIM_POINTS = {
            {60, 48.0},   // 红方框 1（用户实测）
            //     {49.0, 47.0},   // 红方框 1（用户实测）(60, 48, -90)
            // {95.0, 47.0},   // 红方框 2（待补）
            // {49.0, 97.0},   // 蓝方框 1（待补）
            // {95.0, 97.0},   // 蓝方框 2（待补）
    };

    /* ==================== 自瞄 PID ==================== */
    private static final double AIM_KP = 1.7;
    private static final double AIM_KI = 0;//0.1
    private static final double AIM_KD = 0.1;
    private static final double AIM_MAX_TURN      = 0.8;
    private static final double AIM_TOLERANCE_RAD = Math.toRadians(2.0);
    private static final double AIM_I_MAX   = 1.0;
    private static final double AIM_I_ZONE  = Math.toRadians(30.0);
    private static final double AIM_D_FILTER = 0.7;

    /* ==================== RT 分阶段 ==================== */
    private static final double RT_AIM_THRESHOLD  = 0.50;
    private static final double RT_FIRE_THRESHOLD = 0.85;

    /* ==================== 防抖参数 ==================== */
    private static final double AIM_ENTER_DEG       = 3.0;
    private static final double AIM_EXIT_DEG        = 8.0;
    private static final double RPM_ENTER_ERR       = 100.0;
    private static final double RPM_EXIT_ERR        = 200.0;
    private static final double ENTER_STABLE_TIME   = 0.10;
    private static final double FIRE_ABORT_DEG      = 15.0;
    private static final double FIRE_ABORT_TIME     = 0.30;

    // LT 反转中转层
    private static final double TRANSFER_REVERSE_POWER = -0.35;

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

    /* ==================== 校准数据（★默认值已更新） ==================== */
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

    /* ==================== 状态（★默认值已更新） ==================== */
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

    // 边沿检测
    private boolean prevStart = false, prevY = false, prevA = false;
    private boolean prevDpadUp = false, prevDpadDown = false;
    private boolean prevDpadLeft = false, prevDpadRight = false;
    private boolean prevLB = false, prevRB = false;
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
        shovel_lr = hardwareMap.get(Servo.class, "s00");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        pinpoint = new PinpointXYReader(hardwareMap);
        loadCalibration();
        applyPitch(smallPitchPos, bigPitchPos);

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

        if (currentTargetIdx == 0) {
            targetRPM = calRpmRed;
            smallPitchPos = calSmallPitchRed;
            bigPitchPos   = calBigPitchRed;
        } else {
            targetRPM = calRpmBlue;
            smallPitchPos = calSmallPitchBlue;
            bigPitchPos   = calBigPitchBlue;
        }

        /* ============ 2. RT 分阶段 ============ */
        double rt = gamepad1.right_trigger;
        boolean rtAiming = rt >= RT_AIM_THRESHOLD;
        boolean rtFireRequest = rt >= RT_FIRE_THRESHOLD;

        /* ============ 3. 底盘 + 车尾瞄准 ============ */
        double drive  = deadzone(-gamepad1.left_stick_y);
        double strafe = deadzone( gamepad1.left_stick_x);
        double turn   = deadzone( gamepad1.right_stick_x);

        if (rtAiming && pinpoint != null && pinpointAnchored
                && pinpoint.isPositionValid() && !pinpoint.needsReanchor()) {
            double aimHeading = normalizeAngle(controlHeading + Math.PI);
            turn = computeAimTurn(currentTargetX, currentTargetY, aimHeading, dt);
        } else {
            resetAimPID();
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
        shooter0.setPower(sPower);
        shooter1.setPower(sPower);

        /* ============ 5. 微调 RPM / 俯仰（驾驶员） ============ */
        if (gamepad1.dpad_up && !prevDpadUp)     targetRPM = clamp(targetRPM + 25, MIN_RPM, MAX_RPM);
        if (gamepad1.dpad_down && !prevDpadDown) targetRPM = clamp(targetRPM - 25, MIN_RPM, MAX_RPM);
        prevDpadUp = gamepad1.dpad_up; prevDpadDown = gamepad1.dpad_down;

        double P_STEP = 0.01;
        if (gamepad1.dpad_left && !prevDpadLeft) {
            smallPitchPos = clamp(smallPitchPos - P_STEP, PITCH_MIN, PITCH_MAX); applyPitch(smallPitchPos, bigPitchPos);
        }
        if (gamepad1.dpad_right && !prevDpadRight) {
            smallPitchPos = clamp(smallPitchPos + P_STEP, PITCH_MIN, PITCH_MAX); applyPitch(smallPitchPos, bigPitchPos);
        }
        if (gamepad1.left_bumper && !prevLB) {
            bigPitchPos = clamp(bigPitchPos - P_STEP, PITCH_MIN, PITCH_MAX); applyPitch(smallPitchPos, bigPitchPos);
        }
        if (gamepad1.right_bumper && !prevRB) {
            bigPitchPos = clamp(bigPitchPos + P_STEP, PITCH_MIN, PITCH_MAX); applyPitch(smallPitchPos, bigPitchPos);
        }
        prevDpadLeft = gamepad1.dpad_left; prevDpadRight = gamepad1.dpad_right;
        prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;

        /* ============ 6. 发射锁存状态机 ============ */
        double errDeg = Math.toDegrees(Math.abs(lastAimErrorRad));
        double rpmAbsErr = Math.abs(sErr);

        if (aimAlignedLatched) {
            if (errDeg > AIM_EXIT_DEG) aimAlignedLatched = false;
        } else {
            if (errDeg < AIM_ENTER_DEG) aimAlignedLatched = true;
        }

        if (rpmReadyLatched) {
            if (rpmAbsErr > RPM_EXIT_ERR) rpmReadyLatched = false;
        } else {
            if (rpmAbsErr < RPM_ENTER_ERR) rpmReadyLatched = true;
        }

        if (!firingActive) {
            boolean enterable = rtFireRequest && aimAlignedLatched && rpmReadyLatched
                    && rtAiming && pinpointAnchored;
            if (enterable) {
                stableTimer += dt;
                if (stableTimer >= ENTER_STABLE_TIME) {
                    firingActive = true;
                    stableTimer = 0;
                    fireAbortTimer = 0;
                }
            } else {
                stableTimer = 0;
            }
        } else {
            if (!rtAiming) {
                firingActive = false;
                fireAbortTimer = 0;
            } else if (errDeg > FIRE_ABORT_DEG) {
                fireAbortTimer += dt;
                if (fireAbortTimer > FIRE_ABORT_TIME) {
                    firingActive = false;
                    fireAbortTimer = 0;
                }
            } else {
                fireAbortTimer = 0;
            }
        }

        smallGateServo.setPosition(firingActive ? GATE_SMALL_OPEN : GATE_SMALL_CLOSED);
        bigGateServo.setPosition(firingActive ? GATE_BIG_OPEN   : GATE_BIG_CLOSED);

        double intakePower = 0.0, transferPower = 0.0;
        if (firingActive) {
            intakePower   = 1.0;
            transferPower = 1.0;
        } else if (gamepad1.left_trigger > TRIGGER_DEADZONE) {
            transferPower = TRANSFER_REVERSE_POWER;
            intakePower   = 0.0;
        }

        intakeMotor.setPower(intakePower);
        transferMotor.setPower(transferPower);

        /* ============ 7. START / Y 重锚定 ============ */
        if (gamepad1.start && !prevStart) { imu.resetYaw(); anchorPinpoint(); }
        prevStart = gamepad1.start;
        if (gamepad1.y && !prevY) { imu.resetYaw(); anchorPinpoint(); }
        prevY = gamepad1.y;

        /* ============ 8. A 铲球 ============ */
        if (gamepad1.a && !prevA) shovelHigh = !shovelHigh;
        prevA = gamepad1.a;
        shovel_lr.setPosition(shovelHigh ? 0 : 0.48);

        /* ============ 9. 遥测 ============ */
        telemetry.addData("瞄准点 #", "%d (%.1f, %.1f)", currentTargetIdx, currentTargetX, currentTargetY);
        telemetry.addData("位置", "%.1f, %.1f", px, py);
        telemetry.addData("到目标距离", "%.1f in", bestDist);

        telemetry.addData("RT", "%.2f", rt);
        telemetry.addData("对准°", "%.1f", errDeg);
        telemetry.addData("对准锁存", aimAlignedLatched ? "✅" : "❌");
        telemetry.addData("转速就绪锁存", rpmReadyLatched ? "✅" : "❌");
        telemetry.addData("发射中", firingActive ? "★ 是 ★" : "否");
        telemetry.addData("稳定计时", "%.2f / %.2f", stableTimer, ENTER_STABLE_TIME);

        telemetry.addData("RPM", "%.0f / %.0f", targetRPM, filteredRPM);
        telemetry.addData("小球俯仰", "%.3f", smallPitchPos);
        telemetry.addData("大球俯仰", "%.3f", bigPitchPos);
        telemetry.addData("Intake / Transfer", "%.2f / %.2f", intakePower, transferPower);
        telemetry.addData("闸门", firingActive ? "开/开" : "关/关");

        telemetry.update();
    }

    /* ==================== 工具 ==================== */
    private void applyPitch(double small, double big) {
        smallPitchServo1.setPosition(small);
        smallPitchServo2.setPosition(PITCH_MIRROR ? 1.0 - small : small);
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