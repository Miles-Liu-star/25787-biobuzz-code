package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Headless", group = "Advanced")
public class MilesLiner01 extends OpMode {
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor intake = null;
    private DcMotor middleTurn = null;
    private DcMotorEx shooter0 = null;
    private DcMotorEx shooter1 = null;
    private Servo pitch = null;
    private boolean pitch_model = true;// true为高 false为低
    private boolean was_press_pitch = false;
    private Servo switch_model = null;
    private boolean ball_model = true;// true 为大球 false 为小球
    private boolean was_press_switch = false;
    private IMU imu = null;
    private final ElapsedTime yawPidTimer = new ElapsedTime();

    private static final double DEADBAND = 0.05;
    private static final double ACCEL_SLEW_RATE = 5; // 加速度变化率限制 (单位: power/秒)
    private static final double DECEL_SLEW_RATE = 5.0; // 减速度变化率限制 (单位: power/秒)
    private double lastDrivePower = 0.0;
    private double lastStrafePower = 0.0;
    private double lastTurnPower = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double yawIntegral = 0;
    private double previousYawError = 0;
    private int targetRPM = 1000;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;
    private static final double SHOOTER_KP = 0.0004;
    private static final double SHOOTER_KI = 0.00001;
    private static final double SHOOTER_KD = 0.00000;
    private static final double SHOOTER_KF = 0.0000;
    private static final double SHOOTER_INTEGRAL_LIMIT = 5000.0;
    private final ElapsedTime shooterPidTimer = new ElapsedTime();
    private double shooterIntegral = 0.0;
    private double previousShooterRPM = 0.0;
    private boolean shooterPidInitialized = false;

    @Override
    public void init() {
        LeftFront = hardwareMap.get(DcMotor.class, "0");
        LeftBack = hardwareMap.get(DcMotor.class, "1");
        RightFront = hardwareMap.get(DcMotor.class, "2");
        RightBack = hardwareMap.get(DcMotor.class, "3");
        intake = hardwareMap.get(DcMotor.class, "03");
        middleTurn = hardwareMap.get(DcMotor.class, "02");
        shooter0 = hardwareMap.get(DcMotorEx.class, "00");
        shooter1 = hardwareMap.get(DcMotorEx.class, "01");
        pitch = hardwareMap.get(Servo.class, "s5");
        switch_model = hardwareMap.get(Servo.class, "s00");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        middleTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
        loopTimer.reset();
        shooterPidTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    // shooter代码，使shooter的转速始终约等于targetRPM
    public double FPD() {
        if (targetRPM <= 0) {
            shooterIntegral = 0.0;
            shooterPidInitialized = false;
            shooterPidTimer.reset();
            return 0.0;
        }

        double ticksPerRev = shooter0.getMotorType().getTicksPerRev();
        if (ticksPerRev <= 0.0) {
            telemetry.addData("Shooter PID error", "Invalid encoder ticks per revolution");
            return 0.0;
        }

        double currentRPM = Math.abs(shooter0.getVelocity()) * 60.0 / ticksPerRev;
        double error = targetRPM - currentRPM;
        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();
        if (dt <= 0.0) {
            dt = 0.02;
        }

        double derivative = 0.0;
        if (shooterPidInitialized) {
            derivative = -(currentRPM - previousShooterRPM) / dt;
        }
        previousShooterRPM = currentRPM;
        shooterPidInitialized = true;

        shooterIntegral = Range.clip(
                shooterIntegral + error * dt,
                -SHOOTER_INTEGRAL_LIMIT,
                SHOOTER_INTEGRAL_LIMIT);

        double feedForward = SHOOTER_KF * targetRPM;
        double power = feedForward
                + SHOOTER_KP * error
                + SHOOTER_KI * shooterIntegral
                + SHOOTER_KD * derivative;
        power = Range.clip(power, 0.0, 1.0);

        // telemetry.addData("Shooter actual RPM", "%.0f", currentRPM);
        // telemetry.addData("Shooter power", "%.3f", power);
        return power;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > DEADBAND ? value : 0.0;
    }

    public void setDrivePower() {
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawTurn = gamepad1.right_stick_x;

        double y = applyDeadband(rawY);
        double x = applyDeadband(rawX);
        double turn = applyDeadband(rawTurn);

        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        turn = Math.pow(turn, 3);

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double headingOffset = -botHeading;
        double rotatedX = x * Math.cos(headingOffset) - y * Math.sin(headingOffset);
        double rotatedY = x * Math.sin(headingOffset) + y * Math.cos(headingOffset);
        double targetDrive = rotatedY;
        double targetStrafe = rotatedX;
        double targetTurn = turn;

        double driveSlewRate = Math.abs(targetDrive) > Math.abs(lastDrivePower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;
        double strafeSlewRate = Math.abs(targetStrafe) > Math.abs(lastStrafePower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;
        double turnSlewRate = Math.abs(targetTurn) > Math.abs(lastTurnPower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;

        double elapsedTime = loopTimer.seconds();
        loopTimer.reset();

        double driveMaxChange = driveSlewRate * elapsedTime;
        double strafeMaxChange = strafeSlewRate * elapsedTime;
        double turnMaxChange = turnSlewRate * elapsedTime;

        double currentDrive = lastDrivePower
                + Range.clip(targetDrive - lastDrivePower, -driveMaxChange, driveMaxChange);
        double currentStrafe = lastStrafePower
                + Range.clip(targetStrafe - lastStrafePower, -strafeMaxChange, strafeMaxChange);
        double currentTurn = lastTurnPower
                + Range.clip(targetTurn - lastTurnPower, -turnMaxChange, turnMaxChange);

        lastDrivePower = currentDrive;
        lastStrafePower = currentStrafe;
        lastTurnPower = currentTurn;

        double frontLeftPower = currentDrive + currentStrafe + currentTurn;
        double backLeftPower = currentDrive - currentStrafe + currentTurn;
        double frontRightPower = currentDrive - currentStrafe - currentTurn;
        double backRightPower = currentDrive + currentStrafe - currentTurn;

        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        telemetry.addData("frontLeftPower", "%5.1f", frontLeftPower);
        telemetry.addData("frontRightPower", "%5.1f", frontRightPower);
        telemetry.addData("backLeftPower", "%5.1f", backLeftPower);
        telemetry.addData("backRightPower", "%5.1f", backRightPower);

        LeftFront.setPower(frontLeftPower);
        LeftBack.setPower(backLeftPower);
        RightFront.setPower(frontRightPower);
        RightBack.setPower(backRightPower);
    }

    @Override
    public void loop() {

        if (gamepad1.y) { // 按手柄的Y键重置IMU
            imu.resetYaw();
        }

        // intake
        if (gamepad1.right_bumper) {
            intake.setPower(1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        // shooter
        if (gamepad1.right_trigger > 0.5) {
            double shooterPower = FPD();
            shooter0.setPower(shooterPower);
            shooter1.setPower(shooterPower);
            // telemetry.addData("shooterPower : ","%.2f",shooterPower);
        } else {
            shooter0.setPower(0);
            shooter1.setPower(0);
            shooterIntegral = 0.0;
            previousShooterRPM = 0.0;
            shooterPidInitialized = false;
            shooterPidTimer.reset();
        }
        if (gamepad1.dpad_up && !wasDpadUpPressed) {
            targetRPM += 200;
        }
        if (gamepad1.dpad_down && !wasDpadDownPressed) {
            targetRPM = Math.max(0, targetRPM - 200);
        }
        if (!gamepad1.dpad_up) {
            wasDpadUpPressed = false;
        }
        if (!gamepad1.dpad_down) {
            wasDpadDownPressed = false;
        }
         telemetry.addData("Shooter target RPM", targetRPM);
         telemetry.addData("shooter velocity : ","%.2f",shooter0.getVelocity());

        // 俯仰pitch
        // pitch_model true为高 false为低
        if (gamepad1.a && !was_press_pitch) {
            pitch_model = !pitch_model;
            was_press_pitch = true;
        }
        if (!gamepad1.a) {
            was_press_pitch = false;
        }
        if (pitch_model) {
            pitch.setPosition(0);
        } else {
            pitch.setPosition(0.47);
        }

        // 大小球切换 ball_model true->大球 false->小球
        if (gamepad1.x && !was_press_switch) {
            ball_model = !ball_model;
            was_press_switch = true;
        }
        if (!gamepad1.x) {
            was_press_switch = false;
        }
        if (ball_model) {
            switch_model.setPosition(0.5);
        } else {
            switch_model.setPosition(1);
        }

        // midturn
        middleTurn.setPower(-gamepad1.left_trigger);

        setDrivePower();
    }

    @Override
    public void stop() {
        if (LeftFront != null)
            LeftFront.setPower(0);
        if (LeftBack != null)
            LeftBack.setPower(0);
        if (RightFront != null)
            RightFront.setPower(0);
        if (RightBack != null)
            RightBack.setPower(0);
        if (intake != null)
            intake.setPower(0);
        if (shooter0 != null)
            shooter0.setPower(0);
        if (shooter1 != null)
            shooter1.setPower(0);
        if (middleTurn != null)
            middleTurn.setPower(0);
    }
}